"""Contact and insertion metrics publisher for Research Baseline v0.7."""

from __future__ import annotations

import importlib
import json
import math
import re
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_CONTACT_TOPICS = (
    {"name": "peg", "topic": "/gazebo/contacts/peg"},
    {"name": "hole", "topic": "/gazebo/contacts/hole"},
    {"name": "target", "topic": "/gazebo/contacts/target"},
    {"name": "validation", "topic": "/gazebo/contacts/validation"},
    {
        "name": "robot_validation",
        "topic": (
            "/world/peg_in_hole_robot_contact_validation_world/model/"
            "robot_contact_validation_pad/link/robot_contact_validation_pad_link/"
            "sensor/robot_contact_validation_sensor/contact"
        ),
    },
    {
        "name": "peg_validation",
        "topic": (
            "/world/peg_in_hole_insertion_validation_world/model/peg/link/"
            "peg_link/sensor/peg_contact_sensor/contact"
        ),
    },
    {
        "name": "hole_validation",
        "topic": (
            "/world/peg_in_hole_insertion_validation_world/model/hole_block/link/"
            "hole_block_link/sensor/hole_contact_sensor/contact"
        ),
    },
)

FORCE_EXTRACTION_METHOD = "ros_gz_interfaces Contacts.wrenches force magnitude"


def extract_max_contact_force(contacts_msg: Any) -> Optional[float]:
    """Return the maximum force-vector magnitude in a Contacts message."""
    max_force: float | None = None
    for contact in _extract_contacts_from_message(contacts_msg):
        for wrench in _iter_contact_wrenches(contact):
            for vector in _iter_wrench_force_vectors(wrench):
                magnitude = _vector_magnitude(vector)
                if magnitude is not None:
                    max_force = (
                        magnitude if max_force is None else max(max_force, magnitude)
                    )
    return max_force


def _extract_contacts_from_message(message: Any) -> list[Any]:
    contacts = getattr(message, "contacts", None)
    if contacts is None:
        contacts = getattr(message, "contact", None)
    if contacts is None:
        return []
    try:
        return list(contacts)
    except TypeError:
        return []


def _iter_contact_wrenches(contact: Any) -> list[Any]:
    result = []
    for field_name in (
        "wrenches",
        "wrench",
        "body_1_wrench",
        "body_2_wrench",
        "body1_wrench",
        "body2_wrench",
    ):
        wrenches = getattr(contact, field_name, None)
        if wrenches is None:
            continue
        try:
            result.extend(list(wrenches))
        except TypeError:
            result.append(wrenches)
    return result


def _iter_wrench_force_vectors(wrench: Any) -> list[Any]:
    vectors = []
    for field_name in (
        "force",
        "body_1_force",
        "body_2_force",
        "body1_force",
        "body2_force",
    ):
        vector = getattr(wrench, field_name, None)
        if vector is not None:
            vectors.append(vector)
    for field_name in (
        "body_1_wrench",
        "body_2_wrench",
        "body1_wrench",
        "body2_wrench",
    ):
        nested_wrench = getattr(wrench, field_name, None)
        vector = getattr(nested_wrench, "force", None)
        if vector is not None:
            vectors.append(vector)
    return vectors


def _vector_magnitude(vector: Any) -> float | None:
    if vector is None:
        return None
    try:
        x = float(getattr(vector, "x"))
        y = float(getattr(vector, "y"))
        z = float(getattr(vector, "z"))
        return math.sqrt(x * x + y * y + z * z)
    except (AttributeError, TypeError, ValueError):
        return None


class ContactMetricsNode(Node):
    """Convert contact messages and task phases into trial metrics JSON."""

    def __init__(self) -> None:
        super().__init__("contact_metrics_node")
        self.declare_parameter(
            "contact_topics",
            [f"{entry['name']}:{entry['topic']}" for entry in DEFAULT_CONTACT_TOPICS],
        )
        self.declare_parameter("insertion_phase_name", "insertion_hold")
        self.declare_parameter("contact_enabled", True)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("max_contact_force_available", True)
        self.declare_parameter("zero_contact_event_throttle_sec", 2.0)
        self.declare_parameter("contact_event_debounce_sec", 0.25)
        self.declare_parameter("positive_contact_event_throttle_sec", 0.2)
        self.declare_parameter("contact_force_update_epsilon", 1.0e-6)
        self.declare_parameter("physical_contact_sources", [""])
        self.declare_parameter("robot_validation_warning_force_n", 50.0)
        self.declare_parameter("robot_validation_violation_force_n", 100.0)

        self._contact_topics = self._load_contact_topics()
        self._insertion_phase_name = (
            self.get_parameter("insertion_phase_name").get_parameter_value().string_value
        )
        self._contact_enabled = (
            self.get_parameter("contact_enabled").get_parameter_value().bool_value
        )
        self._publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._max_contact_force_available = (
            self.get_parameter("max_contact_force_available")
            .get_parameter_value()
            .bool_value
        )
        self._zero_contact_event_throttle_sec = max(
            0.0,
            self.get_parameter("zero_contact_event_throttle_sec")
            .get_parameter_value()
            .double_value,
        )
        self._contact_event_debounce_sec = max(
            0.0,
            self.get_parameter("contact_event_debounce_sec")
            .get_parameter_value()
            .double_value,
        )
        self._positive_contact_event_throttle_sec = max(
            0.0,
            self.get_parameter("positive_contact_event_throttle_sec")
            .get_parameter_value()
            .double_value,
        )
        self._contact_force_update_epsilon = max(
            0.0,
            self.get_parameter("contact_force_update_epsilon")
            .get_parameter_value()
            .double_value,
        )
        self._physical_contact_sources = self._load_physical_contact_sources()
        self._robot_validation_warning_force_n = max(
            0.0,
            self.get_parameter("robot_validation_warning_force_n")
            .get_parameter_value()
            .double_value,
        )
        self._robot_validation_violation_force_n = max(
            0.0,
            self.get_parameter("robot_validation_violation_force_n")
            .get_parameter_value()
            .double_value,
        )

        self._current_phase = "uninitialized"
        self._trial_status = "idle"
        self._explicit_failure = False
        self._insertion_attempted = False
        self._insertion_hold_reached = False
        self._contact_events_count = 0
        self._contact_episode_count = 0
        self._contact_samples_count = 0
        self._max_contact_force: float | None = None
        self._force_extraction_available = False
        self._contact_message_type_available = False
        self._contact_messages_observed = False
        self._physical_contact_observed = False
        self._collision_pairs: list[str] = []
        self._collision_pair_set: set[str] = set()
        self._first_collision1: str | None = None
        self._first_collision2: str | None = None
        self._peg_contact_observed = False
        self._hole_contact_observed = False
        self._peg_table_contact_count = 0
        self._peg_hole_contact_count = 0
        self._peg_hole_collision_pairs: list[str] = []
        self._peg_hole_collision_pair_set: set[str] = set()
        self._non_insertion_contact_pairs: list[str] = []
        self._non_insertion_contact_pair_set: set[str] = set()
        self._initial_contact_detected = False
        self._initial_contact_pairs: list[str] = []
        self._initial_contact_pair_set: set[str] = set()
        self._uninitialized_contact_count = 0
        self._first_peg_hole_contact_phase: str | None = None
        self._first_peg_table_contact_phase: str | None = None
        self._positive_contact_counts = {name: 0 for name in self._contact_topics}
        self._max_contact_force_by_source: dict[str, float | None] = {
            name: None for name in self._contact_topics
        }
        self._contact_topic_seen = {name: False for name in self._contact_topics}
        self._previous_in_contact = {name: False for name in self._contact_topics}
        self._last_contact_transition_event_sec = {
            name: None for name in self._contact_topics
        }
        self._last_zero_contact_event_sec = {
            name: None for name in self._contact_topics
        }
        self._last_positive_contact_event_sec = {
            name: None for name in self._contact_topics
        }
        self._last_reported_contact_force = {
            name: None for name in self._contact_topics
        }
        self._warned_missing_publishers = False

        self._contact_event_pub = self.create_publisher(String, "/contact_event", 100)
        self._force_guard_status_pub = self.create_publisher(
            String, "/force_guard_status", 100
        )
        self._insertion_metrics_pub = self.create_publisher(
            String, "/insertion_metrics", 10
        )

        self.create_subscription(String, "/task_phase", self._on_task_phase, 100)
        self.create_subscription(String, "/trial_status", self._on_trial_status, 100)
        self._create_contact_subscriptions()

        rate_hz = max(self._publish_rate_hz, 0.1)
        self.create_timer(1.0 / rate_hz, self._publish_metrics)
        self.create_timer(10.0, self._warn_if_contact_topics_missing)

        self.get_logger().info(
            "Contact metrics node publishing /contact_event, /force_guard_status, "
            "and /insertion_metrics."
        )

    def _load_contact_topics(self) -> dict[str, str]:
        """Load contact topic specs while accepting legacy string-only configs."""
        parameter_value = self.get_parameter("contact_topics").get_parameter_value()
        entries = list(parameter_value.string_array_value)
        if not entries:
            entries = [
                f"{entry['name']}:{entry['topic']}" for entry in DEFAULT_CONTACT_TOPICS
            ]

        topics: dict[str, str] = {}
        for index, entry in enumerate(entries):
            name, topic = self._parse_contact_topic_entry(str(entry), index)
            topics[name] = topic
        return topics

    def _parse_contact_topic_entry(self, entry: str, index: int) -> tuple[str, str]:
        stripped = entry.strip()
        if not stripped:
            default_entry = DEFAULT_CONTACT_TOPICS[index % len(DEFAULT_CONTACT_TOPICS)]
            return str(default_entry["name"]), str(default_entry["topic"])

        try:
            parsed = json.loads(stripped)
        except json.JSONDecodeError:
            parsed = None
        if isinstance(parsed, dict):
            name = str(parsed.get("name", "")).strip()
            topic = str(parsed.get("topic", "")).strip()
            if name and topic:
                return name, topic

        if ":" in stripped:
            name, topic = stripped.split(":", 1)
            name = name.strip()
            topic = topic.strip()
            if name and topic:
                return name, topic

        source = self._source_from_topic(stripped, index)
        return source, stripped

    def _load_physical_contact_sources(self) -> set[str]:
        values = list(
            self.get_parameter("physical_contact_sources")
            .get_parameter_value()
            .string_array_value
        )
        return {str(value).strip() for value in values if str(value).strip()}

    def _create_contact_subscriptions(self) -> None:
        if not self._contact_enabled:
            self.get_logger().warning(
                "Contact metrics are disabled by configuration; continuing without "
                "contact subscriptions."
            )
            return

        try:
            contacts_msg = importlib.import_module("ros_gz_interfaces.msg").Contacts
        except (AttributeError, ImportError) as exc:
            self.get_logger().warning(
                "ros_gz_interfaces/msg/Contacts is unavailable; contact topics will "
                f"not be subscribed, but insertion metrics will continue. Detail: {exc}"
            )
            return

        self._contact_message_type_available = True
        for source, topic in self._contact_topics.items():
            self.create_subscription(
                contacts_msg,
                topic,
                lambda message, source=source: self._on_contacts(source, message),
                100,
            )
            self.get_logger().info(
                f"Subscribed to contact topic {topic} as source '{source}'"
            )

    def _on_task_phase(self, message: String) -> None:
        phase = message.data.strip() or "empty_phase"
        self._current_phase = phase
        if "insertion_hold" in phase or self._insertion_phase_name in phase:
            self._insertion_attempted = True
            self._insertion_hold_reached = True
        elif "insertion_approach" in phase:
            self._insertion_attempted = True

    def _on_trial_status(self, message: String) -> None:
        self._trial_status = message.data.strip() or "empty_status"
        if self._trial_status.lower() in {
            "failed",
            "failed_pre_contact",
            "aborted",
            "canceled",
            "cancelled",
            "guarded_stop",
            "timeout",
            "timed_out",
        }:
            self._explicit_failure = True

    def _on_contacts(self, source: str, message: Any) -> None:
        contacts = self._extract_contacts(message)
        contact_count = len(contacts)
        collision_pairs = self._extract_collision_pairs(contacts)
        first_collision1, first_collision2 = (
            collision_pairs[0] if collision_pairs else (None, None)
        )
        self._contact_messages_observed = True
        self._contact_topic_seen[source] = True

        max_force = self._extract_max_force(message)
        previous_max_force = self._max_contact_force
        if max_force is not None:
            self._force_extraction_available = True
            self._max_contact_force = (
                max_force
                if self._max_contact_force is None
                else max(self._max_contact_force, max_force)
            )
            source_force = self._max_contact_force_by_source.get(source)
            self._max_contact_force_by_source[source] = (
                max_force if source_force is None else max(source_force, max_force)
            )
        new_force_peak = (
            max_force is not None
            and (
                previous_max_force is None
                or max_force > previous_max_force + self._contact_force_update_epsilon
            )
        )

        if contact_count > 0:
            self._contact_samples_count += 1
            self._positive_contact_counts[source] = (
                self._positive_contact_counts.get(source, 0) + 1
            )
            if self._current_phase == "uninitialized" and not collision_pairs:
                self._initial_contact_detected = True
                self._uninitialized_contact_count += contact_count
            self._classify_collision_pairs(collision_pairs)
            if self._counts_as_physical_contact(source):
                self._physical_contact_observed = True
                self._record_collision_pairs(collision_pairs)
            self._publish_force_guard_status(source, contact_count, max_force)
            if not self._previous_in_contact.get(source, False):
                self._previous_in_contact[source] = True
                if self._counts_as_physical_contact(source):
                    self._contact_episode_count += 1
                    self._contact_events_count = self._contact_episode_count
                if self._should_publish_transition_event(source):
                    self._record_positive_contact_event(source, max_force)
                    self._publish_contact_event(
                        self._contact_event_payload(
                            event_type="contact_started",
                            source=source,
                            contact_count=contact_count,
                            max_force=max_force,
                            collision_pairs=collision_pairs,
                            first_collision1=first_collision1,
                            first_collision2=first_collision2,
                            message=self._contact_note(contact_count, max_force),
                        )
                    )
            else:
                self._previous_in_contact[source] = True
                if self._should_publish_positive_contact_event(source, max_force):
                    self._publish_contact_event(
                        self._contact_event_payload(
                            event_type="contact_updated",
                            source=source,
                            contact_count=contact_count,
                            max_force=max_force,
                            collision_pairs=collision_pairs,
                            first_collision1=first_collision1,
                            first_collision2=first_collision2,
                            message=self._contact_note(contact_count, max_force),
                        )
                    )
            if new_force_peak:
                self._publish_metrics()
            return

        self._publish_force_guard_status(source, contact_count, max_force)

        if self._previous_in_contact.get(source, False):
            if self._should_publish_transition_event(source):
                self._previous_in_contact[source] = False
                self._publish_contact_event(
                    self._contact_event_payload(
                        event_type="contact_ended",
                        source=source,
                        contact_count=contact_count,
                        max_force=max_force,
                        collision_pairs=collision_pairs,
                        first_collision1=first_collision1,
                        first_collision2=first_collision2,
                        message="Contact ended; no contacts detected.",
                    )
                )
            return

        if self._should_publish_zero_contact_event(source):
            self._publish_contact_event(
                self._contact_event_payload(
                    event_type="no_contact",
                    source=source,
                    contact_count=contact_count,
                    max_force=max_force,
                    collision_pairs=collision_pairs,
                    first_collision1=first_collision1,
                    first_collision2=first_collision2,
                    message="Contact topic message received; no contacts detected.",
                )
            )

    def _contact_note(self, contact_count: int, max_force: float | None) -> str:
        if contact_count == 0:
            return "Contact topic message received; no contacts detected."
        if max_force is None:
            note = "Contact observed; force value unavailable in parsed message."
        else:
            note = (
                "Contact observed; max_contact_force parsed from Contacts.wrenches "
                "force vectors."
            )
        return note

    def _contact_event_payload(
        self,
        *,
        event_type: str,
        source: str,
        contact_count: int,
        max_force: float | None,
        collision_pairs: list[tuple[str, str]],
        first_collision1: str | None,
        first_collision2: str | None,
        message: str,
    ) -> dict[str, Any]:
        return {
            "timestamp_ros_sec": self._now_sec(),
            "event_type": event_type,
            "phase": self._current_phase,
            "source": source,
            "contact_count": contact_count,
            "max_contact_force": max_force,
            "collision_pairs": self._format_collision_pairs(collision_pairs),
            "first_collision1": first_collision1,
            "first_collision2": first_collision2,
            "message": message,
        }

    def _extract_contacts(self, message: Any) -> list[Any]:
        return _extract_contacts_from_message(message)

    def _extract_collision_pairs(self, contacts: list[Any]) -> list[tuple[str, str]]:
        pairs: list[tuple[str, str]] = []
        seen: set[str] = set()
        for contact in contacts:
            collision1 = self._collision_name(contact, "collision1")
            collision2 = self._collision_name(contact, "collision2")
            if not collision1 and not collision2:
                continue
            key = self._collision_pair_key(collision1, collision2)
            if key in seen:
                continue
            seen.add(key)
            pairs.append((collision1, collision2))
        return pairs

    @staticmethod
    def _collision_name(contact: Any, field_name: str) -> str:
        collision = getattr(contact, field_name, None)
        name = getattr(collision, "name", None)
        if name is None:
            return ""
        return str(name)

    @staticmethod
    def _collision_pair_key(collision1: str, collision2: str) -> str:
        return f"{collision1}|{collision2}"

    def _record_collision_pairs(self, collision_pairs: list[tuple[str, str]]) -> None:
        for collision1, collision2 in collision_pairs:
            key = self._collision_pair_key(collision1, collision2)
            if key in self._collision_pair_set:
                continue
            self._collision_pair_set.add(key)
            self._collision_pairs.append(key)
            if self._first_collision1 is None and self._first_collision2 is None:
                self._first_collision1 = collision1
                self._first_collision2 = collision2

    def _classify_collision_pairs(
        self, collision_pairs: list[tuple[str, str]]
    ) -> None:
        for collision1, collision2 in collision_pairs:
            key = self._collision_pair_key(collision1, collision2)
            peg1 = self._is_peg_collision(collision1)
            peg2 = self._is_peg_collision(collision2)
            hole1 = self._is_hole_collision(collision1)
            hole2 = self._is_hole_collision(collision2)
            table1 = self._is_table_collision(collision1)
            table2 = self._is_table_collision(collision2)

            if self._current_phase == "uninitialized":
                self._initial_contact_detected = True
                self._uninitialized_contact_count += 1
                self._record_unique_pair(
                    key,
                    self._initial_contact_pair_set,
                    self._initial_contact_pairs,
                )

            if peg1 or peg2:
                self._peg_contact_observed = True
            if hole1 or hole2:
                self._hole_contact_observed = True

            if (peg1 and hole2) or (peg2 and hole1):
                if self._current_phase == "uninitialized":
                    self._record_unique_pair(
                        key,
                        self._non_insertion_contact_pair_set,
                        self._non_insertion_contact_pairs,
                    )
                    continue
                self._peg_hole_contact_count += 1
                self._record_unique_pair(
                    key,
                    self._peg_hole_collision_pair_set,
                    self._peg_hole_collision_pairs,
                )
                if self._first_peg_hole_contact_phase is None:
                    self._first_peg_hole_contact_phase = self._current_phase
                continue

            if (peg1 and table2) or (peg2 and table1):
                self._peg_table_contact_count += 1
                if self._first_peg_table_contact_phase is None:
                    self._first_peg_table_contact_phase = self._current_phase

            self._record_unique_pair(
                key,
                self._non_insertion_contact_pair_set,
                self._non_insertion_contact_pairs,
            )

    @staticmethod
    def _record_unique_pair(key: str, seen: set[str], pairs: list[str]) -> None:
        if key in seen:
            return
        seen.add(key)
        pairs.append(key)

    @staticmethod
    def _is_peg_collision(collision: str) -> bool:
        normalized = collision.lower()
        return (
            "peg::" in normalized
            or "peg_link" in normalized
            or "peg_collision" in normalized
        )

    @staticmethod
    def _is_hole_collision(collision: str) -> bool:
        normalized = collision.lower()
        return (
            "hole_block::" in normalized
            or "hole_block" in normalized
            or "hole_contact" in normalized
            or "hole_collision" in normalized
        )

    @staticmethod
    def _is_table_collision(collision: str) -> bool:
        normalized = collision.lower()
        return "work_table" in normalized or "table_collision" in normalized

    @classmethod
    def _format_collision_pairs(cls, collision_pairs: list[tuple[str, str]]) -> list[str]:
        return [
            cls._collision_pair_key(collision1, collision2)
            for collision1, collision2 in collision_pairs
        ]

    def _should_publish_zero_contact_event(self, source: str) -> bool:
        now_sec = self._now_sec()
        last_event_sec = self._last_zero_contact_event_sec.get(source)
        if (
            last_event_sec is not None
            and now_sec - last_event_sec < self._zero_contact_event_throttle_sec
        ):
            return False
        self._last_zero_contact_event_sec[source] = now_sec
        return True

    def _should_publish_transition_event(self, source: str) -> bool:
        now_sec = self._now_sec()
        last_event_sec = self._last_contact_transition_event_sec.get(source)
        if (
            last_event_sec is not None
            and now_sec - last_event_sec < self._contact_event_debounce_sec
        ):
            return False
        self._last_contact_transition_event_sec[source] = now_sec
        return True

    def _record_positive_contact_event(
        self, source: str, max_force: float | None
    ) -> None:
        self._last_positive_contact_event_sec[source] = self._now_sec()
        if max_force is not None:
            self._last_reported_contact_force[source] = max_force

    def _should_publish_positive_contact_event(
        self, source: str, max_force: float | None
    ) -> bool:
        now_sec = self._now_sec()
        last_event_sec = self._last_positive_contact_event_sec.get(source)
        throttle_elapsed = (
            last_event_sec is None
            or now_sec - last_event_sec >= self._positive_contact_event_throttle_sec
        )
        if not throttle_elapsed:
            return False
        self._record_positive_contact_event(source, max_force)
        return True

    def _extract_max_force(self, message: Any) -> float | None:
        if not self._max_contact_force_available:
            return None
        return extract_max_contact_force(message)

    def _publish_contact_event(self, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._contact_event_pub.publish(message)

    def _publish_force_guard_status(
        self,
        source: str,
        contact_count: int,
        max_force: float | None,
    ) -> None:
        payload = {
            "timestamp_ros_sec": self._now_sec(),
            "source": source,
            "contact_count": contact_count,
            "physical_contact_observed": (
                contact_count > 0 and self._counts_as_physical_contact(source)
            ),
            "max_contact_force": max_force,
            "collision_pairs": list(self._collision_pairs),
            "first_collision1": self._first_collision1,
            "first_collision2": self._first_collision2,
            "force_extraction_available": max_force is not None,
            "force_threshold_warning": self._force_threshold_warning(max_force),
            "force_threshold_violation": self._force_threshold_violation(max_force),
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._force_guard_status_pub.publish(message)

    def _publish_metrics(self) -> None:
        notes = []
        contact_topics_connected = self._contact_topics_connected()
        contact_metrics_available = bool(contact_topics_connected)
        contact_topics_seen = [
            source for source, seen in self._contact_topic_seen.items() if seen
        ]
        peg_contact_observed = self._peg_contact_observed
        hole_contact_observed = self._hole_contact_observed
        peg_hole_contact_observed = self._peg_hole_contact_count > 0
        peg_table_contact_observed = self._peg_table_contact_count > 0
        force_threshold_violation = self._force_threshold_violation(
            self._max_contact_force
        )
        if not self._contact_message_type_available:
            notes.append("Contact message type unavailable; no contact subscriptions active.")
        if not contact_topics_connected:
            notes.append("No ROS publishers are currently visible for configured contact topics.")
        elif not self._contact_messages_observed:
            notes.append(
                "Contact topics are connected but no contact messages were observed."
            )
        elif not self._physical_contact_observed:
            notes.append("Contact messages observed but no physical contacts.")
        else:
            notes.append("Physical contacts observed.")
        if peg_table_contact_observed and not peg_hole_contact_observed:
            notes.append(
                "Peg contact was observed against the table, not the hole; "
                "insertion contact was not validated."
            )
        if self._initial_contact_detected:
            notes.append(
                "Initial contact was observed during the uninitialized phase; "
                "clean_initial_state is false."
            )
        if not self._max_contact_force_available:
            notes.append(
                "max_contact_force is null because force extraction is "
                "disabled/unvalidated."
            )
        elif not self._force_extraction_available:
            notes.append(
                "Force extraction is enabled, but no Contacts.wrenches force vector "
                "has been observed yet."
            )
        if self._force_threshold_violation(self._max_contact_force):
            notes.append(
                "Robot validation contact force exceeded the configured simulation "
                "violation threshold."
            )
        elif self._force_threshold_warning(self._max_contact_force):
            notes.append(
                "Robot validation contact force exceeded the configured simulation "
                "warning threshold."
            )
        notes.append(
            "insertion_success remains null until a validated success rule is "
            "implemented."
        )
        notes.append(
            "insertion_success_estimate is a heuristic based on insertion_hold, "
            "peg/hole contact observation, accepted final trial status, and absence "
            "of force threshold violation."
        )

        payload = {
            "timestamp_ros_sec": self._now_sec(),
            "current_phase": self._current_phase,
            "trial_status": self._trial_status,
            "insertion_attempted": self._insertion_attempted,
            "insertion_hold_reached": self._insertion_hold_reached,
            "contact_topics_configured": self._contact_topics,
            "contact_topics_connected": contact_topics_connected,
            "contact_messages_observed": self._contact_messages_observed,
            "physical_contact_observed": self._physical_contact_observed,
            "collision_pairs": list(self._collision_pairs),
            "first_collision1": self._first_collision1,
            "first_collision2": self._first_collision2,
            "contact_topics_seen": contact_topics_seen,
            "positive_contact_counts": self._positive_contact_counts,
            "contact_events_count": self._contact_events_count,
            "contact_episode_count": self._contact_episode_count,
            "contact_samples_count": self._contact_samples_count,
            "max_contact_force": self._max_contact_force,
            "peg_contact_observed": peg_contact_observed,
            "hole_contact_observed": hole_contact_observed,
            "peg_table_contact_observed": peg_table_contact_observed,
            "peg_hole_contact_observed": peg_hole_contact_observed,
            "peg_hole_contact_count": self._peg_hole_contact_count,
            "peg_table_contact_count": self._peg_table_contact_count,
            "first_peg_hole_contact_phase": self._first_peg_hole_contact_phase,
            "first_peg_table_contact_phase": self._first_peg_table_contact_phase,
            "peg_hole_collision_pairs": list(self._peg_hole_collision_pairs),
            "non_insertion_contact_pairs": list(self._non_insertion_contact_pairs),
            "initial_contact_detected": self._initial_contact_detected,
            "initial_contact_pairs": list(self._initial_contact_pairs),
            "uninitialized_contact_count": self._uninitialized_contact_count,
            "clean_initial_state": not self._initial_contact_detected,
            "max_peg_contact_force": self._max_force_for_sources(
                self._peg_contact_sources()
            ),
            "max_hole_contact_force": self._max_force_for_sources(
                self._hole_contact_sources()
            ),
            "insertion_depth_estimate": None,
            "insertion_depth_available": False,
            "insertion_phase": self._current_phase,
            "robot_validation_warning_force_n": self._robot_validation_warning_force_n,
            "robot_validation_violation_force_n": (
                self._robot_validation_violation_force_n
            ),
            "force_threshold_warning": self._force_threshold_warning(
                self._max_contact_force
            ),
            "force_threshold_violation": force_threshold_violation,
            "force_extraction_available": self._force_extraction_available,
            "force_extraction_method": FORCE_EXTRACTION_METHOD,
            "insertion_success": None,
            "insertion_success_estimate": self._insertion_success_estimate(
                peg_hole_contact_observed=peg_hole_contact_observed,
                force_threshold_violation=force_threshold_violation,
            ),
            "contact_metrics_available": contact_metrics_available,
            "notes": " ".join(notes),
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._insertion_metrics_pub.publish(message)

    def _contact_topics_connected(self) -> list[str]:
        return [
            source
            for source, topic in self._contact_topics.items()
            if self.get_publishers_info_by_topic(topic)
        ]

    def _warn_if_contact_topics_missing(self) -> None:
        if self._warned_missing_publishers or not self._contact_enabled:
            return
        missing_topics = [
            topic
            for topic in self._contact_topics.values()
            if not self.get_publishers_info_by_topic(topic)
        ]
        if missing_topics:
            self.get_logger().warning(
                "No publishers currently visible for contact topic(s): "
                f"{', '.join(missing_topics)}. Continuing without failing the trial."
            )
            self._warned_missing_publishers = True

    def _insertion_success_estimate(
        self,
        *,
        peg_hole_contact_observed: bool,
        force_threshold_violation: bool,
    ) -> bool | None:
        if self._explicit_failure:
            return False
        if (
            self._insertion_hold_reached
            and peg_hole_contact_observed
            and not force_threshold_violation
            and self._trial_status in {"completed", "guarded_contact_stop"}
        ):
            return True
        return False

    @staticmethod
    def _peg_contact_sources() -> set[str]:
        return {"peg", "peg_validation"}

    @staticmethod
    def _hole_contact_sources() -> set[str]:
        return {"hole", "hole_validation"}

    def _contact_observed_for_sources(self, sources: set[str]) -> bool:
        return any(
            self._positive_contact_counts.get(source, 0) > 0 for source in sources
        )

    def _max_force_for_sources(self, sources: set[str]) -> float | None:
        forces = [
            force
            for source, force in self._max_contact_force_by_source.items()
            if source in sources and force is not None
        ]
        return max(forces) if forces else None

    def _counts_as_physical_contact(self, source: str) -> bool:
        if not self._physical_contact_sources:
            return True
        return source in self._physical_contact_sources

    def _force_threshold_warning(self, max_force: float | None) -> bool:
        return (
            max_force is not None
            and max_force > self._robot_validation_warning_force_n
        )

    def _force_threshold_violation(self, max_force: float | None) -> bool:
        return (
            max_force is not None
            and max_force > self._robot_validation_violation_force_n
        )

    @staticmethod
    def _source_from_topic(topic: str, index: int) -> str:
        for source in (
            "robot_validation",
            "peg_validation",
            "hole_validation",
            "peg",
            "hole",
            "target",
            "validation",
        ):
            if re.search(rf"(^|/){source}($|/)", topic):
                return source
        return f"contact_{index}"

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ContactMetricsNode | None = None
    try:
        node = ContactMetricsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
