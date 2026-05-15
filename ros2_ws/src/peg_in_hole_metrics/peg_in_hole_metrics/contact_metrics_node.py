"""Contact and insertion metrics publisher for Research Baseline v0.4."""

from __future__ import annotations

import importlib
import json
import math
import re
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_CONTACT_TOPICS = (
    {"name": "peg", "topic": "/gazebo/contacts/peg"},
    {"name": "hole", "topic": "/gazebo/contacts/hole"},
    {"name": "target", "topic": "/gazebo/contacts/target"},
    {"name": "validation", "topic": "/gazebo/contacts/validation"},
)


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
        self.declare_parameter("max_contact_force_available", False)
        self.declare_parameter("zero_contact_event_throttle_sec", 2.0)
        self.declare_parameter("contact_event_debounce_sec", 0.25)

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

        self._current_phase = "uninitialized"
        self._trial_status = "idle"
        self._explicit_failure = False
        self._insertion_attempted = False
        self._insertion_hold_reached = False
        self._contact_events_count = 0
        self._contact_samples_count = 0
        self._max_contact_force: float | None = None
        self._contact_message_type_available = False
        self._contact_messages_observed = False
        self._physical_contact_observed = False
        self._contact_topic_seen = {name: False for name in self._contact_topics}
        self._previous_in_contact = {name: False for name in self._contact_topics}
        self._last_contact_transition_event_sec = {
            name: None for name in self._contact_topics
        }
        self._last_zero_contact_event_sec = {
            name: None for name in self._contact_topics
        }
        self._warned_missing_publishers = False

        self._contact_event_pub = self.create_publisher(String, "/contact_event", 100)
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
            "Contact metrics node publishing /contact_event and /insertion_metrics."
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
            "aborted",
            "canceled",
            "cancelled",
            "timeout",
            "timed_out",
        }:
            self._explicit_failure = True

    def _on_contacts(self, source: str, message: Any) -> None:
        contacts = self._extract_contacts(message)
        contact_count = len(contacts)
        self._contact_messages_observed = True
        self._contact_topic_seen[source] = True

        max_force = self._extract_max_force(contacts)
        if max_force is not None:
            self._max_contact_force = (
                max_force
                if self._max_contact_force is None
                else max(self._max_contact_force, max_force)
            )

        if contact_count > 0:
            self._contact_samples_count += 1
            self._physical_contact_observed = True
            if not self._previous_in_contact.get(source, False):
                self._previous_in_contact[source] = True
                if self._should_publish_transition_event(source):
                    self._contact_events_count += 1
                    self._publish_contact_event(
                        self._contact_event_payload(
                            event_type="contact_started",
                            source=source,
                            contact_count=contact_count,
                            max_force=max_force,
                            message=self._contact_note(contact_count, max_force),
                        )
                    )
            else:
                self._previous_in_contact[source] = True
            return

        if self._previous_in_contact.get(source, False):
            if self._should_publish_transition_event(source):
                self._previous_in_contact[source] = False
                self._publish_contact_event(
                    self._contact_event_payload(
                        event_type="contact_ended",
                        source=source,
                        contact_count=contact_count,
                        max_force=max_force,
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
                    message="Contact topic message received; no contacts detected.",
                )
            )

    def _contact_note(self, contact_count: int, max_force: float | None) -> str:
        if contact_count == 0:
            return "Contact topic message received; no contacts detected."
        if not self._max_contact_force_available:
            note = "Contact observed; force extraction disabled/unvalidated."
        elif max_force is None:
            note = "Contact observed; force value unavailable in parsed message."
        else:
            note = (
                "Contact observed; max_contact_force parsed from message wrenches. "
                "Force extraction is preliminary/unvalidated."
            )
        return note

    def _contact_event_payload(
        self,
        *,
        event_type: str,
        source: str,
        contact_count: int,
        max_force: float | None,
        message: str,
    ) -> dict[str, Any]:
        return {
            "timestamp_ros_sec": self._now_sec(),
            "event_type": event_type,
            "phase": self._current_phase,
            "source": source,
            "contact_count": contact_count,
            "max_contact_force": max_force,
            "message": message,
        }

    def _extract_contacts(self, message: Any) -> list[Any]:
        contacts = getattr(message, "contacts", None)
        if contacts is None:
            contacts = getattr(message, "contact", None)
        if contacts is None:
            return []
        try:
            return list(contacts)
        except TypeError:
            return []

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

    def _extract_max_force(self, contacts: list[Any]) -> float | None:
        if not self._max_contact_force_available:
            return None

        max_force: float | None = None
        for contact in contacts:
            for wrench in self._iter_wrenches(contact):
                for vector in self._iter_force_vectors(wrench):
                    magnitude = self._vector_magnitude(vector)
                    if magnitude is not None:
                        max_force = (
                            magnitude if max_force is None else max(max_force, magnitude)
                        )
        return max_force

    def _iter_wrenches(self, contact: Any) -> list[Any]:
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

    def _iter_force_vectors(self, wrench: Any) -> list[Any]:
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

    @staticmethod
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

    def _publish_contact_event(self, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._contact_event_pub.publish(message)

    def _publish_metrics(self) -> None:
        notes = []
        contact_topics_connected = self._contact_topics_connected()
        contact_metrics_available = bool(contact_topics_connected)
        contact_topics_seen = [
            source for source, seen in self._contact_topic_seen.items() if seen
        ]
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
        if not self._max_contact_force_available:
            notes.append(
                "max_contact_force is null because force extraction is "
                "disabled/unvalidated."
            )
        notes.append(
            "insertion_success remains null until a validated success rule is "
            "implemented."
        )
        notes.append(
            "insertion_success_estimate is a heuristic based on insertion_hold, "
            "completed trial status, and absence of explicit failure."
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
            "contact_topics_seen": contact_topics_seen,
            "contact_events_count": self._contact_events_count,
            "contact_samples_count": self._contact_samples_count,
            "max_contact_force": self._max_contact_force,
            "insertion_success": None,
            "insertion_success_estimate": self._insertion_success_estimate(),
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

    def _insertion_success_estimate(self) -> bool | None:
        if self._explicit_failure:
            return False
        if self._insertion_hold_reached and self._trial_status == "completed":
            return True
        return None

    @staticmethod
    def _source_from_topic(topic: str, index: int) -> str:
        for source in ("peg", "hole", "target", "validation"):
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
