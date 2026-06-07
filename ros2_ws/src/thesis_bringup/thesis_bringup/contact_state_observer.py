#!/usr/bin/env python3
"""Passive contact/state observer for research baseline diagnostics."""

from __future__ import annotations

import csv
import importlib
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def _extract_contacts(message: Any) -> list[Any]:
    contacts = getattr(message, "contacts", None)
    if contacts is None:
        contacts = getattr(message, "contact", None)
    if contacts is None:
        return []
    try:
        return list(contacts)
    except TypeError:
        return []


def _iter_wrenches(contact: Any) -> list[Any]:
    result = []
    for field_name in (
        "wrenches",
        "wrench",
        "body_1_wrench",
        "body_2_wrench",
        "body1_wrench",
        "body2_wrench",
    ):
        value = getattr(contact, field_name, None)
        if value is None:
            continue
        try:
            result.extend(list(value))
        except TypeError:
            result.append(value)
    return result


def _force_vectors(wrench: Any) -> list[Any]:
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
        nested = getattr(wrench, field_name, None)
        vector = getattr(nested, "force", None)
        if vector is not None:
            vectors.append(vector)
    return vectors


def _vector_magnitude(vector: Any) -> float | None:
    try:
        x = float(getattr(vector, "x"))
        y = float(getattr(vector, "y"))
        z = float(getattr(vector, "z"))
    except (AttributeError, TypeError, ValueError):
        return None
    return math.sqrt(x * x + y * y + z * z)


def _max_contact_force(message: Any) -> float:
    max_force = 0.0
    for contact in _extract_contacts(message):
        for wrench in _iter_wrenches(contact):
            for vector in _force_vectors(wrench):
                magnitude = _vector_magnitude(vector)
                if magnitude is not None:
                    max_force = max(max_force, magnitude)
    return max_force


def _entity_name(entity: Any) -> str:
    name = getattr(entity, "name", "")
    return str(name) if name else "unknown"


def _collision_pairs(message: Any, limit: int = 8) -> list[str]:
    pairs: list[str] = []
    for contact in _extract_contacts(message):
        name_1 = _entity_name(getattr(contact, "collision1", None))
        name_2 = _entity_name(getattr(contact, "collision2", None))
        pair = " <-> ".join(sorted((name_1, name_2)))
        if pair not in pairs:
            pairs.append(pair)
        if len(pairs) >= limit:
            break
    return pairs


class ContactStateObserver(Node):
    """Write contact counts and force estimates grouped by insertion state."""

    def __init__(self) -> None:
        super().__init__("contact_state_observer")
        self.declare_parameter("state_topic", "/insertion_state")
        self.declare_parameter(
            "contact_topics",
            [
                "peg:/gazebo/contacts/peg",
                "hole:/gazebo/contacts/hole",
                "target:/gazebo/contacts/target",
            ],
        )
        self.declare_parameter("output_dir", "/tmp/thesis_tracking_logs")
        self.declare_parameter("summary_period_s", 5.0)

        self._state_topic = str(self.get_parameter("state_topic").value)
        self._contact_topics = self._parse_contact_topics(
            list(self.get_parameter("contact_topics").value)
        )
        self._output_dir = Path(str(self.get_parameter("output_dir").value))
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._output_dir / "contact_state_samples.csv"
        self._summary_path = self._output_dir / "contact_state_summary.md"
        self._csv_file = self._csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(
            [
                "stamp_s",
                "state",
                "source",
                "contact_count",
                "max_contact_force_n",
                "collision_pairs",
            ]
        )

        self._state = "UNKNOWN"
        self._sample_count = 0
        self._positive_count = 0
        self._max_force = 0.0
        self._by_state_source: dict[tuple[str, str], list[tuple[int, float]]] = defaultdict(list)
        self._by_state_source_pair: dict[tuple[str, str, str], list[tuple[int, float]]] = (
            defaultdict(list)
        )
        self._first_stamp: float | None = None
        self._last_stamp: float | None = None

        self.create_subscription(String, self._state_topic, self._on_state, 20)
        self._subscribe_contacts()
        period = max(1.0, float(self.get_parameter("summary_period_s").value))
        self.create_timer(period, self._write_summary)
        self.get_logger().info(
            f"ContactStateObserver writing {len(self._contact_topics)} contact topics "
            f"by {self._state_topic} to {self._output_dir}"
        )

    def _parse_contact_topics(self, values: list[str]) -> dict[str, str]:
        topics: dict[str, str] = {}
        for value in values:
            if ":" not in value:
                continue
            name, topic = value.split(":", 1)
            name = name.strip()
            topic = topic.strip()
            if name and topic:
                topics[name] = topic
        return topics

    def _subscribe_contacts(self) -> None:
        try:
            contacts_msg = importlib.import_module("ros_gz_interfaces.msg").Contacts
        except (AttributeError, ImportError) as exc:
            self.get_logger().warning(
                "ros_gz_interfaces/msg/Contacts unavailable; contact observer "
                f"will write zero samples. Detail: {exc}"
            )
            return
        for source, topic in self._contact_topics.items():
            self.create_subscription(
                contacts_msg,
                topic,
                lambda message, source=source: self._on_contacts(source, message),
                50,
            )

    def _on_state(self, msg: String) -> None:
        self._state = msg.data or "UNKNOWN"

    def _on_contacts(self, source: str, msg: Any) -> None:
        stamp_s = self.get_clock().now().nanoseconds * 1.0e-9
        contact_count = len(_extract_contacts(msg))
        max_force = _max_contact_force(msg)
        pairs = _collision_pairs(msg)
        pair_text = "; ".join(pairs)
        state = self._state
        self._writer.writerow(
            [
                f"{stamp_s:.9f}",
                state,
                source,
                contact_count,
                f"{max_force:.9f}",
                pair_text,
            ]
        )
        self._sample_count += 1
        if contact_count > 0:
            self._positive_count += 1
        self._max_force = max(self._max_force, max_force)
        self._by_state_source[(state, source)].append((contact_count, max_force))
        if pairs:
            for pair in pairs:
                self._by_state_source_pair[(state, source, pair)].append(
                    (contact_count, max_force)
                )
        else:
            self._by_state_source_pair[(state, source, "none")].append(
                (contact_count, max_force)
            )
        if self._first_stamp is None:
            self._first_stamp = stamp_s
        self._last_stamp = stamp_s

    def _write_summary(self) -> None:
        if self._csv_file.closed:
            return
        self._csv_file.flush()
        duration = (
            (self._last_stamp - self._first_stamp)
            if self._first_stamp is not None and self._last_stamp is not None
            else 0.0
        )
        lines = [
            "# Contact State Summary",
            "",
            f"- state_topic: `{self._state_topic}`",
            f"- contact_topics: `{', '.join(f'{k}:{v}' for k, v in self._contact_topics.items())}`",
            f"- samples: `{self._sample_count}`",
            f"- positive_contact_samples: `{self._positive_count}`",
            f"- duration_s: `{duration:.3f}`",
            f"- max_contact_force_n: `{self._max_force:.6f}`",
            "",
            "| State | Source | Samples | Positive Samples | Max Contacts | Mean Contacts | Max Force N |",
            "|---|---|---:|---:|---:|---:|---:|",
        ]
        for (state, source), rows in sorted(self._by_state_source.items()):
            counts = [count for count, _force in rows]
            forces = [force for _count, force in rows]
            lines.append(
                "| "
                f"{state} | {source} | {len(rows)} | "
                f"{sum(1 for count in counts if count > 0)} | "
                f"{max(counts, default=0)} | "
                f"{(mean(counts) if counts else 0.0):.6f} | "
                f"{max(forces, default=0.0):.6f} |"
            )
        lines.extend(
            [
                "",
                "## Collision Pairs",
                "",
                "| State | Source | Collision Pair | Samples | Max Force N |",
                "|---|---|---|---:|---:|",
            ]
        )
        for (state, source, pair), rows in sorted(self._by_state_source_pair.items()):
            forces = [force for _count, force in rows]
            lines.append(
                "| "
                f"{state} | {source} | `{pair}` | {len(rows)} | "
                f"{max(forces, default=0.0):.6f} |"
            )
        lines.extend(
            [
                "",
                "This observer is passive. It does not publish commands or alter controller behavior.",
            ]
        )
        self._summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def destroy_node(self) -> bool:
        self._write_summary()
        if not self._csv_file.closed:
            self._csv_file.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ContactStateObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
