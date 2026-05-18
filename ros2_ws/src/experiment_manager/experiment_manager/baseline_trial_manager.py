"""Structured trial logger for Research Baseline v0.7."""

from __future__ import annotations

import csv
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, TextIO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


FORCE_EXTRACTION_METHOD = "ros_gz_interfaces Contacts.wrenches force magnitude"


class BaselineTrialManager(Node):
    """Record reproducible metadata, task events, safety events, and summaries."""

    TERMINAL_STATUSES = {
        "completed",
        "failed",
        "failed_pre_contact",
        "guarded_stop",
        "guarded_contact_stop",
    }

    JOINT_NAMES = (
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    )

    def __init__(self) -> None:
        super().__init__("baseline_trial_manager")
        default_results_root = Path.cwd() / "results" / "baseline_trials"
        self.declare_parameter("results_root", str(default_results_root))
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("task_phase_topic", "/task_phase")
        self.declare_parameter("task_event_topic", "/task_event")
        self.declare_parameter("trial_status_topic", "/trial_status")
        self.declare_parameter("safety_status_topic", "/safety_status")
        self.declare_parameter("contact_event_topic", "/contact_event")
        self.declare_parameter("insertion_metrics_topic", "/insertion_metrics")
        self.declare_parameter("force_guard_status_topic", "/force_guard_status")
        self.declare_parameter("trial_mode", "baseline_task")

        results_root_parameter = (
            self.get_parameter("results_root").get_parameter_value().string_value
        )
        self._results_root = Path(results_root_parameter).expanduser()
        self._joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self._task_phase_topic = (
            self.get_parameter("task_phase_topic").get_parameter_value().string_value
        )
        self._task_event_topic = (
            self.get_parameter("task_event_topic").get_parameter_value().string_value
        )
        self._trial_status_topic = (
            self.get_parameter("trial_status_topic").get_parameter_value().string_value
        )
        self._safety_status_topic = (
            self.get_parameter("safety_status_topic").get_parameter_value().string_value
        )
        self._contact_event_topic = (
            self.get_parameter("contact_event_topic").get_parameter_value().string_value
        )
        self._insertion_metrics_topic = (
            self.get_parameter("insertion_metrics_topic").get_parameter_value().string_value
        )
        self._force_guard_status_topic = (
            self.get_parameter("force_guard_status_topic")
            .get_parameter_value()
            .string_value
        )
        self._trial_mode = self._normalize_trial_mode(
            self.get_parameter("trial_mode").get_parameter_value().string_value
        )

        self._start_time = self.get_clock().now()
        self._timestamp, self._trial_id, self._trial_dir = self._create_trial_dir()

        self._joint_state_count = 0
        self._total_task_events = 0
        self._safety_warnings_count = 0
        self._safety_violations_count = 0
        self._safety_status_observed = False
        self._completed_phases_count = 0
        self._task_started = False
        self._task_completed = False
        self._trial_failed = False
        self._final_trial_status = "idle"
        self._final_task_phase = "uninitialized"
        self._contact_events_count = 0
        self._contact_episode_count = 0
        self._contact_samples_count = 0
        self._max_contact_force: float | None = None
        self._force_threshold_warning = False
        self._force_threshold_violation = False
        self._force_guard_triggered = False
        self._force_guard_trigger_force: float | None = None
        self._force_guard_threshold: float | None = None
        self._early_contact_guard_triggered = False
        self._early_contact_guard_trigger_force: float | None = None
        self._early_contact_guard_source: str | None = None
        self._guarded_contact_stop = False
        self._segment_count_executed = 0
        self._force_violation_threshold_n: float | None = 100.0
        self._force_extraction_available = False
        self._force_extraction_method = FORCE_EXTRACTION_METHOD
        self._insertion_attempted = False
        self._insertion_hold_reached = False
        self._insertion_success: bool | None = None
        self._insertion_success_estimate: bool | None = None
        self._peg_contact_observed = False
        self._hole_contact_observed = False
        self._peg_table_contact_observed = False
        self._peg_table_contact_count = 0
        self._peg_hole_contact_observed = False
        self._peg_hole_contact_count = 0
        self._first_peg_hole_contact_phase: str | None = None
        self._first_peg_table_contact_phase: str | None = None
        self._peg_hole_collision_pairs: list[str] = []
        self._peg_hole_collision_pair_set: set[str] = set()
        self._non_insertion_contact_pairs: list[str] = []
        self._non_insertion_contact_pair_set: set[str] = set()
        self._initial_contact_detected = False
        self._initial_contact_pairs: list[str] = []
        self._initial_contact_pair_set: set[str] = set()
        self._uninitialized_contact_count = 0
        self._clean_initial_state = True
        self._max_peg_contact_force: float | None = None
        self._max_hole_contact_force: float | None = None
        self._insertion_depth_available = False
        self._insertion_depth_estimate: float | None = None
        self._insertion_metrics_received = False
        self._contact_metrics_available = False
        self._contact_topics_configured: dict[str, str] = {}
        self._contact_topics_connected: set[str] = set()
        self._contact_messages_observed = False
        self._physical_contact_observed = False
        self._collision_pairs: list[str] = []
        self._collision_pair_set: set[str] = set()
        self._first_contact_collision1: str | None = None
        self._first_contact_collision2: str | None = None
        self._first_contact_phase: str | None = None
        self._pre_approach_contact_detected = False
        self._contact_topics_seen: set[str] = set()
        self._positive_contact_counts: dict[str, int] = {}
        self._contact_notes = "No contact metrics have been received yet."
        self._closed = False
        self._terminal_status_observed = False

        self._metadata_path = self._trial_dir / "trial_metadata.json"
        self._summary_path = self._trial_dir / "trial_summary.json"
        self._write_json(self._metadata_path, self._build_metadata())

        self._joint_states_file = self._open_csv("joint_states.csv")
        self._task_events_file = self._open_csv("task_events.csv")
        self._safety_events_file = self._open_csv("safety_events.csv")
        self._contact_events_file = self._open_csv("contact_events.csv")
        self._joint_states_writer = csv.writer(self._joint_states_file)
        self._task_events_writer = csv.writer(self._task_events_file)
        self._safety_events_writer = csv.writer(self._safety_events_file)
        self._contact_events_writer = csv.writer(self._contact_events_file)
        self._write_headers()
        self._write_summary()

        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_state,
            100,
        )
        self.create_subscription(
            String,
            self._task_phase_topic,
            self._on_task_phase,
            100,
        )
        self.create_subscription(
            String,
            self._task_event_topic,
            self._on_task_event,
            100,
        )
        self.create_subscription(
            String,
            self._trial_status_topic,
            self._on_trial_status,
            100,
        )
        self.create_subscription(
            String,
            self._safety_status_topic,
            self._on_safety_status,
            100,
        )
        self.create_subscription(
            String,
            self._contact_event_topic,
            self._on_contact_event,
            100,
        )
        self.create_subscription(
            String,
            self._insertion_metrics_topic,
            self._on_insertion_metrics,
            100,
        )
        self.create_subscription(
            String,
            self._force_guard_status_topic,
            self._on_force_guard_status,
            100,
        )
        self.create_timer(2.0, self._flush_logs)

        self.get_logger().info(f"Started baseline trial logging: {self._trial_dir}")
        self.get_logger().info(
            "Recording "
            f"{self._joint_states_topic}, {self._task_phase_topic}, "
            f"{self._task_event_topic}, {self._trial_status_topic}, and "
            f"{self._safety_status_topic}; contact metrics from "
            f"{self._contact_event_topic}, {self._insertion_metrics_topic}, and "
            f"{self._force_guard_status_topic}"
        )

    def _on_joint_state(self, message: JointState) -> None:
        positions = {name: value for name, value in zip(message.name, message.position)}
        self._joint_states_writer.writerow(
            [
                f"{self._message_time_sec(message):.9f}",
                *[
                    self._format_optional_float(positions.get(joint_name))
                    for joint_name in self.JOINT_NAMES
                ],
            ]
        )
        self._joint_state_count += 1
        if self._joint_state_count % 10 == 0:
            self._joint_states_file.flush()

    def _on_task_phase(self, message: String) -> None:
        self._final_task_phase = message.data.strip() or "empty_phase"

    def _on_task_event(self, message: String) -> None:
        event = self._parse_json_message(message.data, "task_event")
        ros_time_sec = self._event_time_sec(event)
        event_type = str(event.get("event_type", "unknown"))
        phase = str(event.get("phase", "unknown"))

        self._final_task_phase = phase
        self._total_task_events += 1
        if event_type in {
            "sequence_started",
            "phase_started",
            "segmented_sequence_started",
            "segment_started",
        }:
            self._task_started = True
        segment_count = self._coerce_int(
            event.get("segment_count_executed"), default=self._segment_count_executed
        )
        self._segment_count_executed = max(
            self._segment_count_executed, segment_count
        )
        event_max_force = self._coerce_optional_float(event.get("max_contact_force"))
        if event_max_force is not None:
            self._max_contact_force = (
                event_max_force
                if self._max_contact_force is None
                else max(self._max_contact_force, event_max_force)
            )
        self._physical_contact_observed = self._physical_contact_observed or bool(
            event.get("physical_contact_observed", False)
        )
        if (
            event.get("physical_contact_observed", False)
            or event_type == "unexpected_pre_approach_contact"
        ):
            self._record_contact_diagnostics(
                self._coerce_collision_pairs(event.get("collision_pairs")),
                self._coerce_optional_string(event.get("first_collision1")),
                self._coerce_optional_string(event.get("first_collision2")),
                phase,
            )
        self._force_threshold_violation = self._force_threshold_violation or bool(
            event.get("force_threshold_violation", False)
        )
        if event_type in {"phase_succeeded", "segment_succeeded"}:
            self._completed_phases_count += 1
        elif event_type in {"sequence_completed", "segmented_sequence_completed"}:
            self._task_started = True
            self._task_completed = True
        elif event_type in {"sequence_failed", "segmented_sequence_failed"}:
            self._trial_failed = True
        elif event_type == "unexpected_pre_approach_contact":
            self._pre_approach_contact_detected = True
            self._trial_failed = True
            self._guarded_contact_stop = False
        elif event_type == "force_guard_triggered":
            self._force_guard_triggered = True
            self._trial_failed = True
            trigger_force = self._coerce_optional_float(
                event.get("force_guard_trigger_force")
            )
            threshold = self._coerce_optional_float(event.get("force_guard_threshold"))
            if trigger_force is not None:
                self._force_guard_trigger_force = trigger_force
            if threshold is not None:
                self._force_guard_threshold = threshold
        elif event_type in {"early_contact_guard_triggered", "early_contact_detected"}:
            self._early_contact_guard_triggered = True
            self._guarded_contact_stop = True
            trigger_force = self._coerce_optional_float(
                event.get("early_contact_guard_trigger_force")
            )
            source = str(event.get("early_contact_guard_source", "")).strip()
            if trigger_force is not None:
                self._early_contact_guard_trigger_force = trigger_force
            if source:
                self._early_contact_guard_source = source
        elif event_type == "guarded_contact_stop":
            self._guarded_contact_stop = True

        self._task_events_writer.writerow(
            [
                f"{ros_time_sec:.9f}",
                event_type,
                phase,
                event.get("pose_index", ""),
                event.get("total_poses", ""),
                event.get("safety_tag", ""),
                event.get("message", ""),
                event.get("segment_name", ""),
                self._json_csv_value(event.get("target_joint_positions")),
                self._json_csv_value(event.get("reached_joint_positions")),
                self._json_csv_value(event.get("joint_position_error")),
                event.get("end_effector_base_frame", ""),
                event.get("end_effector_tool_frame", ""),
                self._json_csv_value(event.get("end_effector_position_xyz")),
                self._json_csv_value(event.get("end_effector_orientation_xyzw")),
            ]
        )
        self._task_events_file.flush()
        self._write_summary()

    def _on_trial_status(self, message: String) -> None:
        status = message.data.strip() or "empty_status"
        self._final_trial_status = status
        if status == "completed":
            self._task_completed = True
        elif status == "guarded_contact_stop":
            self._guarded_contact_stop = True
            if self._trial_mode == "segmented_robot_contact_validation":
                self._task_completed = True
            elif self._trial_mode == "peg_hole_insertion_validation":
                self._task_completed = True
        elif status == "failed_pre_contact":
            self._trial_failed = True
            self._pre_approach_contact_detected = True
            self._task_completed = False
        elif status in {"failed", "guarded_stop"}:
            self._trial_failed = True
        self._write_summary()
        if status in self.TERMINAL_STATUSES:
            self._terminal_status_observed = True
            self._flush_logs()
            self.get_logger().info(
                f"Observed terminal trial status '{status}'; final summary flushed."
            )

    def _on_safety_status(self, message: String) -> None:
        event = self._parse_json_message(message.data, "safety_status")
        ros_time_sec = self._event_time_sec(event)
        level = str(event.get("level", "UNKNOWN"))
        code = str(event.get("code", "unknown"))
        phase = str(event.get("phase", self._final_task_phase))
        detail = str(event.get("message", message.data))

        self._safety_status_observed = True
        if level == "WARNING":
            self._safety_warnings_count += 1
        elif level == "VIOLATION":
            self._safety_violations_count += 1

        self._safety_events_writer.writerow(
            [f"{ros_time_sec:.9f}", level, code, phase, detail]
        )
        self._safety_events_file.flush()
        self._write_summary()

    def _on_contact_event(self, message: String) -> None:
        event = self._parse_json_message(message.data, "contact_event")
        ros_time_sec = self._event_time_sec(event)
        event_type = str(event.get("event_type", "unknown"))
        phase = str(event.get("phase", self._final_task_phase))
        source = str(event.get("source", "unknown"))
        contact_count = self._coerce_int(event.get("contact_count"), default=0)
        max_contact_force = self._coerce_optional_float(event.get("max_contact_force"))
        collision_pairs = self._coerce_collision_pairs(event.get("collision_pairs"))
        first_collision1 = self._coerce_optional_string(event.get("first_collision1"))
        first_collision2 = self._coerce_optional_string(event.get("first_collision2"))
        detail = str(event.get("message", message.data))

        if source:
            self._contact_topics_seen.add(source)
            self._contact_topics_connected.add(source)
        self._contact_messages_observed = True
        if phase == "uninitialized" and contact_count > 0 and not collision_pairs:
            self._initial_contact_detected = True
            self._clean_initial_state = False
            self._uninitialized_contact_count += contact_count
        positive_physical_contact = (
            contact_count > 0 and self._counts_as_physical_contact(source)
        )
        if positive_physical_contact and event_type in {"contact_started", "unknown"}:
            self._contact_episode_count += 1
            self._contact_events_count = self._contact_episode_count
            self._physical_contact_observed = True
            self._record_contact_diagnostics(
                collision_pairs,
                first_collision1,
                first_collision2,
                phase,
            )
        elif positive_physical_contact:
            self._physical_contact_observed = True
            self._record_contact_diagnostics(
                collision_pairs,
                first_collision1,
                first_collision2,
                phase,
            )
        self._contact_metrics_available = True
        if max_contact_force is not None:
            self._force_extraction_available = True
            self._max_contact_force = (
                max_contact_force
                if self._max_contact_force is None
                else max(self._max_contact_force, max_contact_force)
            )

        self._contact_events_writer.writerow(
            [
                f"{ros_time_sec:.9f}",
                event_type,
                phase,
                source,
                contact_count,
                self._format_optional_float(max_contact_force),
                self._json_csv_value(collision_pairs),
                first_collision1 or "",
                first_collision2 or "",
                detail,
            ]
        )
        self._contact_events_file.flush()
        self._write_summary()

    def _on_insertion_metrics(self, message: String) -> None:
        metrics = self._parse_json_message(message.data, "insertion_metrics")
        self._update_contact_topics_configured(metrics.get("contact_topics_configured"))
        self._contact_topics_connected = self._coerce_string_set(
            metrics.get("contact_topics_connected"),
            default=self._contact_topics_connected,
        )
        self._contact_metrics_available = bool(
            metrics.get("contact_metrics_available", self._contact_metrics_available)
        )
        self._contact_messages_observed = self._contact_messages_observed or bool(
            metrics.get("contact_messages_observed", False)
        )
        self._physical_contact_observed = self._physical_contact_observed or bool(
            metrics.get("physical_contact_observed", False)
        )
        self._update_contact_topics_seen(metrics.get("contact_topics_seen"))
        self._update_positive_contact_counts(metrics.get("positive_contact_counts"))
        self._force_extraction_available = self._force_extraction_available or bool(
            metrics.get("force_extraction_available", False)
        )
        force_method = str(
            metrics.get("force_extraction_method", self._force_extraction_method)
        ).strip()
        if force_method:
            self._force_extraction_method = force_method
        self._insertion_attempted = bool(
            metrics.get("insertion_attempted", self._insertion_attempted)
        )
        self._insertion_hold_reached = bool(
            metrics.get("insertion_hold_reached", self._insertion_hold_reached)
        )
        self._insertion_success = self._coerce_optional_bool(
            metrics.get("insertion_success")
        )
        self._insertion_success_estimate = self._coerce_optional_bool(
            metrics.get("insertion_success_estimate")
        )
        self._insertion_metrics_received = True
        self._peg_contact_observed = self._peg_contact_observed or bool(
            metrics.get("peg_contact_observed", False)
        )
        self._hole_contact_observed = self._hole_contact_observed or bool(
            metrics.get("hole_contact_observed", False)
        )
        self._peg_table_contact_observed = self._peg_table_contact_observed or bool(
            metrics.get("peg_table_contact_observed", False)
        )
        self._peg_table_contact_count = max(
            self._peg_table_contact_count,
            self._coerce_int(
                metrics.get("peg_table_contact_count"),
                default=self._peg_table_contact_count,
            ),
        )
        self._peg_hole_contact_observed = self._peg_hole_contact_observed or bool(
            metrics.get("peg_hole_contact_observed", False)
        )
        self._peg_hole_contact_count = max(
            self._peg_hole_contact_count,
            self._coerce_int(
                metrics.get("peg_hole_contact_count"),
                default=self._peg_hole_contact_count,
            ),
        )
        self._merge_unique_pairs(
            self._coerce_collision_pairs(metrics.get("peg_hole_collision_pairs")),
            self._peg_hole_collision_pair_set,
            self._peg_hole_collision_pairs,
        )
        self._merge_unique_pairs(
            self._coerce_collision_pairs(metrics.get("non_insertion_contact_pairs")),
            self._non_insertion_contact_pair_set,
            self._non_insertion_contact_pairs,
        )
        metrics_initial_contact_detected = bool(
            metrics.get("initial_contact_detected", False)
        )
        self._initial_contact_detected = (
            self._initial_contact_detected or metrics_initial_contact_detected
        )
        if metrics_initial_contact_detected:
            self._clean_initial_state = False
        self._merge_unique_pairs(
            self._coerce_collision_pairs(metrics.get("initial_contact_pairs")),
            self._initial_contact_pair_set,
            self._initial_contact_pairs,
        )
        self._uninitialized_contact_count = max(
            self._uninitialized_contact_count,
            self._coerce_int(
                metrics.get("uninitialized_contact_count"),
                default=self._uninitialized_contact_count,
            ),
        )
        metrics_clean_initial_state = self._coerce_optional_bool(
            metrics.get("clean_initial_state")
        )
        if metrics_clean_initial_state is not None:
            self._clean_initial_state = (
                self._clean_initial_state and metrics_clean_initial_state
            )
        if self._first_peg_hole_contact_phase is None:
            self._first_peg_hole_contact_phase = self._coerce_optional_string(
                metrics.get("first_peg_hole_contact_phase")
            )
        if self._first_peg_table_contact_phase is None:
            self._first_peg_table_contact_phase = self._coerce_optional_string(
                metrics.get("first_peg_table_contact_phase")
            )
        metrics_peg_force = self._coerce_optional_float(
            metrics.get("max_peg_contact_force")
        )
        if metrics_peg_force is not None:
            self._max_peg_contact_force = (
                metrics_peg_force
                if self._max_peg_contact_force is None
                else max(self._max_peg_contact_force, metrics_peg_force)
            )
        metrics_hole_force = self._coerce_optional_float(
            metrics.get("max_hole_contact_force")
        )
        if metrics_hole_force is not None:
            self._max_hole_contact_force = (
                metrics_hole_force
                if self._max_hole_contact_force is None
                else max(self._max_hole_contact_force, metrics_hole_force)
            )
        self._insertion_depth_available = self._insertion_depth_available or bool(
            metrics.get("insertion_depth_available", False)
        )
        metrics_depth = self._coerce_optional_float(
            metrics.get("insertion_depth_estimate")
        )
        if metrics_depth is not None:
            self._insertion_depth_estimate = metrics_depth

        metrics_contact_count = self._coerce_int(
            metrics.get("contact_events_count"), default=self._contact_events_count
        )
        self._contact_events_count = max(
            self._contact_events_count, metrics_contact_count
        )
        metrics_episode_count = self._coerce_int(
            metrics.get("contact_episode_count"), default=self._contact_episode_count
        )
        self._contact_episode_count = max(
            self._contact_episode_count, metrics_episode_count
        )
        metrics_sample_count = self._coerce_int(
            metrics.get("contact_samples_count"), default=self._contact_samples_count
        )
        self._contact_samples_count = max(
            self._contact_samples_count, metrics_sample_count
        )
        if metrics_contact_count > 0:
            self._contact_messages_observed = True
            self._physical_contact_observed = self._physical_contact_observed or bool(
                metrics.get("physical_contact_observed", False)
            )
        if metrics_sample_count > 0:
            self._contact_messages_observed = True
            self._physical_contact_observed = self._physical_contact_observed or bool(
                metrics.get("physical_contact_observed", False)
            )
        if metrics_episode_count > 0:
            self._contact_messages_observed = True
            self._physical_contact_observed = self._physical_contact_observed or bool(
                metrics.get("physical_contact_observed", False)
            )
        self._record_contact_diagnostics(
            self._coerce_collision_pairs(metrics.get("collision_pairs")),
            self._coerce_optional_string(metrics.get("first_collision1")),
            self._coerce_optional_string(metrics.get("first_collision2")),
            self._final_task_phase,
        )
        metrics_max_force = self._coerce_optional_float(metrics.get("max_contact_force"))
        if metrics_max_force is not None:
            self._force_extraction_available = True
            self._max_contact_force = (
                metrics_max_force
                if self._max_contact_force is None
                else max(self._max_contact_force, metrics_max_force)
            )
        self._force_threshold_warning = self._force_threshold_warning or bool(
            metrics.get("force_threshold_warning", False)
        )
        self._force_threshold_violation = self._force_threshold_violation or bool(
            metrics.get("force_threshold_violation", False)
        )
        violation_threshold = self._coerce_optional_float(
            metrics.get("robot_validation_violation_force_n")
        )
        if violation_threshold is not None:
            self._force_violation_threshold_n = violation_threshold
        self._contact_notes = str(metrics.get("notes", self._contact_notes))
        self._write_summary()

    def _on_force_guard_status(self, message: String) -> None:
        status = self._parse_json_message(message.data, "force_guard_status")
        source = str(status.get("source", "")).strip()
        contact_count = self._coerce_int(status.get("contact_count"), default=0)
        max_contact_force = self._coerce_optional_float(
            status.get("max_contact_force")
        )

        if source:
            self._contact_topics_seen.add(source)
            self._contact_topics_connected.add(source)
        self._contact_messages_observed = True
        if contact_count > 0 and self._counts_as_physical_contact(source):
            self._physical_contact_observed = True
            self._record_contact_diagnostics(
                self._coerce_collision_pairs(status.get("collision_pairs")),
                self._coerce_optional_string(status.get("first_collision1")),
                self._coerce_optional_string(status.get("first_collision2")),
                self._final_task_phase,
            )
        if max_contact_force is not None:
            self._force_extraction_available = True
            self._max_contact_force = (
                max_contact_force
                if self._max_contact_force is None
                else max(self._max_contact_force, max_contact_force)
            )
        self._force_extraction_available = self._force_extraction_available or bool(
            status.get("force_extraction_available", False)
        )
        self._force_threshold_warning = self._force_threshold_warning or bool(
            status.get("force_threshold_warning", False)
        )
        self._force_threshold_violation = self._force_threshold_violation or bool(
            status.get("force_threshold_violation", False)
        )
        self._contact_metrics_available = True
        self._write_summary()

    def close(self) -> None:
        if self._closed:
            return

        self._flush_logs()
        self._write_summary()

        for file_handle in (
            self._joint_states_file,
            self._task_events_file,
            self._safety_events_file,
            self._contact_events_file,
        ):
            file_handle.close()

        self._closed = True
        self.get_logger().info(f"Wrote baseline trial summary: {self._summary_path}")

    def _build_metadata(self) -> dict[str, object]:
        is_contact_probe_validation = self._trial_mode == "contact_probe_validation"
        is_peg_hole_insertion_validation = (
            self._trial_mode == "peg_hole_insertion_validation"
        )
        is_contact_validation = self._trial_mode in {
            "robot_contact_validation",
            "segmented_guarded_contact",
            "segmented_robot_contact_validation",
            "peg_hole_insertion_validation",
        }
        return {
            "trial_id": self._trial_id,
            "timestamp": self._timestamp,
            "simulator": "Gazebo",
            "robot": (
                "none" if is_contact_probe_validation else "KUKA LBR iisy 3 R760"
            ),
            "end_effector": (
                "none" if is_contact_probe_validation else "simplified research gripper"
            ),
            "task": (
                "passive contact probe validation"
                if is_contact_probe_validation
                else "peg/hole insertion instrumentation validation"
                if is_peg_hole_insertion_validation
                else "segmented guarded robot-to-object contact validation"
                if self._is_segmented_contact_mode()
                else "robot-to-object contact validation"
                if self._trial_mode == "robot_contact_validation"
                else "peg-in-hole baseline"
            ),
            "controller": (
                "none" if is_contact_probe_validation else "joint_trajectory_controller"
            ),
            "framework_version": (
                "v2.0"
                if is_peg_hole_insertion_validation
                else "v1.0"
                if self._is_segmented_contact_mode()
                else "v0.9"
                if is_contact_validation
                else "v0.5"
            ),
            "trial_mode": self._trial_mode,
            "notes": (
                "Research Baseline logging records task events, safety events, trial "
                "status, joint states, contact events, and insertion metrics. "
                "Contact-force values are extracted from validated "
                "ros_gz_interfaces Contacts.wrenches force vectors when present. "
                "The contact_probe_validation mode validates instrumentation with a "
                "passive Gazebo probe. The "
                "robot_contact_validation mode runs a separate scripted robot "
                "approach toward a dedicated contact target; contact absence is "
                "reported without failing the trial. The "
                "segmented_robot_contact_validation mode replaces the long contact "
                "approach with short checked joint-space segments and stops before "
                "sending additional approach motion after contact is observed. The "
                "peg_hole_insertion_validation mode starts peg/hole-specific "
                "instrumentation without treating null insertion_success as a trial "
                "failure."
            ),
            "topics": {
                "joint_states": self._joint_states_topic,
                "task_phase": self._task_phase_topic,
                "task_event": self._task_event_topic,
                "trial_status": self._trial_status_topic,
                "safety_status": self._safety_status_topic,
                "contact_event": self._contact_event_topic,
                "insertion_metrics": self._insertion_metrics_topic,
                "force_guard_status": self._force_guard_status_topic,
            },
        }

    def _build_summary(self) -> dict[str, object]:
        execution_time_sec = self._elapsed_sec()
        safe_success = self._safe_success()
        robot_contact_validation_success = self._robot_contact_validation_success()
        segmented_guarded_contact_success = (
            self._segmented_guarded_contact_success()
        )
        segmented_contact_success = self._segmented_contact_success()
        peg_hole_instrumentation_success = (
            self._peg_hole_instrumentation_success()
        )
        clean_scene_success = self._clean_scene_success()
        insertion_success_estimate = self._summary_insertion_success_estimate()
        return {
            "trial_id": self._trial_id,
            "trial_mode": self._trial_mode,
            "task_started": self._effective_task_started(),
            "task_completed": self._task_completed,
            "trial_failed": self._trial_failed,
            "final_trial_status": self._final_trial_status,
            "final_task_phase": self._final_task_phase,
            "completed_phases_count": self._completed_phases_count,
            "total_task_events": self._total_task_events,
            "safety_warnings_count": self._safety_warnings_count,
            "safety_violations_count": self._safety_violations_count,
            "safety_status_observed": self._safety_status_observed,
            "execution_time_sec": execution_time_sec,
            "safe_success": safe_success,
            "robot_contact_validation_success": robot_contact_validation_success,
            "segmented_guarded_contact_success": (
                segmented_guarded_contact_success
            ),
            "segmented_contact_success": segmented_contact_success,
            "segment_count_executed": self._segment_count_executed,
            "contact_events_count": self._contact_events_count,
            "contact_episode_count": self._contact_episode_count,
            "contact_samples_count": self._contact_samples_count,
            "max_contact_force": self._max_contact_force,
            "force_threshold_warning": self._force_threshold_warning,
            "force_threshold_violation": self._force_threshold_violation,
            "force_guard_triggered": self._force_guard_triggered,
            "force_guard_trigger_force": self._force_guard_trigger_force,
            "force_guard_threshold": self._force_guard_threshold,
            "pre_approach_contact_detected": self._pre_approach_contact_detected,
            "first_contact_collision1": self._first_contact_collision1,
            "first_contact_collision2": self._first_contact_collision2,
            "first_contact_phase": self._first_contact_phase,
            "collision_pairs": list(self._collision_pairs),
            "early_contact_guard_triggered": self._early_contact_guard_triggered,
            "early_contact_guard_trigger_force": (
                self._early_contact_guard_trigger_force
            ),
            "early_contact_guard_source": self._early_contact_guard_source,
            "guarded_contact_stop": self._guarded_contact_stop,
            "force_extraction_available": self._force_extraction_available,
            "force_extraction_method": self._force_extraction_method,
            "insertion_attempted": self._insertion_attempted,
            "insertion_hold_reached": self._insertion_hold_reached,
            "insertion_success": self._insertion_success,
            "insertion_success_estimate": insertion_success_estimate,
            "peg_contact_observed": self._peg_contact_observed,
            "hole_contact_observed": self._hole_contact_observed,
            "peg_table_contact_observed": self._peg_table_contact_observed,
            "peg_table_contact_count": self._peg_table_contact_count,
            "peg_hole_contact_observed": self._peg_hole_contact_observed,
            "peg_hole_contact_count": self._peg_hole_contact_count,
            "first_peg_hole_contact_phase": self._first_peg_hole_contact_phase,
            "first_peg_table_contact_phase": self._first_peg_table_contact_phase,
            "peg_hole_collision_pairs": list(self._peg_hole_collision_pairs),
            "non_insertion_contact_pairs": list(self._non_insertion_contact_pairs),
            "initial_contact_detected": self._initial_contact_detected,
            "initial_contact_pairs": list(self._initial_contact_pairs),
            "uninitialized_contact_count": self._uninitialized_contact_count,
            "clean_initial_state": self._clean_initial_state,
            "max_peg_contact_force": self._max_peg_contact_force,
            "max_hole_contact_force": self._max_hole_contact_force,
            "insertion_depth_available": self._insertion_depth_available,
            "insertion_depth_estimate": self._insertion_depth_estimate,
            "peg_hole_instrumentation_success": peg_hole_instrumentation_success,
            "clean_scene_success": clean_scene_success,
            "contact_topics_configured": self._contact_topics_configured,
            "contact_topics_connected": sorted(self._contact_topics_connected),
            "contact_messages_observed": self._contact_messages_observed,
            "physical_contact_observed": self._physical_contact_observed,
            "contact_topics_seen": sorted(self._contact_topics_seen),
            "positive_contact_counts": self._positive_contact_counts,
            "contact_metrics_available": self._contact_metrics_available,
            "notes": self._summary_notes(),
        }

    def _write_headers(self) -> None:
        self._joint_states_writer.writerow(["ros_time_sec", *self.JOINT_NAMES])
        self._task_events_writer.writerow(
            [
                "ros_time_sec",
                "event_type",
                "phase",
                "pose_index",
                "total_poses",
                "safety_tag",
                "message",
                "segment_name",
                "target_joint_positions",
                "reached_joint_positions",
                "joint_position_error",
                "end_effector_base_frame",
                "end_effector_tool_frame",
                "end_effector_position_xyz",
                "end_effector_orientation_xyzw",
            ]
        )
        self._safety_events_writer.writerow(
            ["ros_time_sec", "level", "code", "phase", "message"]
        )
        self._contact_events_writer.writerow(
            [
                "ros_time_sec",
                "event_type",
                "phase",
                "source",
                "contact_count",
                "max_contact_force",
                "collision_pairs",
                "first_collision1",
                "first_collision2",
                "message",
            ]
        )
        self._flush_logs()

    def _open_csv(self, name: str) -> TextIO:
        return (self._trial_dir / name).open("w", encoding="utf-8", newline="")

    def _create_trial_dir(self) -> tuple[str, str, Path]:
        while True:
            timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
            trial_id = f"trial_{timestamp}"
            trial_dir = self._results_root / trial_id
            try:
                trial_dir.mkdir(parents=True, exist_ok=False)
            except FileExistsError:
                time.sleep(1.0)
                continue
            return timestamp, trial_id, trial_dir

    def _flush_logs(self) -> None:
        if self._closed:
            return
        self._joint_states_file.flush()
        self._task_events_file.flush()
        self._safety_events_file.flush()
        self._contact_events_file.flush()
        self._write_summary()

    def _write_summary(self) -> None:
        if self._closed:
            return
        self._write_json(self._summary_path, self._build_summary())

    def _elapsed_sec(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds / 1_000_000_000.0

    def _summary_notes(self) -> str:
        notes = [self._contact_notes]
        if self._trial_mode == "contact_probe_validation":
            notes.append(
                "contact_probe_validation validates Gazebo contact instrumentation "
                "only; task_completed is not required."
            )
        elif self._trial_mode == "robot_contact_validation":
            robot_validation_count = self._positive_contact_counts.get(
                "robot_validation", 0
            )
            if robot_validation_count > 0:
                notes.append(
                    "robot_contact_validation observed positive robot_validation "
                    "contact samples."
                )
            else:
                notes.append(
                    "robot_contact_validation completed without robot_validation "
                    "contact; tune the joint-space contact pose after Gazebo "
                    "observation if needed."
                )
            if self._force_threshold_violation:
                notes.append(
                    "High contact force exceeded the configured simulation "
                    "violation threshold; robot_contact_validation_success is false."
                )
            if self._force_guard_triggered:
                notes.append(
                    "Force guard triggered and canceled the active robot trajectory; "
                    "robot_contact_validation_success requires the lower-latency "
                    "early contact guard for v0.9 validation."
                )
            if self._early_contact_guard_triggered or self._guarded_contact_stop:
                notes.append(
                    "Early contact guard stopped the approach as the intended v0.9 "
                    "low-force response."
                )
            elif self._force_threshold_warning:
                notes.append(
                    "Contact force exceeded the configured simulation warning "
                    "threshold; inspect max_contact_force before treating the run as "
                    "low-force validation."
                )
        elif self._is_segmented_contact_mode():
            notes.append(
                "segmented_robot_contact_validation uses short checked approach moves and "
                "requires guarded_contact_stop without a force threshold violation "
                "for v1.0 success."
            )
            if self._guarded_contact_stop:
                notes.append(
                    "Segmented guard reported a controlled contact stop before "
                    "continuing the approach."
                )
            if self._force_threshold_violation:
                notes.append(
                    "Contact force exceeded the configured v1.0 violation threshold; "
                    "segmented_contact_success is false."
                )
            if self._pre_approach_contact_detected:
                notes.append(
                    "Unexpected contact was detected before contact_segment_01; "
                    "segmented_contact_success is false."
                )
        elif self._trial_mode == "peg_hole_insertion_validation":
            notes.append(
                "peg_hole_insertion_validation validates v2.0 instrumentation and "
                "logging; v2.1 requires an actual peg-hole collision pair for "
                "insertion contact."
            )
            if (
                self._peg_table_contact_observed
                and not self._peg_hole_contact_observed
            ):
                notes.append(
                    "Peg contact was observed against the table, not the hole; "
                    "insertion contact was not validated."
                )
            if self._initial_contact_detected:
                notes.append(
                    "Initial contact was observed before task execution; insertion "
                    "validation scene requires further cleanup and clean_scene_success "
                    "is false."
                )
            else:
                notes.append(
                    "No initial contact was observed; peg/hole validation scene "
                    "starts cleanly."
                )
            if not self._insertion_depth_available:
                notes.append(
                    "insertion_depth_estimate is null because no validated geometry "
                    "or TF depth rule is available."
                )
        if self._contact_metrics_available and not self._physical_contact_observed:
            notes.append(
                "Contact instrumentation is connected; zero physical contact events "
                "can be expected when the scripted joint-space sequence does not "
                "touch instrumented objects."
            )
        elif not self._contact_metrics_available:
            notes.append("contact_metrics_available: false")
        if self._max_contact_force is None:
            notes.append(
                "max_contact_force is null until a Contacts.wrenches force vector is "
                "observed."
            )
        if self._insertion_success is None:
            notes.append(
                "insertion_success is null because no validated contact-based success "
                "rule is implemented."
            )
        return " ".join(notes)

    def _safe_success(self) -> bool | None:
        if self._trial_mode == "contact_probe_validation":
            if self._trial_failed or self._safety_violations_count > 0:
                return False
            if self._safety_status_observed:
                return True
            return None
        if self._final_trial_status == "guarded_contact_stop":
            return (
                self._safety_violations_count == 0
                and not self._force_threshold_violation
            )
        return self._task_completed and self._safety_violations_count == 0

    def _robot_contact_validation_success(self) -> bool | None:
        if self._trial_mode != "robot_contact_validation":
            return None
        below_force_limit = (
            self._max_contact_force is not None
            and self._force_violation_threshold_n is not None
            and self._max_contact_force < self._force_violation_threshold_n
        )
        return (
            self._physical_contact_observed
            and self._force_extraction_available
            and (
                self._early_contact_guard_triggered
                or self._final_trial_status == "guarded_contact_stop"
            )
            and below_force_limit
            and self._safety_violations_count == 0
        )

    def _segmented_guarded_contact_success(self) -> bool | None:
        if self._trial_mode != "segmented_guarded_contact":
            return None
        below_force_limit = (
            self._max_contact_force is not None and self._max_contact_force < 100.0
        )
        return (
            self._physical_contact_observed
            and self._guarded_contact_stop
            and not self._force_threshold_violation
            and self._safety_violations_count == 0
            and below_force_limit
        )

    def _segmented_contact_success(self) -> bool | None:
        if self._trial_mode != "segmented_robot_contact_validation":
            return None
        return (
            self._physical_contact_observed
            and self._final_trial_status == "guarded_contact_stop"
            and not self._pre_approach_contact_detected
            and not self._force_threshold_violation
            and self._safety_violations_count == 0
        )

    def _peg_hole_instrumentation_success(self) -> bool | None:
        if self._trial_mode != "peg_hole_insertion_validation":
            return None
        return (
            bool(self._contact_topics_connected)
            and self._insertion_metrics_received
            and self._summary_path.exists()
            and self._safety_violations_count == 0
        )

    def _clean_scene_success(self) -> bool | None:
        if self._trial_mode != "peg_hole_insertion_validation":
            return None
        return (
            self._clean_initial_state
            and self._safety_violations_count == 0
            and bool(self._contact_topics_connected)
            and self._insertion_metrics_received
        )

    def _summary_insertion_success_estimate(self) -> bool | None:
        if self._trial_mode != "peg_hole_insertion_validation":
            return self._insertion_success_estimate
        if not self._peg_hole_contact_observed:
            return False
        return (
            self._insertion_hold_reached
            and not self._force_threshold_violation
            and self._final_trial_status in {"completed", "guarded_contact_stop"}
        )

    def _effective_task_started(self) -> bool:
        return self._task_started or (
            self._task_completed and self._total_task_events > 0
        )

    def _is_segmented_contact_mode(self) -> bool:
        return self._trial_mode in {
            "segmented_guarded_contact",
            "segmented_robot_contact_validation",
        }

    def _counts_as_physical_contact(self, source: str) -> bool:
        if self._trial_mode in {
            "robot_contact_validation",
            "segmented_guarded_contact",
            "segmented_robot_contact_validation",
        }:
            return source == "robot_validation"
        if self._trial_mode == "peg_hole_insertion_validation":
            return source in {
                "peg_validation",
                "hole_validation",
                "peg",
                "hole",
                "target",
            }
        return True

    @staticmethod
    def _normalize_trial_mode(value: str) -> str:
        mode = str(value).strip()
        if mode in {
            "baseline_task",
            "contact_probe_validation",
            "robot_contact_validation",
            "segmented_guarded_contact",
            "segmented_robot_contact_validation",
            "peg_hole_insertion_validation",
        }:
            return mode
        return "baseline_task"

    def _message_time_sec(self, message: JointState) -> float:
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self._now_sec()
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def _event_time_sec(self, event: dict[str, Any]) -> float:
        value = event.get("timestamp_ros_sec")
        if isinstance(value, (float, int)):
            return float(value)
        return self._now_sec()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _parse_json_message(self, data: str, source: str) -> dict[str, Any]:
        try:
            parsed = json.loads(data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                f"Received non-JSON {source} message; preserving raw text."
            )
            return {
                "timestamp_ros_sec": self._now_sec(),
                "level": data.split(":", 1)[0].strip() if source == "safety_status" else "",
                "code": "unparsed",
                "message": data,
            }
        if not isinstance(parsed, dict):
            self.get_logger().warning(f"Received non-object JSON from {source}.")
            return {
                "timestamp_ros_sec": self._now_sec(),
                "code": "invalid_json_shape",
                "message": data,
            }
        return parsed

    @staticmethod
    def _coerce_int(value: Any, default: int) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _json_csv_value(value: Any) -> str:
        if value in (None, ""):
            return ""
        return json.dumps(value, sort_keys=True)

    @staticmethod
    def _coerce_optional_float(value: Any) -> float | None:
        if value is None or value == "":
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _coerce_optional_bool(value: Any) -> bool | None:
        if value is None:
            return None
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "1", "yes"}:
                return True
            if normalized in {"false", "0", "no"}:
                return False
        return None

    def _update_contact_topics_seen(self, value: Any) -> None:
        if isinstance(value, list):
            for item in value:
                source = str(item).strip()
                if source:
                    self._contact_topics_seen.add(source)
        elif isinstance(value, str):
            source = value.strip()
            if source:
                self._contact_topics_seen.add(source)

    def _update_contact_topics_configured(self, value: Any) -> None:
        if isinstance(value, dict):
            configured = {
                str(name).strip(): str(topic).strip()
                for name, topic in value.items()
                if str(name).strip() and str(topic).strip()
            }
            if configured:
                self._contact_topics_configured = configured

    def _update_positive_contact_counts(self, value: Any) -> None:
        if not isinstance(value, dict):
            return
        for source, count in value.items():
            source_name = str(source).strip()
            if not source_name:
                continue
            self._positive_contact_counts[source_name] = max(
                self._positive_contact_counts.get(source_name, 0),
                self._coerce_int(count, default=0),
            )

    def _record_contact_diagnostics(
        self,
        collision_pairs: list[str],
        first_collision1: str | None,
        first_collision2: str | None,
        phase: str,
    ) -> None:
        if phase == "uninitialized" and collision_pairs:
            self._initial_contact_detected = True
            self._clean_initial_state = False
            self._uninitialized_contact_count += len(collision_pairs)
            self._merge_unique_pairs(
                collision_pairs,
                self._initial_contact_pair_set,
                self._initial_contact_pairs,
            )
        for pair in collision_pairs:
            if pair in self._collision_pair_set:
                continue
            self._collision_pair_set.add(pair)
            self._collision_pairs.append(pair)
        if (
            self._first_contact_collision1 is None
            and (first_collision1 or first_collision2)
        ):
            self._first_contact_collision1 = first_collision1
            self._first_contact_collision2 = first_collision2
            self._first_contact_phase = phase

    @staticmethod
    def _merge_unique_pairs(
        pairs: list[str], seen: set[str], destination: list[str]
    ) -> None:
        for pair in pairs:
            if pair in seen:
                continue
            seen.add(pair)
            destination.append(pair)

    @staticmethod
    def _coerce_collision_pairs(value: Any) -> list[str]:
        if isinstance(value, list):
            return [str(item) for item in value if str(item)]
        if isinstance(value, str):
            stripped = value.strip()
            if not stripped:
                return []
            try:
                parsed = json.loads(stripped)
            except json.JSONDecodeError:
                return [stripped]
            if isinstance(parsed, list):
                return [str(item) for item in parsed if str(item)]
            return [stripped]
        return []

    @staticmethod
    def _coerce_optional_string(value: Any) -> str | None:
        if value in (None, ""):
            return None
        return str(value)

    @staticmethod
    def _coerce_string_set(value: Any, default: set[str]) -> set[str]:
        if isinstance(value, list):
            return {str(item).strip() for item in value if str(item).strip()}
        if isinstance(value, str):
            source = value.strip()
            return {source} if source else set(default)
        return set(default)

    @staticmethod
    def _format_optional_float(value: float | None) -> str:
        if value is None:
            return ""
        return f"{float(value):.9f}"

    @staticmethod
    def _write_json(path: Path, payload: dict[str, object]) -> None:
        with path.open("w", encoding="utf-8") as json_file:
            json.dump(payload, json_file, indent=2, sort_keys=True)
            json_file.write("\n")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: BaselineTrialManager | None = None

    try:
        node = BaselineTrialManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - top-level node failure logging.
        if node is not None:
            node.get_logger().error(f"Baseline trial manager failed: {exc}")
        else:
            print(f"Baseline trial manager failed during startup: {exc}")
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
