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
        self._force_violation_threshold_n: float | None = 100.0
        self._force_extraction_available = False
        self._force_extraction_method = FORCE_EXTRACTION_METHOD
        self._insertion_attempted = False
        self._insertion_hold_reached = False
        self._insertion_success: bool | None = None
        self._insertion_success_estimate: bool | None = None
        self._contact_metrics_available = False
        self._contact_topics_configured: dict[str, str] = {}
        self._contact_topics_connected: set[str] = set()
        self._contact_messages_observed = False
        self._physical_contact_observed = False
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
        if event_type in {"sequence_started", "phase_started"}:
            self._task_started = True
        if event_type == "phase_succeeded":
            self._completed_phases_count += 1
        elif event_type == "sequence_completed":
            self._task_started = True
            self._task_completed = True
        elif event_type == "sequence_failed":
            self._trial_failed = True
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
        elif event_type == "early_contact_guard_triggered":
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

        self._task_events_writer.writerow(
            [
                f"{ros_time_sec:.9f}",
                event_type,
                phase,
                event.get("pose_index", ""),
                event.get("total_poses", ""),
                event.get("safety_tag", ""),
                event.get("message", ""),
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
        detail = str(event.get("message", message.data))

        if source:
            self._contact_topics_seen.add(source)
            self._contact_topics_connected.add(source)
        self._contact_messages_observed = True
        positive_physical_contact = (
            contact_count > 0 and self._counts_as_physical_contact(source)
        )
        if positive_physical_contact and event_type in {"contact_started", "unknown"}:
            self._contact_episode_count += 1
            self._contact_events_count = self._contact_episode_count
            self._physical_contact_observed = True
        elif positive_physical_contact:
            self._physical_contact_observed = True
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
        is_robot_contact_validation = self._trial_mode == "robot_contact_validation"
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
                else "robot-to-object contact validation"
                if is_robot_contact_validation
                else "peg-in-hole baseline"
            ),
            "controller": (
                "none" if is_contact_probe_validation else "joint_trajectory_controller"
            ),
            "framework_version": (
                "v0.9" if is_robot_contact_validation else "v0.5"
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
                "reported without failing the trial."
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
            "contact_events_count": self._contact_events_count,
            "contact_episode_count": self._contact_episode_count,
            "contact_samples_count": self._contact_samples_count,
            "max_contact_force": self._max_contact_force,
            "force_threshold_warning": self._force_threshold_warning,
            "force_threshold_violation": self._force_threshold_violation,
            "force_guard_triggered": self._force_guard_triggered,
            "force_guard_trigger_force": self._force_guard_trigger_force,
            "force_guard_threshold": self._force_guard_threshold,
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
            "insertion_success_estimate": self._insertion_success_estimate,
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

    def _effective_task_started(self) -> bool:
        return self._task_started or (
            self._task_completed and self._total_task_events > 0
        )

    def _counts_as_physical_contact(self, source: str) -> bool:
        if self._trial_mode == "robot_contact_validation":
            return source == "robot_validation"
        return True

    @staticmethod
    def _normalize_trial_mode(value: str) -> str:
        mode = str(value).strip()
        if mode in {
            "baseline_task",
            "contact_probe_validation",
            "robot_contact_validation",
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
