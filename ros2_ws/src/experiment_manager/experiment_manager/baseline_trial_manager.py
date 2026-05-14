"""Structured trial logger for the Gazebo KUKA peg-in-hole baseline."""

from __future__ import annotations

import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import TextIO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class BaselineTrialManager(Node):
    """Record reproducible metadata and topic events for one baseline trial."""

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
        self.declare_parameter("results_root", "results/baseline_trials")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("task_phase_topic", "/task_phase")
        self.declare_parameter("safety_status_topic", "/safety_status")

        self._results_root = Path(
            self.get_parameter("results_root").get_parameter_value().string_value
        )
        self._joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self._task_phase_topic = (
            self.get_parameter("task_phase_topic").get_parameter_value().string_value
        )
        self._safety_status_topic = (
            self.get_parameter("safety_status_topic").get_parameter_value().string_value
        )

        self._start_time = self.get_clock().now()
        self._timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        self._trial_id = f"baseline_v0_1_{self._timestamp}"
        self._trial_dir = self._results_root / self._trial_id
        self._trial_dir.mkdir(parents=True, exist_ok=False)

        self._joint_state_count = 0
        self._task_event_count = 0
        self._safety_event_count = 0
        self._safety_violation_count = 0
        self._last_task_phase = "uninitialized"
        self._last_safety_status = "uninitialized"
        self._closed = False

        self._metadata_path = self._trial_dir / "trial_metadata.json"
        self._summary_path = self._trial_dir / "trial_summary.json"
        self._metadata = self._build_metadata()
        self._write_json(self._metadata_path, self._metadata)

        self._joint_states_file = self._open_csv("joint_states.csv")
        self._task_events_file = self._open_csv("task_events.csv")
        self._safety_events_file = self._open_csv("safety_events.csv")
        self._joint_states_writer = csv.writer(self._joint_states_file)
        self._task_events_writer = csv.writer(self._task_events_file)
        self._safety_events_writer = csv.writer(self._safety_events_file)
        self._write_headers()

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
            self._safety_status_topic,
            self._on_safety_status,
            100,
        )
        self.create_timer(5.0, self._flush_logs)

        self.get_logger().info(f"Started baseline trial logging: {self._trial_dir}")
        self.get_logger().info(
            f"Recording {self._joint_states_topic}, {self._task_phase_topic}, "
            f"and {self._safety_status_topic}"
        )

    def _on_joint_state(self, message: JointState) -> None:
        elapsed = self._elapsed_sec()
        positions = {name: value for name, value in zip(message.name, message.position)}
        velocities = {name: value for name, value in zip(message.name, message.velocity)}
        efforts = {name: value for name, value in zip(message.name, message.effort)}

        for joint_name in self.JOINT_NAMES:
            self._joint_states_writer.writerow(
                [
                    f"{elapsed:.9f}",
                    joint_name,
                    self._format_optional_float(positions.get(joint_name)),
                    self._format_optional_float(velocities.get(joint_name)),
                    self._format_optional_float(efforts.get(joint_name)),
                ]
            )
        self._joint_state_count += 1

    def _on_task_phase(self, message: String) -> None:
        elapsed = self._elapsed_sec()
        phase = message.data
        self._last_task_phase = phase
        self._task_events_writer.writerow([f"{elapsed:.9f}", phase])
        self._task_event_count += 1
        self._task_events_file.flush()

    def _on_safety_status(self, message: String) -> None:
        elapsed = self._elapsed_sec()
        status = message.data
        self._last_safety_status = status
        level = status.split(":", 1)[0].strip() if ":" in status else status.strip()
        if level == "VIOLATION":
            self._safety_violation_count += 1
        self._safety_events_writer.writerow([f"{elapsed:.9f}", level, status])
        self._safety_event_count += 1
        self._safety_events_file.flush()

    def close(self) -> None:
        if self._closed:
            return

        self._flush_logs()
        summary = self._build_summary()
        self._write_json(self._summary_path, summary)

        for file_handle in (
            self._joint_states_file,
            self._task_events_file,
            self._safety_events_file,
        ):
            file_handle.close()

        self._closed = True
        self.get_logger().info(f"Wrote baseline trial summary: {self._summary_path}")

    def _build_metadata(self) -> dict[str, object]:
        return {
            "timestamp": self._timestamp,
            "trial_id": self._trial_id,
            "simulator": "Gazebo",
            "robot": "KUKA LBR iisy 3 R760",
            "task": "peg-in-hole baseline",
            "controller": "joint_trajectory_controller",
            "topics": {
                "joint_states": self._joint_states_topic,
                "task_phase": self._task_phase_topic,
                "safety_status": self._safety_status_topic,
            },
        }

    def _build_summary(self) -> dict[str, object]:
        execution_time_sec = self._elapsed_sec()
        return {
            "trial_id": self._trial_id,
            "task_success": None,
            "insertion_success": None,
            "collision_events": None,
            "max_contact_force": None,
            "safety_violations": self._safety_violation_count,
            "execution_time_sec": execution_time_sec,
            "safe_success": None,
            "joint_state_messages": self._joint_state_count,
            "task_events": self._task_event_count,
            "safety_events": self._safety_event_count,
            "last_task_phase": self._last_task_phase,
            "last_safety_status": self._last_safety_status,
            "notes": (
                "Contact metrics and task-success labeling are placeholders in "
                "Research Baseline v0.1."
            ),
        }

    def _write_headers(self) -> None:
        self._joint_states_writer.writerow(
            ["elapsed_sec", "joint_name", "position", "velocity", "effort"]
        )
        self._task_events_writer.writerow(["elapsed_sec", "phase"])
        self._safety_events_writer.writerow(["elapsed_sec", "level", "status"])
        self._flush_logs()

    def _open_csv(self, name: str) -> TextIO:
        return (self._trial_dir / name).open("w", encoding="utf-8", newline="")

    def _flush_logs(self) -> None:
        if self._closed:
            return
        self._joint_states_file.flush()
        self._task_events_file.flush()
        self._safety_events_file.flush()

    def _elapsed_sec(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds / 1_000_000_000.0

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
