#!/usr/bin/env python3
"""Passive joint trajectory controller tracking observer."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from statistics import mean
from typing import Iterable

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _finite(values: Iterable[float]) -> list[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def _rms(values: Iterable[float]) -> float:
    finite = _finite(values)
    if not finite:
        return 0.0
    return math.sqrt(sum(value * value for value in finite) / len(finite))


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((percentile / 100.0) * (len(ordered) - 1)))))
    return ordered[index]


def _duration_to_sec(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


class TrajectoryTrackingObserver(Node):
    """Write JTC reference/feedback/error samples and rolling summaries."""

    def __init__(self) -> None:
        super().__init__("trajectory_tracking_observer")
        self.declare_parameter(
            "state_topic",
            "/joint_trajectory_controller/controller_state",
        )
        self.declare_parameter(
            "command_topic",
            "/joint_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("output_dir", "/tmp/thesis_tracking_logs")
        self.declare_parameter("summary_period_s", 5.0)

        self._state_topic = str(self.get_parameter("state_topic").value)
        self._command_topic = str(self.get_parameter("command_topic").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._output_dir = Path(str(self.get_parameter("output_dir").value))
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._output_dir / "trajectory_tracking_samples.csv"
        self._command_path = self._output_dir / "trajectory_commands.csv"
        self._summary_path = self._output_dir / "trajectory_tracking_summary.md"

        self._csv_file = self._csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._csv_file)
        self._command_file = self._command_path.open("w", newline="", encoding="utf-8")
        self._command_writer = csv.writer(self._command_file)
        self._command_writer.writerow(
            [
                "receipt_stamp_s",
                "joint_names",
                "point_count",
                "final_time_from_start_s",
                "final_positions_rad",
            ]
        )
        self._header_written = False
        self._joint_names: list[str] = []
        self._active_command: JointTrajectory | None = None
        self._active_command_start_s: float | None = None
        self._command_count = 0
        self._sample_count = 0
        self._jtc_state_sample_count = 0
        self._max_abs_errors: list[float] = []
        self._rms_errors: list[float] = []
        self._final_max_abs_error = 0.0
        self._first_stamp: float | None = None
        self._last_stamp: float | None = None

        self.create_subscription(
            JointTrajectoryControllerState,
            self._state_topic,
            self._on_state,
            20,
        )
        self.create_subscription(
            JointTrajectory,
            self._command_topic,
            self._on_command,
            20,
        )
        self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._on_joint_state,
            50,
        )
        period = max(1.0, float(self.get_parameter("summary_period_s").value))
        self.create_timer(period, self._write_summary)
        self.get_logger().info(
            "TrajectoryTrackingObserver writing "
            f"{self._command_topic} vs {self._joint_state_topic} to {self._output_dir}"
        )

    def _write_header(self, joint_names: list[str]) -> None:
        header = [
            "stamp_s",
            "max_abs_position_error_rad",
            "rms_position_error_rad",
        ]
        for name in joint_names:
            header.extend(
                [
                    f"{name}_reference_rad",
                    f"{name}_feedback_rad",
                    f"{name}_error_rad",
                ]
            )
        self._writer.writerow(header)
        self._header_written = True
        self._joint_names = joint_names

    def _on_state(self, msg: JointTrajectoryControllerState) -> None:
        self._jtc_state_sample_count += 1

    def _on_command(self, msg: JointTrajectory) -> None:
        if not msg.joint_names or not msg.points:
            return
        self._active_command = msg
        self._active_command_start_s = self.get_clock().now().nanoseconds * 1.0e-9
        self._command_count += 1
        final_point = msg.points[-1]
        self._command_writer.writerow(
            [
                f"{self._active_command_start_s:.9f}",
                " ".join(msg.joint_names),
                len(msg.points),
                f"{_duration_to_sec(final_point.time_from_start):.9f}",
                " ".join(f"{value:.9f}" for value in final_point.positions),
            ]
        )
        self._command_file.flush()
        self.get_logger().info(
            f"Observed trajectory command: points={len(msg.points)}, "
            f"duration={_duration_to_sec(final_point.time_from_start):.3f}s"
        )

    def _interpolate_reference(self, elapsed_s: float) -> tuple[list[str], list[float]] | None:
        command = self._active_command
        if command is None or not command.points:
            return None
        points = list(command.points)
        if len(points) == 1:
            return list(command.joint_names), list(points[0].positions)
        previous = points[0]
        previous_t = _duration_to_sec(previous.time_from_start)
        if elapsed_s <= previous_t:
            return list(command.joint_names), list(previous.positions)
        for point in points[1:]:
            current_t = _duration_to_sec(point.time_from_start)
            if elapsed_s <= current_t:
                span = max(1.0e-9, current_t - previous_t)
                ratio = min(1.0, max(0.0, (elapsed_s - previous_t) / span))
                positions = [
                    float(a) + ratio * (float(b) - float(a))
                    for a, b in zip(previous.positions, point.positions)
                ]
                return list(command.joint_names), positions
            previous = point
            previous_t = current_t
        return list(command.joint_names), list(points[-1].positions)

    def _on_joint_state(self, msg: JointState) -> None:
        if self._active_command is None or self._active_command_start_s is None:
            return
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        reference = self._interpolate_reference(now_s - self._active_command_start_s)
        if reference is None:
            return
        joint_names, reference_positions = reference
        position_by_name = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
        }
        if not all(name in position_by_name for name in joint_names):
            return
        feedback_positions = [position_by_name[name] for name in joint_names]
        errors = [
            ref - feedback
            for ref, feedback in zip(reference_positions, feedback_positions)
        ]
        self._record_sample(now_s, joint_names, reference_positions, feedback_positions, errors)

    def _record_sample(
        self,
        stamp_s: float,
        joint_names: list[str],
        reference_positions: list[float],
        feedback_positions: list[float],
        errors: list[float],
    ) -> None:
        if not self._header_written:
            self._write_header(joint_names)
        max_abs_error = max((abs(value) for value in errors), default=0.0)
        rms_error = _rms(errors)

        row: list[float | str] = [f"{stamp_s:.9f}", f"{max_abs_error:.9f}", f"{rms_error:.9f}"]
        for index, _name in enumerate(joint_names):
            reference = reference_positions[index] if index < len(reference_positions) else 0.0
            feedback = feedback_positions[index] if index < len(feedback_positions) else 0.0
            error = errors[index] if index < len(errors) else 0.0
            row.extend([f"{reference:.9f}", f"{feedback:.9f}", f"{error:.9f}"])
        self._writer.writerow(row)
        self._sample_count += 1
        self._max_abs_errors.append(max_abs_error)
        self._rms_errors.append(rms_error)
        self._final_max_abs_error = max_abs_error
        if self._first_stamp is None:
            self._first_stamp = stamp_s
        self._last_stamp = stamp_s

    def _write_summary(self) -> None:
        if self._csv_file.closed:
            return
        self._csv_file.flush()
        max_error = max(self._max_abs_errors, default=0.0)
        mean_max_error = mean(self._max_abs_errors) if self._max_abs_errors else 0.0
        p95_error = _percentile(self._max_abs_errors, 95.0)
        mean_rms_error = mean(self._rms_errors) if self._rms_errors else 0.0
        duration = (
            (self._last_stamp - self._first_stamp)
            if self._first_stamp is not None and self._last_stamp is not None
            else 0.0
        )
        lines = [
            "# Trajectory Tracking Summary",
            "",
            f"- state_topic: `{self._state_topic}`",
            f"- command_topic: `{self._command_topic}`",
            f"- joint_state_topic: `{self._joint_state_topic}`",
            f"- observed_commands: `{self._command_count}`",
            f"- jtc_state_samples: `{self._jtc_state_sample_count}`",
            f"- samples: `{self._sample_count}`",
            f"- duration_s: `{duration:.3f}`",
            f"- joints: `{', '.join(self._joint_names)}`",
            f"- max_abs_position_error_rad: `{max_error:.6f}`",
            f"- mean_max_abs_position_error_rad: `{mean_max_error:.6f}`",
            f"- p95_max_abs_position_error_rad: `{p95_error:.6f}`",
            f"- mean_rms_position_error_rad: `{mean_rms_error:.6f}`",
            f"- final_max_abs_position_error_rad: `{self._final_max_abs_error:.6f}`",
            "",
            "This observer is passive. It does not publish commands or alter controller behavior.",
        ]
        self._summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def destroy_node(self) -> bool:
        self._write_summary()
        if not self._csv_file.closed:
            self._csv_file.close()
        if not self._command_file.closed:
            self._command_file.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TrajectoryTrackingObserver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
