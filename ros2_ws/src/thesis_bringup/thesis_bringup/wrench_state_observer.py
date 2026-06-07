#!/usr/bin/env python3
"""Passive wrench/state observer for research baseline diagnostics."""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean
from typing import Iterable

import numpy as np

import rclpy
from geometry_msgs.msg import Wrench
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from kuka_task_control.robot_kinematics import RobotKinematics


JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]
HOLE_CENTRE_XY = np.array([0.520, -0.200])


def _finite(values: Iterable[float]) -> list[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def _percentile(values: list[float], percentile: float) -> float:
    finite = sorted(_finite(values))
    if not finite:
        return 0.0
    index = min(
        len(finite) - 1,
        max(0, int(round((percentile / 100.0) * (len(finite) - 1)))),
    )
    return finite[index]


class WrenchStateObserver(Node):
    """Write wrench samples grouped by insertion state and peg pose."""

    def __init__(self) -> None:
        super().__init__("wrench_state_observer")
        self.declare_parameter("wrench_topic", "/ft_sensor_wrench")
        self.declare_parameter("state_topic", "/insertion_state")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("output_dir", "/tmp/thesis_tracking_logs")
        self.declare_parameter("summary_period_s", 5.0)

        self._wrench_topic = str(self.get_parameter("wrench_topic").value)
        self._state_topic = str(self.get_parameter("state_topic").value)
        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._output_dir = Path(str(self.get_parameter("output_dir").value))
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._output_dir / "wrench_state_samples.csv"
        self._summary_path = self._output_dir / "wrench_state_summary.md"

        self._csv_file = self._csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(
            [
                "stamp_s",
                "state",
                "fx_n",
                "fy_n",
                "fz_n",
                "force_norm_n",
                "tx_nm",
                "ty_nm",
                "tz_nm",
                "torque_norm_nm",
                "peg_x_m",
                "peg_y_m",
                "peg_z_m",
                "xy_error_m",
            ]
        )

        self._kinematics = RobotKinematics()
        self._state = "UNKNOWN"
        self._current_joints: np.ndarray | None = None
        self._first_stamp: float | None = None
        self._last_stamp: float | None = None
        self._sample_count = 0
        self._state_counts: dict[str, int] = defaultdict(int)
        self._fz_by_state: dict[str, list[float]] = defaultdict(list)
        self._norm_by_state: dict[str, list[float]] = defaultdict(list)
        self._xy_by_state: dict[str, list[float]] = defaultdict(list)
        self._max_abs_fz = 0.0
        self._max_force_norm = 0.0

        self.create_subscription(Wrench, self._wrench_topic, self._on_wrench, 50)
        self.create_subscription(String, self._state_topic, self._on_state, 20)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 50)
        period = max(1.0, float(self.get_parameter("summary_period_s").value))
        self.create_timer(period, self._write_summary)
        self.get_logger().info(
            f"WrenchStateObserver writing {self._wrench_topic} by {self._state_topic} "
            f"to {self._output_dir}"
        )

    def _on_state(self, msg: String) -> None:
        self._state = msg.data or "UNKNOWN"

    def _on_joint_state(self, msg: JointState) -> None:
        position_by_name = {
            name: float(position)
            for name, position in zip(msg.name, msg.position)
        }
        if all(name in position_by_name for name in JOINT_NAMES):
            self._current_joints = np.array(
                [position_by_name[name] for name in JOINT_NAMES],
                dtype=float,
            )

    def _peg_pose(self) -> tuple[float, float, float, float]:
        if self._current_joints is None:
            return math.nan, math.nan, math.nan, math.nan
        peg, _ = self._kinematics.pose(self._current_joints)
        xy_error = float(np.linalg.norm(peg[:2] - HOLE_CENTRE_XY))
        return float(peg[0]), float(peg[1]), float(peg[2]), xy_error

    def _on_wrench(self, msg: Wrench) -> None:
        stamp_s = self.get_clock().now().nanoseconds * 1.0e-9
        fx = float(msg.force.x)
        fy = float(msg.force.y)
        fz = float(msg.force.z)
        tx = float(msg.torque.x)
        ty = float(msg.torque.y)
        tz = float(msg.torque.z)
        force_norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        torque_norm = math.sqrt(tx * tx + ty * ty + tz * tz)
        peg_x, peg_y, peg_z, xy_error = self._peg_pose()

        state = self._state
        self._writer.writerow(
            [
                f"{stamp_s:.9f}",
                state,
                f"{fx:.9f}",
                f"{fy:.9f}",
                f"{fz:.9f}",
                f"{force_norm:.9f}",
                f"{tx:.9f}",
                f"{ty:.9f}",
                f"{tz:.9f}",
                f"{torque_norm:.9f}",
                f"{peg_x:.9f}",
                f"{peg_y:.9f}",
                f"{peg_z:.9f}",
                f"{xy_error:.9f}",
            ]
        )
        self._sample_count += 1
        self._state_counts[state] += 1
        self._fz_by_state[state].append(fz)
        self._norm_by_state[state].append(force_norm)
        if math.isfinite(xy_error):
            self._xy_by_state[state].append(xy_error)
        self._max_abs_fz = max(self._max_abs_fz, abs(fz))
        self._max_force_norm = max(self._max_force_norm, force_norm)
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
            "# Wrench State Summary",
            "",
            f"- wrench_topic: `{self._wrench_topic}`",
            f"- state_topic: `{self._state_topic}`",
            f"- joint_state_topic: `{self._joint_state_topic}`",
            f"- samples: `{self._sample_count}`",
            f"- duration_s: `{duration:.3f}`",
            f"- max_abs_fz_n: `{self._max_abs_fz:.6f}`",
            f"- max_force_norm_n: `{self._max_force_norm:.6f}`",
            "",
            "| State | Samples | Mean Fz N | P95 abs Fz N | Max abs Fz N | Mean norm N | Max norm N | Min XY m | Mean XY m |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        for state in sorted(self._state_counts):
            fz_values = self._fz_by_state[state]
            abs_fz = [abs(value) for value in fz_values]
            norm_values = self._norm_by_state[state]
            xy_values = self._xy_by_state[state]
            lines.append(
                "| "
                f"{state} | "
                f"{self._state_counts[state]} | "
                f"{(mean(fz_values) if fz_values else 0.0):.6f} | "
                f"{_percentile(abs_fz, 95.0):.6f} | "
                f"{(max(abs_fz) if abs_fz else 0.0):.6f} | "
                f"{(mean(norm_values) if norm_values else 0.0):.6f} | "
                f"{(max(norm_values) if norm_values else 0.0):.6f} | "
                f"{(min(xy_values) if xy_values else 0.0):.6f} | "
                f"{(mean(xy_values) if xy_values else 0.0):.6f} |"
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
    node = WrenchStateObserver()
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
