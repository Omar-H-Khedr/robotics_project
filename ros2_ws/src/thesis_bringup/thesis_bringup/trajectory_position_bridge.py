#!/usr/bin/env python3
"""Trajectory -> position_controllers/JointGroupPositionController bridge.

This node lets the kuka_task_control node keep publishing its
JointTrajectory messages while the underlying ros2_control controller
is `position_controllers/JointGroupPositionController` (which expects
`std_msgs/Float64MultiArray`). The bridge subscribes to the trajectory
topic, stores the active multi-point trajectory, and at the controller
update rate (250 Hz) linearly interpolates the current reference
between waypoints and republishes it as a `Float64MultiArray`.

This is a diagnostic bridge: it preserves the JTC's linear-interpolation
semantics so the comparison between JTC and JointGroupPositionController
isolates the controller-type change.

Diagnostic-only: this node does not enforce any safety gating, it just
republishes the active trajectory at a fixed rate. All safety lives in
the upstream admittance_insertion_node and the ros2_control hardware
interface.
"""

from __future__ import annotations

import math
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _duration_to_sec(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


class TrajectoryPositionBridge(Node):
    """Linear-interpolation bridge from JointTrajectory to Float64MultiArray."""

    def __init__(self) -> None:
        super().__init__("trajectory_position_bridge")
        self.declare_parameter(
            "input_trajectory_topic",
            "/joint_trajectory_controller/joint_trajectory",
        )
        self.declare_parameter(
            "output_command_topic",
            "/position_controller/commands",
        )
        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )
        self.declare_parameter("interp_rate_hz", 250.0)
        self.declare_parameter("publish_on_update", True)
        self.declare_parameter("hold_last_point", True)

        self._input_topic = str(self.get_parameter("input_trajectory_topic").value)
        self._output_topic = str(self.get_parameter("output_command_topic").value)
        self._joint_names = [
            str(name) for name in self.get_parameter("joint_names").value
        ]
        self._interp_rate = float(self.get_parameter("interp_rate_hz").value)
        self._publish_on_update = bool(self.get_parameter("publish_on_update").value)
        self._hold_last_point = bool(self.get_parameter("hold_last_point").value)

        self._active_trajectory: JointTrajectory | None = None
        self._active_start_s: float | None = None
        self._active_total_s: float = 0.0
        self._last_published_positions: list[float] | None = None
        self._command_count = 0
        self._interp_count = 0

        self._cmd_pub = self.create_publisher(
            Float64MultiArray, self._output_topic, 10,
        )
        self.create_subscription(
            JointTrajectory, self._input_topic, self._on_trajectory, 10,
        )

        period = 1.0 / max(1.0, self._interp_rate)
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"TrajectoryPositionBridge: {self._input_topic} -> {self._output_topic} "
            f"@ {self._interp_rate:.1f} Hz, joints={self._joint_names}"
        )

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.joint_names or not msg.points:
            return
        self._active_trajectory = msg
        self._active_start_s = self.get_clock().now().nanoseconds * 1.0e-9
        final_t = _duration_to_sec(msg.points[-1].time_from_start)
        self._active_total_s = max(0.0, final_t)
        self._command_count += 1
        self.get_logger().info(
            f"Bridge received trajectory: points={len(msg.points)}, "
            f"total_duration={self._active_total_s:.3f}s"
        )

    def _interpolate(self, elapsed_s: float) -> list[float] | None:
        traj = self._active_trajectory
        if traj is None or not traj.points:
            return None
        points = list(traj.points)
        if len(points) == 1:
            return [float(p) for p in points[0].positions]
        previous = points[0]
        previous_t = _duration_to_sec(previous.time_from_start)
        if elapsed_s <= previous_t:
            return [float(p) for p in previous.positions]
        for point in points[1:]:
            current_t = _duration_to_sec(point.time_from_start)
            if elapsed_s <= current_t:
                span = max(1.0e-9, current_t - previous_t)
                ratio = min(1.0, max(0.0, (elapsed_s - previous_t) / span))
                return [
                    float(a) + ratio * (float(b) - float(a))
                    for a, b in zip(previous.positions, point.positions)
                ]
            previous = point
            previous_t = current_t
        if self._hold_last_point:
            return [float(p) for p in points[-1].positions]
        return None

    def _on_timer(self) -> None:
        if self._active_trajectory is None or self._active_start_s is None:
            return
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        elapsed = now_s - self._active_start_s
        positions = self._interpolate(elapsed)
        if positions is None:
            return
        self._last_published_positions = positions
        self._interp_count += 1
        if self._publish_on_update:
            msg = Float64MultiArray()
            msg.data = list(positions)
            self._cmd_pub.publish(msg)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPositionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
