"""Diagnostic-only tool-axis audit for coordinate-based insertion."""

from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class ToolAxisAudit(Node):
    """Compare tool0 local axes against the configured insertion direction."""

    AUDIT_TOPIC = "/tool_axis_audit"
    TOOL_AXES_LOCAL = {
        "tool0_+X": [1.0, 0.0, 0.0],
        "tool0_-X": [-1.0, 0.0, 0.0],
        "tool0_+Y": [0.0, 1.0, 0.0],
        "tool0_-Y": [0.0, -1.0, 0.0],
        "tool0_+Z": [0.0, 0.0, 1.0],
        "tool0_-Z": [0.0, 0.0, -1.0],
    }

    def __init__(self) -> None:
        super().__init__("tool_axis_audit")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("hole_frame", "hole_center")
        self.declare_parameter("axis_align_frame", "axis_align_pose")
        self.declare_parameter("publish_period_sec", 1.0)

        self._world_frame = str(self.get_parameter("world_frame").value)
        self._tool_frame = str(self.get_parameter("tool_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._hole_frame = str(self.get_parameter("hole_frame").value)
        self._axis_align_frame = str(self.get_parameter("axis_align_frame").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(String, self.AUDIT_TOPIC, 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_audit,
        )

        self.get_logger().info(
            "Tool-axis audit started in diagnostic_only_no_motion mode."
        )

    def _publish_audit(self) -> None:
        world_to_tool = self._lookup_transform(self._world_frame, self._tool_frame)
        frames_observed = {
            "world_to_tool0": world_to_tool is not None,
            "world_to_base_link": self._lookup_transform(
                self._world_frame,
                self._base_frame,
            )
            is not None,
            "world_to_hole_center": self._lookup_transform(
                self._world_frame,
                self._hole_frame,
            )
            is not None,
            "world_to_axis_align_pose": self._lookup_transform(
                self._world_frame,
                self._axis_align_frame,
            )
            is not None,
        }

        insertion_axis_world = [0.0, 0.0, -1.0]
        tool_axes_world: dict[str, list[float] | None] = {}
        candidate_alignment_scores: dict[str, dict[str, float | None]] = {}
        best_candidate_tool_axis = None
        best_candidate_dot = None
        best_candidate_angle_deg = None

        if world_to_tool is not None:
            rotation = world_to_tool.transform.rotation
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            for axis_name, local_axis in self.TOOL_AXES_LOCAL.items():
                world_axis = self._normalize(self._rotate_vector(quaternion, local_axis))
                dot = self._dot(world_axis, insertion_axis_world)
                angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
                tool_axes_world[axis_name] = world_axis
                candidate_alignment_scores[axis_name] = {
                    "dot": dot,
                    "angle_deg": angle_deg,
                }
                if best_candidate_dot is None or dot > best_candidate_dot:
                    best_candidate_tool_axis = axis_name
                    best_candidate_dot = dot
                    best_candidate_angle_deg = angle_deg
        else:
            for axis_name in self.TOOL_AXES_LOCAL:
                tool_axes_world[axis_name] = None
                candidate_alignment_scores[axis_name] = {
                    "dot": None,
                    "angle_deg": None,
                }

        payload = {
            "status": "tool_axis_audit_diagnostic_only_no_motion",
            "frames_observed": frames_observed,
            "insertion_axis_world": insertion_axis_world,
            "tool_axes_world": tool_axes_world,
            "candidate_alignment_scores": candidate_alignment_scores,
            "best_candidate_tool_axis": best_candidate_tool_axis,
            "best_candidate_dot": best_candidate_dot,
            "best_candidate_angle_deg": best_candidate_angle_deg,
            "recommended_selected_tool_axis_candidate": best_candidate_tool_axis,
            "recommended_next_step": "compute_orientation_targets",
            "orientation_validated": False,
            "validation_reason": "manual validation required before motion",
            "motion_execution_allowed": False,
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _lookup_transform(self, target_frame: str, source_frame: str) -> Any | None:
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as exc:
            self.get_logger().debug(
                f"TF lookup unavailable for {target_frame} -> {source_frame}: {exc}"
            )
            return None

    @staticmethod
    def _rotate_vector(quaternion_xyzw: list[float], vector: list[float]) -> list[float]:
        qx, qy, qz, qw = quaternion_xyzw
        vx, vy, vz = vector
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return [
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        ]

    @staticmethod
    def _normalize(vector: list[float]) -> list[float]:
        norm = math.sqrt(sum(value * value for value in vector))
        if norm == 0.0:
            return [0.0, 0.0, 0.0]
        return [value / norm for value in vector]

    @staticmethod
    def _dot(first: list[float], second: list[float]) -> float:
        return sum(first[index] * second[index] for index in range(3))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ToolAxisAudit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
