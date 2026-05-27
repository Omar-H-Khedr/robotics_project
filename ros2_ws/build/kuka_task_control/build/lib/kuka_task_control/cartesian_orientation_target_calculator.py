"""Diagnostic-only Cartesian orientation target calculator."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class CartesianOrientationTargetCalculator(Node):
    """Compute target quaternions without commanding robot motion."""

    TARGET_TOPIC = "/cartesian_orientation_targets"
    DEFAULT_CONFIG_FILE = "peg_hole_cartesian_targets.yaml"
    TARGET_POSE_FRAMES = (
        "staging_pose",
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
        "retreat_pose",
    )
    TOOL_AXES_LOCAL = {
        "tool0_+X": [1.0, 0.0, 0.0],
        "tool0_-X": [-1.0, 0.0, 0.0],
        "tool0_+Y": [0.0, 1.0, 0.0],
        "tool0_-Y": [0.0, -1.0, 0.0],
        "tool0_+Z": [0.0, 0.0, 1.0],
        "tool0_-Z": [0.0, 0.0, -1.0],
    }

    def __init__(self) -> None:
        super().__init__("cartesian_orientation_target_calculator")
        self.declare_parameter("config_path", "")
        self.declare_parameter("publish_period_sec", 1.0)

        self._config = self._load_config(self._resolve_config_path())
        self._world_frame = str(self._config.get("world_frame", "world"))
        self._base_frame = str(self._config.get("robot_base_frame", "base_link"))
        self._tool_frame = str(self._config.get("tool_frame", "tool0"))
        self._targets = self._config.get("targets", {})
        if not isinstance(self._targets, dict):
            raise ValueError("peg_hole_cartesian_targets.yaml field 'targets' must be a map")

        orientation_config = self._orientation_config()
        self._selected_tool_axis_candidate = str(
            orientation_config.get("selected_tool_axis_candidate", "tool0_+Z")
        )
        self._yaw_reference_mode = str(
            orientation_config.get(
                "yaw_reference_mode",
                "keep_current_tool_yaw_if_possible",
            )
        )
        self._insertion_axis_world = self._configured_insertion_axis(orientation_config)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(String, self.TARGET_TOPIC, 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_targets,
        )

        self.get_logger().info(
            "Cartesian orientation target calculator started in "
            "diagnostic_only_no_motion mode."
        )

    def _resolve_config_path(self) -> Path:
        configured_path = str(self.get_parameter("config_path").value).strip()
        if configured_path:
            return Path(configured_path).expanduser()
        return (
            Path(get_package_share_directory("kuka_task_control"))
            / "config"
            / self.DEFAULT_CONFIG_FILE
        )

    @staticmethod
    def _load_config(config_path: Path) -> dict[str, Any]:
        with config_path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}
        if not isinstance(config, dict):
            raise ValueError(f"Cartesian insertion config must be a map: {config_path}")
        return config

    def _orientation_config(self) -> dict[str, Any]:
        orientation_config = self._config.get("orientation_planning", {})
        return orientation_config if isinstance(orientation_config, dict) else {}

    def _configured_insertion_axis(
        self,
        orientation_config: dict[str, Any],
    ) -> list[float]:
        axis = orientation_config.get("insertion_axis_world")
        if not self._is_vector(axis, 3):
            axis = self._config.get("insertion_axis_world")
        if not self._is_vector(axis, 3):
            insertion_axis = self._config.get("insertion_axis", {})
            if isinstance(insertion_axis, dict):
                axis = insertion_axis.get("direction_xyz")
        if not self._is_vector(axis, 3):
            axis = [0.0, 0.0, -1.0]
        return self._normalize([float(value) for value in axis])

    def _publish_targets(self) -> None:
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
                "hole_center",
            )
            is not None,
            "world_to_axis_align_pose": self._lookup_transform(
                self._world_frame,
                "axis_align_pose",
            )
            is not None,
        }

        current_tool_orientation_world = None
        current_tool_axes_world = {
            axis_name: None for axis_name in self.TOOL_AXES_LOCAL
        }
        desired_quaternion = None
        expected_alignment = None
        orientation_target_available = False
        yaw_reference_unresolved = True
        orientation_targets_available = False

        if world_to_tool is not None:
            rotation = world_to_tool.transform.rotation
            current_tool_orientation_world = [
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ]
            current_tool_axes_world = self._tool_axes_world(
                current_tool_orientation_world
            )
            desired_quaternion, yaw_reference_unresolved = (
                self._desired_orientation_from_current_tool(
                    current_tool_orientation_world
                )
            )
            if desired_quaternion is not None:
                desired_axis_world = self._rotate_vector(
                    desired_quaternion,
                    self.TOOL_AXES_LOCAL[self._selected_tool_axis_candidate],
                )
                dot = self._dot(
                    self._normalize(desired_axis_world),
                    self._insertion_axis_world,
                )
                clamped_dot = max(-1.0, min(1.0, dot))
                alignment_dot = 1.0 if abs(clamped_dot - 1.0) < 1e-9 else clamped_dot
                alignment_angle_deg = 0.0 if alignment_dot == 1.0 else math.degrees(
                    math.acos(alignment_dot)
                )
                expected_alignment = {
                    "selected_tool_axis_world": self._normalize(desired_axis_world),
                    "dot": alignment_dot,
                    "angle_deg": alignment_angle_deg,
                }
                orientation_target_available = True
                orientation_targets_available = True

        desired_orientations_world = {}
        for target_name in self.TARGET_POSE_FRAMES:
            target_pose, source = self._target_pose_from_tf_or_yaml(target_name)
            desired_orientations_world[target_name] = {
                "target_pose_world": target_pose,
                "target_pose_source": source,
                "orientation_xyzw": desired_quaternion,
                "selected_tool_axis_aligned_to": self._insertion_axis_world,
                "orientation_target_available": orientation_target_available,
                "orientation_source": "cartesian_orientation_targets"
                if orientation_target_available
                else None,
                "expected_alignment_after_orientation": expected_alignment,
                "motion_execution_allowed": False,
            }

        payload = {
            "status": "orientation_targets_diagnostic_only_no_motion",
            "frames_observed": frames_observed,
            "selected_tool_axis_candidate": self._selected_tool_axis_candidate,
            "insertion_axis_world": self._insertion_axis_world,
            "current_tool_orientation_world": current_tool_orientation_world,
            "current_tool_axes_world": current_tool_axes_world,
            "desired_orientations_world": desired_orientations_world,
            "expected_alignment_after_orientation": expected_alignment,
            "yaw_reference_mode": self._yaw_reference_mode,
            "yaw_reference_unresolved": yaw_reference_unresolved,
            "orientation_targets_available": orientation_targets_available,
            "orientation_validated": False,
            "motion_execution_allowed": False,
            "validation_reason": (
                "orientation target computed but not validated by IK or motion"
            ),
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _desired_orientation_from_current_tool(
        self,
        current_quaternion_xyzw: list[float],
    ) -> tuple[list[float] | None, bool]:
        if self._selected_tool_axis_candidate != "tool0_+Z":
            return None, True

        target_z = self._insertion_axis_world
        current_x = self._rotate_vector(current_quaternion_xyzw, [1.0, 0.0, 0.0])
        current_y = self._rotate_vector(current_quaternion_xyzw, [0.0, 1.0, 0.0])

        desired_x = self._project_onto_plane(current_x, target_z)
        if desired_x is not None:
            desired_y = self._normalize(self._cross(target_z, desired_x))
            return self._quaternion_from_basis(desired_x, desired_y, target_z), False

        desired_y = self._project_onto_plane(current_y, target_z)
        if desired_y is not None:
            desired_x = self._normalize(self._cross(desired_y, target_z))
            return self._quaternion_from_basis(desired_x, desired_y, target_z), False

        fallback_x = self._orthogonal_unit_vector(target_z)
        fallback_y = self._normalize(self._cross(target_z, fallback_x))
        return self._quaternion_from_basis(fallback_x, fallback_y, target_z), True

    def _tool_axes_world(
        self,
        quaternion_xyzw: list[float],
    ) -> dict[str, list[float]]:
        return {
            axis_name: self._normalize(self._rotate_vector(quaternion_xyzw, axis))
            for axis_name, axis in self.TOOL_AXES_LOCAL.items()
        }

    def _target_pose_from_tf_or_yaml(
        self,
        name: str,
    ) -> tuple[dict[str, Any] | None, str | None]:
        pose = self._lookup_pose(self._world_frame, name)
        if pose is not None:
            pose["frame_source"] = "tf"
            return pose, "tf"

        target = self._targets.get(name)
        if not isinstance(target, dict):
            return None, None
        position = target.get("position_xyz")
        if not self._is_vector(position, 3):
            return None, None
        pose = {
            "frame": str(target.get("frame", self._world_frame)),
            "child_frame": name,
            "position_xyz": [float(value) for value in position],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "frame_source": "yaml_fallback",
        }
        return pose, "yaml_fallback"

    def _lookup_pose(self, target_frame: str, source_frame: str) -> dict[str, Any] | None:
        transform = self._lookup_transform(target_frame, source_frame)
        if transform is None:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "frame": target_frame,
            "child_frame": source_frame,
            "position_xyz": [translation.x, translation.y, translation.z],
            "orientation_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w],
        }

    def _lookup_transform(self, target_frame: str, source_frame: str) -> Any | None:
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as exc:
            self.get_logger().debug(
                f"TF lookup unavailable for {target_frame} -> {source_frame}: {exc}"
            )
            return None

    @classmethod
    def _project_onto_plane(
        cls,
        vector: list[float],
        plane_normal: list[float],
    ) -> list[float] | None:
        projected = [
            vector[index] - cls._dot(vector, plane_normal) * plane_normal[index]
            for index in range(3)
        ]
        norm = math.sqrt(sum(value * value for value in projected))
        if norm < 1e-9:
            return None
        return [value / norm for value in projected]

    @classmethod
    def _orthogonal_unit_vector(cls, vector: list[float]) -> list[float]:
        reference = [1.0, 0.0, 0.0]
        if abs(cls._dot(vector, reference)) > 0.9:
            reference = [0.0, 1.0, 0.0]
        return cls._normalize(cls._cross(reference, vector))

    @staticmethod
    def _quaternion_from_basis(
        x_axis: list[float],
        y_axis: list[float],
        z_axis: list[float],
    ) -> list[float]:
        m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
        m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
        m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
        trace = m00 + m11 + m22

        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * scale
            qx = (m21 - m12) / scale
            qy = (m02 - m20) / scale
            qz = (m10 - m01) / scale
        elif m00 > m11 and m00 > m22:
            scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            qw = (m21 - m12) / scale
            qx = 0.25 * scale
            qy = (m01 + m10) / scale
            qz = (m02 + m20) / scale
        elif m11 > m22:
            scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            qw = (m02 - m20) / scale
            qx = (m01 + m10) / scale
            qy = 0.25 * scale
            qz = (m12 + m21) / scale
        else:
            scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            qw = (m10 - m01) / scale
            qx = (m02 + m20) / scale
            qy = (m12 + m21) / scale
            qz = 0.25 * scale

        return CartesianOrientationTargetCalculator._normalize([qx, qy, qz, qw])

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
    def _cross(first: list[float], second: list[float]) -> list[float]:
        return [
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        ]

    @staticmethod
    def _dot(first: list[float], second: list[float]) -> float:
        return sum(first[index] * second[index] for index in range(3))

    @staticmethod
    def _normalize(vector: list[float]) -> list[float]:
        norm = math.sqrt(sum(value * value for value in vector))
        if norm == 0.0:
            return [0.0 for _ in vector]
        return [value / norm for value in vector]

    @staticmethod
    def _is_vector(value: Any, length: int) -> bool:
        return isinstance(value, list) and len(value) == length


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CartesianOrientationTargetCalculator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
