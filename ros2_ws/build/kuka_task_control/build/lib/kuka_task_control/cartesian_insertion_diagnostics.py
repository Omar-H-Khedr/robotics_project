"""Diagnostic-only Cartesian peg/hole insertion frame monitor."""

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


class CartesianInsertionDiagnostics(Node):
    """Publish frame/target distances without commanding robot motion."""

    DIAGNOSTIC_TOPIC = "/cartesian_insertion_diagnostics"
    DEFAULT_CONFIG_FILE = "peg_hole_cartesian_targets.yaml"
    OPTIONAL_OBJECT_FRAMES = (
        "hole_center",
        "insertion_axis_marker",
        "staging_pose",
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
        "retreat_pose",
    )
    TARGET_POSE_FRAMES = (
        "hole_center",
        "staging_pose",
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
        "retreat_pose",
    )
    INSERTION_ALIGNED_TARGETS = (
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
    )
    XY_TOLERANCE_M = 0.002

    def __init__(self) -> None:
        super().__init__("cartesian_insertion_diagnostics")
        self.declare_parameter("config_path", "")
        self.declare_parameter("publish_period_sec", 1.0)

        self._config = self._load_config(self._resolve_config_path())
        self._world_frame = str(self._config.get("world_frame", "world"))
        self._base_frame = str(self._config.get("robot_base_frame", "base_link"))
        self._tool_frame = str(self._config.get("tool_frame", "tool0"))
        self._targets = self._config.get("targets", {})
        if not isinstance(self._targets, dict):
            raise ValueError("peg_hole_cartesian_targets.yaml field 'targets' must be a map")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(String, self.DIAGNOSTIC_TOPIC, 10)
        self._phase_publisher = self.create_publisher(String, "/task_phase", 10)
        self._trial_status_publisher = self.create_publisher(String, "/trial_status", 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_diagnostics,
        )

        self.get_logger().info(
            "Cartesian insertion diagnostics started in diagnostic_only_no_motion mode."
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

    def _publish_diagnostics(self) -> None:
        current_base_pose_world = self._lookup_pose(self._world_frame, self._base_frame)
        current_tool_pose_world = self._lookup_pose(self._world_frame, self._tool_frame)
        current_tool_pose_base = self._lookup_pose(self._base_frame, self._tool_frame)
        object_frames_world = self._lookup_optional_object_frames()
        target_poses = {}
        frame_sources = {}
        for frame_name in self.TARGET_POSE_FRAMES:
            pose, source = self._target_pose_from_tf_or_yaml(frame_name)
            target_poses[frame_name] = pose
            frame_sources[frame_name] = source

        hole_center_world = target_poses["hole_center"]
        staging_pose_world = target_poses["staging_pose"]
        axis_align_pose_world = target_poses["axis_align_pose"]
        insertion_axis_world = self._insertion_axis()
        geometry_validity = self._geometry_validity(target_poses)
        orientation_validated = self._tool_orientation_validated()
        safety_guard_active = bool(self._config.get("safety_guard_active", False))
        ik_available = bool(self._config.get("ik_available", False))
        motion_execution_allowed = (
            geometry_validity["cartesian_geometry_valid"]
            and ik_available
            and orientation_validated
            and safety_guard_active
        )

        payload = {
            "status": "diagnostic_only_no_motion",
            "frames": {
                "world_frame": self._world_frame,
                "robot_base_frame": self._base_frame,
                "tool_frame": self._tool_frame,
            },
            "current_base_pose_world": current_base_pose_world,
            "current_tool_pose_world": current_tool_pose_world,
            "current_tool_pose_base": current_tool_pose_base,
            "available_object_frames_world": object_frames_world,
            "frame_source": frame_sources,
            "hole_center_world": hole_center_world,
            "staging_pose_world": staging_pose_world,
            "axis_align_pose_world": axis_align_pose_world,
            "insertion_touch_pose_world": target_poses["insertion_touch_pose"],
            "insertion_hold_pose_world": target_poses["insertion_hold_pose"],
            "final_insertion_pose_world": target_poses["final_insertion_pose"],
            "retreat_pose_world": target_poses["retreat_pose"],
            "insertion_axis_world": insertion_axis_world,
            "orientation_mode": self._config.get("orientation_mode"),
            "tool_insertion_axis": self._config.get("tool_insertion_axis", "unknown"),
            "orientation_validated": orientation_validated,
            "orientation_block_reason": None
            if orientation_validated
            else "tool insertion axis not validated",
            "distance_tool_to_hole": self._distance_between(
                current_tool_pose_world,
                hole_center_world,
            ),
            "distance_tool_to_staging": self._distance_between(
                current_tool_pose_world,
                staging_pose_world,
            ),
            "lateral_offset_staging_to_hole_xy": self._xy_distance_between(
                staging_pose_world,
                hole_center_world,
            ),
            "lateral_offset_axis_align_to_hole_xy": self._xy_distance_between(
                axis_align_pose_world,
                hole_center_world,
            ),
            "vertical_clearance_axis_align_above_hole": self._z_offset(
                axis_align_pose_world,
                hole_center_world,
            ),
            "path_segments": {
                "current_tool_to_staging": self._path_segment(
                    current_tool_pose_world,
                    staging_pose_world,
                ),
                "staging_to_axis_align": self._path_segment(
                    staging_pose_world,
                    axis_align_pose_world,
                ),
                "axis_align_to_touch": self._path_segment(
                    axis_align_pose_world,
                    target_poses["insertion_touch_pose"],
                ),
                "touch_to_hold": self._path_segment(
                    target_poses["insertion_touch_pose"],
                    target_poses["insertion_hold_pose"],
                ),
                "hold_to_final": self._path_segment(
                    target_poses["insertion_hold_pose"],
                    target_poses["final_insertion_pose"],
                ),
                "final_to_retreat": self._path_segment(
                    target_poses["final_insertion_pose"],
                    target_poses["retreat_pose"],
                ),
            },
            "cartesian_geometry_valid": geometry_validity["cartesian_geometry_valid"],
            "geometry_validity": geometry_validity,
            "execution_gates": {
                "geometry_valid": geometry_validity["cartesian_geometry_valid"],
                "ik_available": ik_available,
                "tool_axis_orientation_validated": orientation_validated,
                "safety_guard_active": safety_guard_active,
            },
            "execution_gate_hint": {
                "geometry_valid": geometry_validity["cartesian_geometry_valid"],
                "ik_required": True,
                "tool_axis_required": True,
                "safety_guard_required": True,
                "force_guard_required": True,
                "controller_execution_allowed": False,
            },
            "motion_execution_allowed": motion_execution_allowed,
            "motion_execution_block_reason": self._motion_block_reason(
                geometry_validity["cartesian_geometry_valid"],
                ik_available,
                orientation_validated,
                safety_guard_active,
            ),
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self._publish_text(self._phase_publisher, "cartesian_diagnostics")
        self._publish_text(self._trial_status_publisher, "diagnostic_running")
        self.get_logger().info(message.data)

    def _lookup_optional_object_frames(self) -> dict[str, dict[str, Any]]:
        frames = {}
        for frame in self.OPTIONAL_OBJECT_FRAMES:
            pose = self._lookup_pose(self._world_frame, frame)
            if pose is not None:
                frames[frame] = pose
        return frames

    def _target_pose_from_tf_or_yaml(
        self,
        name: str,
    ) -> tuple[dict[str, Any] | None, str | None]:
        pose = self._lookup_pose(self._world_frame, name)
        if pose is not None:
            pose["frame_source"] = "tf"
            return pose, "tf"

        pose = self._target_pose(name)
        if pose is not None:
            pose["frame_source"] = "yaml_fallback"
            return pose, "yaml_fallback"
        return None, None

    def _lookup_pose(self, target_frame: str, source_frame: str) -> dict[str, Any] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(
                f"TF lookup unavailable for {target_frame} -> {source_frame}: {exc}"
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "frame": target_frame,
            "child_frame": source_frame,
            "position_xyz": [translation.x, translation.y, translation.z],
            "orientation_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w],
        }

    def _target_pose(self, name: str) -> dict[str, Any] | None:
        target = self._targets.get(name)
        if not isinstance(target, dict):
            return None
        position = target.get("position_xyz")
        orientation = target.get("orientation_xyzw")
        if not self._is_vector(position, 3):
            return None
        if not self._is_vector(orientation, 4):
            orientation = [0.0, 0.0, 0.0, 1.0]
        return {
            "frame": str(target.get("frame", self._world_frame)),
            "position_xyz": [float(value) for value in position],
            "orientation_xyzw": [float(value) for value in orientation],
            "orientation_placeholder": not self._is_vector(
                target.get("orientation_xyzw"),
                4,
            ),
        }

    def _insertion_axis(self) -> dict[str, Any] | None:
        raw_axis = self._config.get("insertion_axis", {})
        if isinstance(raw_axis, dict):
            direction = raw_axis.get("direction_xyz")
            frame = str(raw_axis.get("frame", self._world_frame))
        else:
            direction = self._config.get("insertion_axis_world")
            frame = self._world_frame
        if not self._is_vector(direction, 3):
            return None
        return {
            "frame": frame,
            "direction_xyz": [float(value) for value in direction],
        }

    def _tool_orientation_validated(self) -> bool:
        tool_axis = str(self._config.get("tool_insertion_axis", "unknown")).strip()
        return bool(tool_axis) and tool_axis != "unknown"

    def _geometry_validity(
        self,
        target_poses: dict[str, dict[str, Any] | None],
    ) -> dict[str, Any]:
        hole_center = target_poses.get("hole_center")
        xy_checks = {}
        for target_name in self.INSERTION_ALIGNED_TARGETS:
            offset = self._xy_distance_between(target_poses.get(target_name), hole_center)
            xy_checks[target_name] = {
                "xy_offset_m": offset,
                "within_2mm": offset is not None and offset <= self.XY_TOLERANCE_M,
            }

        z_order_valid = self._z_order_valid(
            target_poses.get("axis_align_pose"),
            target_poses.get("insertion_touch_pose"),
            target_poses.get("insertion_hold_pose"),
            target_poses.get("final_insertion_pose"),
        )
        staging = self._targets.get("staging_pose", {})
        staging_label_valid = (
            isinstance(staging, dict)
            and "staging" in str(staging.get("description", "")).lower()
        )
        geometry_valid = (
            all(check["within_2mm"] for check in xy_checks.values())
            and z_order_valid
            and staging_label_valid
        )
        return {
            "cartesian_geometry_valid": geometry_valid,
            "xy_tolerance_m": self.XY_TOLERANCE_M,
            "insertion_aligned_xy_checks": xy_checks,
            "z_order_axis_touch_hold_final_valid": z_order_valid,
            "staging_pose_laterally_offset_allowed": True,
            "staging_pose_label_valid": staging_label_valid,
        }

    @staticmethod
    def _motion_block_reason(
        geometry_valid: bool,
        ik_available: bool,
        orientation_validated: bool,
        safety_guard_active: bool,
    ) -> str | None:
        reasons = []
        if not geometry_valid:
            reasons.append("cartesian geometry invalid")
        if not ik_available:
            reasons.append("IK not available")
        if not orientation_validated:
            reasons.append("tool insertion axis not validated")
        if not safety_guard_active:
            reasons.append("safety guard not active")
        if reasons:
            return "; ".join(reasons)
        return None

    @staticmethod
    def _publish_text(publisher: Any, value: str) -> None:
        message = String()
        message.data = value
        publisher.publish(message)

    @staticmethod
    def _is_vector(value: Any, length: int) -> bool:
        return isinstance(value, list) and len(value) == length

    @staticmethod
    def _distance_between(
        tool_pose: dict[str, Any] | None,
        target_pose: dict[str, Any] | None,
    ) -> float | None:
        if tool_pose is None or target_pose is None:
            return None
        tool_position = tool_pose.get("position_xyz")
        target_position = target_pose.get("position_xyz")
        if not isinstance(tool_position, list) or not isinstance(target_position, list):
            return None
        if len(tool_position) != 3 or len(target_position) != 3:
            return None
        return math.sqrt(
            sum(
                (float(tool_position[index]) - float(target_position[index])) ** 2
                for index in range(3)
            )
        )

    @staticmethod
    def _xy_distance_between(
        first_pose: dict[str, Any] | None,
        second_pose: dict[str, Any] | None,
    ) -> float | None:
        if first_pose is None or second_pose is None:
            return None
        first_position = first_pose.get("position_xyz")
        second_position = second_pose.get("position_xyz")
        if not isinstance(first_position, list) or not isinstance(second_position, list):
            return None
        if len(first_position) != 3 or len(second_position) != 3:
            return None
        return math.sqrt(
            sum(
                (float(first_position[index]) - float(second_position[index])) ** 2
                for index in (0, 1)
            )
        )

    @staticmethod
    def _z_offset(
        first_pose: dict[str, Any] | None,
        second_pose: dict[str, Any] | None,
    ) -> float | None:
        if first_pose is None or second_pose is None:
            return None
        first_position = first_pose.get("position_xyz")
        second_position = second_pose.get("position_xyz")
        if not isinstance(first_position, list) or not isinstance(second_position, list):
            return None
        if len(first_position) != 3 or len(second_position) != 3:
            return None
        return float(first_position[2]) - float(second_position[2])

    @classmethod
    def _path_segment(
        cls,
        start_pose: dict[str, Any] | None,
        end_pose: dict[str, Any] | None,
    ) -> dict[str, Any]:
        start_position = cls._position(start_pose)
        end_position = cls._position(end_pose)
        if start_position is None or end_position is None:
            return {
                "delta_xyz": None,
                "distance": None,
                "direction_xyz": None,
            }

        delta = [
            end_position[index] - start_position[index]
            for index in range(3)
        ]
        distance = math.sqrt(sum(value**2 for value in delta))
        if distance > 0.0:
            direction = [value / distance for value in delta]
        else:
            direction = [0.0, 0.0, 0.0]
        return {
            "delta_xyz": delta,
            "distance": distance,
            "direction_xyz": direction,
        }

    @staticmethod
    def _position(pose: dict[str, Any] | None) -> list[float] | None:
        if pose is None:
            return None
        position = pose.get("position_xyz")
        if not isinstance(position, list) or len(position) != 3:
            return None
        return [float(value) for value in position]

    @classmethod
    def _z_order_valid(
        cls,
        axis_align_pose: dict[str, Any] | None,
        insertion_touch_pose: dict[str, Any] | None,
        insertion_hold_pose: dict[str, Any] | None,
        final_insertion_pose: dict[str, Any] | None,
    ) -> bool:
        positions = [
            cls._position(axis_align_pose),
            cls._position(insertion_touch_pose),
            cls._position(insertion_hold_pose),
            cls._position(final_insertion_pose),
        ]
        if any(position is None for position in positions):
            return False
        z_values = [position[2] for position in positions if position is not None]
        return z_values[0] > z_values[1] > z_values[2] > z_values[3]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CartesianInsertionDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
