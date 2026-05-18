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
        "pre_insertion_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
    )
    TARGET_POSE_FRAMES = (
        "hole_center",
        "pre_insertion_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
    )

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
        pre_insertion_pose_world = target_poses["pre_insertion_pose"]
        insertion_axis_world = self._target_direction("insertion_axis")

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
            "pre_insertion_pose_world": pre_insertion_pose_world,
            "insertion_touch_pose_world": target_poses["insertion_touch_pose"],
            "insertion_hold_pose_world": target_poses["insertion_hold_pose"],
            "final_insertion_pose_world": target_poses["final_insertion_pose"],
            "insertion_axis_world": insertion_axis_world,
            "distance_tool_to_hole": self._distance_between(
                current_tool_pose_world,
                hole_center_world,
            ),
            "distance_tool_to_pre_insertion": self._distance_between(
                current_tool_pose_world,
                pre_insertion_pose_world,
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
        if not self._is_vector(position, 3) or not self._is_vector(orientation, 4):
            return None
        return {
            "frame": str(target.get("frame", self._world_frame)),
            "position_xyz": [float(value) for value in position],
            "orientation_xyzw": [float(value) for value in orientation],
        }

    def _target_direction(self, name: str) -> dict[str, Any] | None:
        target = self._targets.get(name)
        if not isinstance(target, dict):
            return None
        direction = target.get("direction_xyz")
        if not self._is_vector(direction, 3):
            return None
        return {
            "frame": str(target.get("frame", self._world_frame)),
            "direction_xyz": [float(value) for value in direction],
        }

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
