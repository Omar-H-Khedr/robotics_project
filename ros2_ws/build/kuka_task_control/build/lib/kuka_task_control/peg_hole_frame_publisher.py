"""Publish configured peg/hole object frames as TF transforms."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


class PegHoleFramePublisher(Node):
    """Publish explicit object frames for coordinate-based insertion planning."""

    STATUS_TOPIC = "/peg_hole_frame_status"
    DEFAULT_CONFIG_FILE = "peg_hole_cartesian_targets.yaml"
    TARGET_FRAMES = (
        "hole_center",
        "staging_pose",
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
        "retreat_pose",
        "insertion_axis_marker",
    )

    def __init__(self) -> None:
        super().__init__("peg_hole_frame_publisher")
        self.declare_parameter("config_path", "")
        self.declare_parameter("status_period_sec", 1.0)

        self._config = self._load_config(self._resolve_config_path())
        self._world_frame = str(self._config.get("world_frame", "world"))
        self._targets = self._config.get("targets", {})
        if not isinstance(self._targets, dict):
            raise ValueError("peg_hole_cartesian_targets.yaml field 'targets' must be a map")
        self._orientation_mode = str(self._config.get("orientation_mode", "unknown"))
        self._tool_insertion_axis = str(self._config.get("tool_insertion_axis", "unknown"))
        self._motion_execution_allowed = bool(
            self._config.get("motion_execution_allowed", False)
        )
        self._motion_execution_block_reason = str(
            self._config.get("motion_execution_block_reason", "")
        )
        self._orientation_placeholder_reason = None
        if self._tool_insertion_axis == "unknown":
            self._orientation_placeholder_reason = "tool insertion axis not validated"

        self._status_publisher = self.create_publisher(String, self.STATUS_TOPIC, 10)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._published_frames = self._publish_static_frames()
        self.create_timer(
            float(self.get_parameter("status_period_sec").value),
            self._publish_status,
        )

        self.get_logger().info(
            "Published peg/hole object TF frames: "
            + ", ".join(self._published_frames)
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
            raise ValueError(f"Peg/hole Cartesian target config must be a map: {config_path}")
        return config

    def _publish_static_frames(self) -> list[str]:
        transforms = []
        published_frames = []
        for frame_name in self.TARGET_FRAMES:
            transform = self._target_transform(frame_name)
            if transform is None:
                self.get_logger().warn(
                    f"Skipping peg/hole frame '{frame_name}': missing position"
                )
                continue
            transforms.append(transform)
            published_frames.append(frame_name)

        if transforms:
            self._static_broadcaster.sendTransform(transforms)
        return published_frames

    def _target_transform(self, frame_name: str) -> TransformStamped | None:
        if frame_name == "insertion_axis_marker":
            return self._insertion_axis_marker_transform()

        target = self._targets.get(frame_name)
        if not isinstance(target, dict):
            return None
        position = target.get("position_xyz")
        orientation = target.get("orientation_xyzw")
        if not self._is_vector(position, 3):
            return None
        if not self._is_vector(orientation, 4):
            orientation = [0.0, 0.0, 0.0, 1.0]

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(target.get("frame", self._world_frame))
        transform.child_frame_id = frame_name
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = float(orientation[0])
        transform.transform.rotation.y = float(orientation[1])
        transform.transform.rotation.z = float(orientation[2])
        transform.transform.rotation.w = float(orientation[3])
        return transform

    def _insertion_axis_marker_transform(self) -> TransformStamped | None:
        hole_center = self._targets.get("hole_center")
        if not isinstance(hole_center, dict):
            return None
        position = hole_center.get("position_xyz")
        if not self._is_vector(position, 3):
            return None

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(hole_center.get("frame", self._world_frame))
        transform.child_frame_id = "insertion_axis_marker"
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        return transform

    def _publish_status(self) -> None:
        payload = {
            "status": "object_frames_published",
            "world_frame": self._world_frame,
            "published_frames": self._published_frames,
            "target_count": len(self._published_frames),
            "orientation_mode": self._orientation_mode,
            "tool_insertion_axis": self._tool_insertion_axis,
            "orientation_placeholder_xyzw": [0.0, 0.0, 0.0, 1.0],
            "orientation_placeholder_reason": self._orientation_placeholder_reason,
            "motion_execution_allowed": self._motion_execution_allowed,
            "motion_execution_block_reason": self._motion_execution_block_reason,
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._status_publisher.publish(message)

    @staticmethod
    def _is_vector(value: Any, length: int) -> bool:
        return isinstance(value, list) and len(value) == length


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PegHoleFramePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
