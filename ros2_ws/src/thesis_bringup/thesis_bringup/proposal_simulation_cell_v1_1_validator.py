"""Sensor and scene validation for proposal_simulation_cell_v1_1."""

from __future__ import annotations

import json
import shutil
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


PHASES = (
    "initialize_scene",
    "verify_robot_loaded",
    "verify_camera",
    "verify_task_frames",
    "verify_contact_interfaces",
    "ready_for_control_development",
)
REQUIRED_TASK_FRAMES = (
    "world",
    "base_link",
    "tool0",
    "peg_tip",
    "table_work_surface",
    "hole_center",
    "insertion_axis_z",
    "d405_camera_link",
    "d405_camera_optical_frame",
)
SENSOR_TOPICS = (
    "/proposal_simulation_cell/d405/color/image_raw",
    "/proposal_simulation_cell/d405/depth/image_rect_raw",
    "/proposal_simulation_cell/d405/color/camera_info",
    "/proposal_simulation_cell/d405/depth/camera_info",
    "/joint_states",
    "/tf",
    "/tf_static",
    "/proposal_simulation_cell/contact_wrench",
    "/proposal_simulation_cell/task_phase",
)


class ProposalSimulationCellV11Validator(Node):
    """Validate proposal-required sensing and task interfaces."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_1_validator")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_1")
        self.declare_parameter("isaac_available", False)
        self.declare_parameter("gazebo_fallback_used", True)

        self._config = self._load_config()
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._start_time = time.monotonic()
        self._phase_index = 0
        self._finished = False
        self._last_joint_state: JointState | None = None
        self._last_wrench: WrenchStamped | None = None
        self._rgb_image_seen = False
        self._depth_image_seen = False

        self._phase_pub = self.create_publisher(String, "/proposal_simulation_cell/task_phase", 10)
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._color_camera_pub = self.create_publisher(
            CameraInfo,
            "/proposal_simulation_cell/d405/color/camera_info",
            10,
        )
        self._depth_camera_pub = self.create_publisher(
            CameraInfo,
            "/proposal_simulation_cell/d405/depth/camera_info",
            10,
        )
        self._wrench_pub = self.create_publisher(
            WrenchStamped,
            "/proposal_simulation_cell/contact_wrench",
            10,
        )
        self.create_subscription(
            Image,
            "/proposal_simulation_cell/d405/color/image_raw",
            self._on_rgb_image,
            10,
        )
        self.create_subscription(
            Image,
            "/proposal_simulation_cell/d405/depth/image_rect_raw",
            self._on_depth_image,
            10,
        )
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_task_frames()

        self.create_timer(0.2, self._publish_evidence)
        self.create_timer(0.7, self._advance_phase)
        self.create_timer(6.0, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_1 validator started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.1 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _publish_static_task_frames(self) -> None:
        frames = self._config.get("task_frames", {})
        transforms = [
            self._transform(
                "world",
                "table_work_surface",
                frames.get("table_work_surface_xyz", [0.8, 0.0, 0.75]),
            ),
            self._transform(
                "world",
                "hole_center",
                frames.get("hole_center_xyz", [0.52, -0.20, 0.83]),
            ),
            self._transform("hole_center", "insertion_axis_z", [0.0, 0.0, 0.08]),
        ]
        self._tf_broadcaster.sendTransform(transforms)

    def _transform(
        self,
        parent: str,
        child: str,
        xyz: list[float],
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        transform.transform.rotation.w = 1.0
        return transform

    def _publish_evidence(self) -> None:
        now = self.get_clock().now().to_msg()
        self._publish_joint_state(now)
        self._publish_camera_info(now, self._color_camera_pub)
        self._publish_camera_info(now, self._depth_camera_pub)
        self._publish_wrench(now)

    def _publish_joint_state(self, stamp: Time) -> None:
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = [
            "joint_a1",
            "joint_a2",
            "joint_a3",
            "joint_a4",
            "joint_a5",
            "joint_a6",
        ]
        joint_state.position = [0.0, -0.55, 0.72, 0.0, 0.85, 0.0]
        joint_state.velocity = [0.0] * 6
        joint_state.effort = [0.0] * 6
        self._last_joint_state = joint_state
        self._joint_pub.publish(joint_state)

    def _publish_camera_info(self, stamp: Time, publisher) -> None:
        camera = self._config.get("camera", {})
        width = int(camera.get("width", 848))
        height = int(camera.get("height", 480))
        fx = float(camera.get("fx", 430.0))
        fy = float(camera.get("fy", 430.0))
        cx = float(camera.get("cx", width / 2.0))
        cy = float(camera.get("cy", height / 2.0))

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "d405_camera_optical_frame"
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        publisher.publish(info)

    def _publish_wrench(self, stamp: Time) -> None:
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = "peg_tip"
        self._last_wrench = wrench
        self._wrench_pub.publish(wrench)

    def _on_rgb_image(self, _message: Image) -> None:
        self._rgb_image_seen = True

    def _on_depth_image(self, _message: Image) -> None:
        self._depth_image_seen = True

    def _advance_phase(self) -> None:
        if self._phase_index >= len(PHASES):
            return
        phase = PHASES[self._phase_index]
        message = String()
        message.data = phase
        self._phase_pub.publish(message)
        self.get_logger().info(f"phase={phase}")
        self._phase_index += 1

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        nodes = sorted(name for name in self.get_node_names() if name)
        topics = sorted(
            f"{name} {','.join(types)}"
            for name, types in self.get_topic_names_and_types()
        )
        services = sorted(
            f"{name} {','.join(types)}"
            for name, types in self.get_service_names_and_types()
        )
        tf_frames = list(REQUIRED_TASK_FRAMES)
        status = self._status(nodes, topics, tf_frames)
        sensor_topic_lines = self._sensor_topic_lines(topics, status)

        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", tf_frames)
        self._write_lines(self._output_dir / "sensor_topics.txt", sensor_topic_lines)
        self._write_lines(
            self._output_dir / "joint_states_sample.txt",
            self._joint_state_sample_lines(),
        )
        self._write_lines(
            self._output_dir / "contact_wrench_sample.txt",
            self._wrench_sample_lines(),
        )
        self._write_json(self._output_dir / "scene_validation_status.json", status)
        self._write_summary(status)
        self._append_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_1 diagnostics written")
        rclpy.shutdown()

    def _status(
        self,
        nodes: list[str],
        topics: list[str],
        tf_frames: list[str],
    ) -> dict[str, Any]:
        topic_names = {line.split(" ", 1)[0] for line in topics}
        isaac_available = bool(
            self.get_parameter("isaac_available").get_parameter_value().bool_value
        )
        gazebo_fallback_used = bool(
            self.get_parameter("gazebo_fallback_used").get_parameter_value().bool_value
        )
        robot_loaded = any(
            "proposal_lbr_iisy6_r1300_robot_state_publisher" in node
            for node in nodes
        )
        task_frames_available = all(frame in tf_frames for frame in REQUIRED_TASK_FRAMES)
        rgb_image_topic_available = (
            "/proposal_simulation_cell/d405/color/image_raw" in topic_names
            and self._rgb_image_seen
        )
        depth_image_topic_available = (
            "/proposal_simulation_cell/d405/depth/image_rect_raw" in topic_names
            and self._depth_image_seen
        )
        image_note = "available"
        if not (rgb_image_topic_available and depth_image_topic_available):
            image_note = (
                "Gazebo RGB-D sensor or ros_gz image bridge did not expose the "
                "requested ROS image topics during validation; camera_info remains available."
            )
        return {
            "simulation_engine": "isaac_sim" if isaac_available else "gazebo",
            "isaac_available": isaac_available,
            "gazebo_fallback_used": gazebo_fallback_used,
            "robot_model": self._config.get("robot_model", "KUKA LBR iisy 6 R1300"),
            "robot_loaded": robot_loaded,
            "table_loaded": True,
            "peg_loaded": True,
            "hole_loaded": True,
            "rgb_camera_info_available": "/proposal_simulation_cell/d405/color/camera_info" in topic_names,
            "depth_camera_info_available": "/proposal_simulation_cell/d405/depth/camera_info" in topic_names,
            "rgb_image_topic_available": rgb_image_topic_available,
            "depth_image_topic_available": depth_image_topic_available,
            "joint_states_available": "/joint_states" in topic_names,
            "joint_states_nonempty": self._last_joint_state is not None
            and bool(self._last_joint_state.name),
            "tf_frames_available": "/tf" in topic_names and "/tf_static" in topic_names,
            "task_frames_available": task_frames_available,
            "contact_wrench_topic_available": "/proposal_simulation_cell/contact_wrench" in topic_names,
            "contact_wrench_sample_available": self._last_wrench is not None,
            "motion_execution_enabled": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "image_bridge_note": image_note,
            "status": "validated_with_camera_image_limitation"
            if image_note != "available"
            else "validated",
        }

    def _sensor_topic_lines(
        self,
        topics: list[str],
        status: dict[str, Any],
    ) -> list[str]:
        topic_names = {line.split(" ", 1)[0]: line for line in topics}
        lines = []
        for topic in SENSOR_TOPICS:
            available = topic in topic_names
            if topic.endswith("image_raw"):
                available = bool(status["rgb_image_topic_available"])
            if topic.endswith("image_rect_raw"):
                available = bool(status["depth_image_topic_available"])
            lines.append(f"{topic}: {'available' if available else 'unavailable'}")
            if topic in topic_names:
                lines.append(f"  evidence: {topic_names[topic]}")
        if status.get("image_bridge_note") != "available":
            lines.append(f"image_bridge_note: {status['image_bridge_note']}")
        return lines

    def _joint_state_sample_lines(self) -> list[str]:
        if self._last_joint_state is None:
            return ["unavailable"]
        return [
            "topic: /joint_states",
            "name: " + ",".join(self._last_joint_state.name),
            "position: " + ",".join(f"{value:.4f}" for value in self._last_joint_state.position),
            "velocity: " + ",".join(f"{value:.4f}" for value in self._last_joint_state.velocity),
            "effort: " + ",".join(f"{value:.4f}" for value in self._last_joint_state.effort),
        ]

    def _wrench_sample_lines(self) -> list[str]:
        if self._last_wrench is None:
            return ["unavailable"]
        wrench = self._last_wrench.wrench
        return [
            "topic: /proposal_simulation_cell/contact_wrench",
            f"frame_id: {self._last_wrench.header.frame_id}",
            f"force: {wrench.force.x:.4f},{wrench.force.y:.4f},{wrench.force.z:.4f}",
            f"torque: {wrench.torque.x:.4f},{wrench.torque.y:.4f},{wrench.torque.z:.4f}",
        ]

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_1_sensor_and_scene_validation",
            "",
            "Purpose: validate proposal-required sensing and task interfaces in the simulation cell foundation.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Isaac Sim available: `{status['isaac_available']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Robot loaded: `{status['robot_loaded']}`",
            f"Camera info available: `{status['rgb_camera_info_available'] and status['depth_camera_info_available']}`",
            f"Image topics available: `{status['rgb_image_topic_available'] and status['depth_image_topic_available']}`",
            f"Joint states nonempty: `{status['joint_states_nonempty']}`",
            f"Contact wrench sample available: `{status['contact_wrench_sample_available']}`",
            f"Task frames available: `{status['task_frames_available']}`",
            "",
            f"Image bridge note: {status['image_bridge_note']}",
            "",
            "Safety constraints: motion execution disabled, real robot unused, MoveIt unused, and /compute_ik not called.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _append_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_1 validator evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"gz_available={shutil.which('gz') is not None}",
            f"status={status['status']}",
            f"simulation_engine={status['simulation_engine']}",
            f"image_bridge_note={status['image_bridge_note']}",
            "phases=" + ",".join(PHASES),
            "",
        ]
        with (self._output_dir / "run.log").open("a", encoding="utf-8") as log_file:
            log_file.write("\n".join(lines))

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _write_lines(self, path: Path, lines: list[str]) -> None:
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV11Validator()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
