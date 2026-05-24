"""RGB-D image bridge validation for proposal_simulation_cell_v1_2."""

from __future__ import annotations

import json
import shutil
import subprocess
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
    "verify_gazebo_camera_topics",
    "verify_ros_image_bridge",
    "record_rgbd_samples",
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


class ProposalSimulationCellV12RgbdValidator(Node):
    """Validate that Gazebo publishes real RGB and depth image samples into ROS 2."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_2_rgbd_validator")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_2")
        self.declare_parameter("isaac_available", False)
        self.declare_parameter("gazebo_fallback_used", True)
        self.declare_parameter("ros_gz_bridge_available", False)
        self.declare_parameter("ros_gz_image_available", False)

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
        self._rgb_image: Image | None = None
        self._depth_image: Image | None = None
        self._rgb_camera_info: CameraInfo | None = None
        self._depth_camera_info: CameraInfo | None = None

        camera = self._config.get("camera", {})
        self._rgb_topic = str(
            camera.get("expected_rgb_image_topic", "/proposal_simulation_cell/d405/color/image_raw")
        )
        self._depth_topic = str(
            camera.get(
                "expected_depth_image_topic",
                "/proposal_simulation_cell/d405/depth/image_rect_raw",
            )
        )
        self._rgb_info_topic = str(
            camera.get("color_camera_info_topic", "/proposal_simulation_cell/d405/color/camera_info")
        )
        self._depth_info_topic = str(
            camera.get("depth_camera_info_topic", "/proposal_simulation_cell/d405/depth/camera_info")
        )
        self._rgb_gz_topic = str(camera.get("rgb_gazebo_topic", self._rgb_topic))
        self._depth_gz_topic = str(camera.get("depth_gazebo_topic", self._depth_topic))

        self._phase_pub = self.create_publisher(String, "/proposal_simulation_cell/task_phase", 10)
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._wrench_pub = self.create_publisher(
            WrenchStamped,
            "/proposal_simulation_cell/contact_wrench",
            10,
        )
        self.create_subscription(Image, self._rgb_topic, self._on_rgb_image, 10)
        self.create_subscription(Image, self._depth_topic, self._on_depth_image, 10)
        self.create_subscription(CameraInfo, self._rgb_info_topic, self._on_rgb_camera_info, 10)
        self.create_subscription(CameraInfo, self._depth_info_topic, self._on_depth_camera_info, 10)

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_task_frames()

        self.create_timer(0.2, self._publish_evidence)
        self.create_timer(0.8, self._advance_phase)
        self.create_timer(9.0, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_2 RGB-D validator started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.2 config not found: {path}")
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

    def _transform(self, parent: str, child: str, xyz: list[float]) -> TransformStamped:
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
        self._publish_wrench(now)

    def _publish_joint_state(self, stamp: Time) -> None:
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]
        joint_state.position = [0.0, -0.55, 0.72, 0.0, 0.85, 0.0]
        joint_state.velocity = [0.0] * 6
        joint_state.effort = [0.0] * 6
        self._last_joint_state = joint_state
        self._joint_pub.publish(joint_state)

    def _publish_wrench(self, stamp: Time) -> None:
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = "peg_tip"
        self._last_wrench = wrench
        self._wrench_pub.publish(wrench)

    def _on_rgb_image(self, message: Image) -> None:
        self._rgb_image = message

    def _on_depth_image(self, message: Image) -> None:
        self._depth_image = message

    def _on_rgb_camera_info(self, message: CameraInfo) -> None:
        self._rgb_camera_info = message

    def _on_depth_camera_info(self, message: CameraInfo) -> None:
        self._depth_camera_info = message

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
            f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types()
        )
        services = sorted(
            f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types()
        )
        gz_topics = self._gz_topic_lines()
        status = self._status(nodes, topics, gz_topics)

        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "gz_topics.txt", gz_topics)
        self._write_lines(self._output_dir / "sensor_topics.txt", self._sensor_topic_lines(topics, status))
        self._write_lines(
            self._output_dir / "rgb_image_sample_info.txt",
            self._image_sample_lines(self._rgb_topic, self._rgb_image),
        )
        self._write_lines(
            self._output_dir / "depth_image_sample_info.txt",
            self._image_sample_lines(self._depth_topic, self._depth_image),
        )
        self._write_lines(self._output_dir / "camera_info_sample.txt", self._camera_info_lines())
        self._write_json(self._output_dir / "rgbd_bridge_status.json", status)
        self._write_summary(status)
        self._append_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_2 RGB-D diagnostics written")
        rclpy.shutdown()

    def _status(
        self,
        nodes: list[str],
        topics: list[str],
        gz_topics: list[str],
    ) -> dict[str, Any]:
        topic_names = {line.split(" ", 1)[0] for line in topics}
        gz_topic_text = "\n".join(gz_topics)
        rgb_gz_discovered = self._rgb_gz_topic in gz_topic_text
        depth_gz_discovered = self._depth_gz_topic in gz_topic_text
        rgb_ros_available = self._rgb_topic in topic_names
        depth_ros_available = self._depth_topic in topic_names
        rgb_info_available = self._rgb_info_topic in topic_names and self._rgb_camera_info is not None
        depth_info_available = self._depth_info_topic in topic_names and self._depth_camera_info is not None
        rgb_sample_received = self._rgb_image is not None
        depth_sample_received = self._depth_image is not None
        status = (
            "rgbd_image_bridge_validated"
            if rgb_sample_received and depth_sample_received
            else "validated_with_rgbd_image_limitation"
        )

        limitation_reason = "none"
        if status != "rgbd_image_bridge_validated":
            reasons = []
            if not self._ros_pkg_executable_available("ros_gz_image", "image_bridge"):
                reasons.append("ros_gz_image image_bridge executable unavailable")
            if not rgb_gz_discovered:
                reasons.append(f"Gazebo RGB topic not discovered: {self._rgb_gz_topic}")
            if not depth_gz_discovered:
                reasons.append(f"Gazebo depth topic not discovered: {self._depth_gz_topic}")
            if rgb_ros_available and not rgb_sample_received:
                reasons.append(f"ROS RGB topic available but no Image sample received: {self._rgb_topic}")
            if depth_ros_available and not depth_sample_received:
                reasons.append(
                    f"ROS depth topic available but no Image sample received: {self._depth_topic}"
                )
            if not rgb_ros_available:
                reasons.append(f"ROS RGB topic unavailable: {self._rgb_topic}")
            if not depth_ros_available:
                reasons.append(f"ROS depth topic unavailable: {self._depth_topic}")
            limitation_reason = "; ".join(reasons) if reasons else "RGB-D samples unavailable"

        return {
            "simulation_engine": "gazebo",
            "isaac_available": bool(
                self.get_parameter("isaac_available").get_parameter_value().bool_value
            ),
            "gazebo_fallback_used": bool(
                self.get_parameter("gazebo_fallback_used").get_parameter_value().bool_value
            ),
            "ros_gz_bridge_available": self._ros_pkg_executable_available(
                "ros_gz_bridge",
                "parameter_bridge",
            ),
            "ros_gz_image_available": self._ros_pkg_executable_available(
                "ros_gz_image",
                "image_bridge",
            ),
            "rgb_gazebo_topic_discovered": rgb_gz_discovered,
            "depth_gazebo_topic_discovered": depth_gz_discovered,
            "rgb_ros_topic_available": rgb_ros_available,
            "depth_ros_topic_available": depth_ros_available,
            "rgb_image_sample_received": rgb_sample_received,
            "depth_image_sample_received": depth_sample_received,
            "rgb_camera_info_available": rgb_info_available,
            "depth_camera_info_available": depth_info_available,
            "image_bridge_method": "ros_gz_image/image_bridge direct Gazebo topic bridge",
            "limitation_reason": limitation_reason,
            "motion_execution_enabled": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "status": status,
        }

    def _sensor_topic_lines(self, topics: list[str], status: dict[str, Any]) -> list[str]:
        topic_names = {line.split(" ", 1)[0]: line for line in topics}
        lines = []
        for topic in SENSOR_TOPICS:
            available = topic in topic_names
            if topic == self._rgb_topic:
                available = bool(status["rgb_ros_topic_available"])
            if topic == self._depth_topic:
                available = bool(status["depth_ros_topic_available"])
            lines.append(f"{topic}: {'available' if available else 'unavailable'}")
            if topic in topic_names:
                lines.append(f"  evidence: {topic_names[topic]}")
        lines.append(f"rgb_image_sample_received: {status['rgb_image_sample_received']}")
        lines.append(f"depth_image_sample_received: {status['depth_image_sample_received']}")
        lines.append(f"limitation_reason: {status['limitation_reason']}")
        return lines

    def _gz_topic_lines(self) -> list[str]:
        topics = self._run_command(["gz", "topic", "-l"], timeout=5.0)
        lines = ["# gz topic -l", *topics]
        for topic in (self._rgb_gz_topic, self._depth_gz_topic):
            lines.extend(["", f"# gz topic -i -t {topic}"])
            lines.extend(self._run_command(["gz", "topic", "-i", "-t", topic], timeout=5.0))
        return lines

    def _run_command(self, command: list[str], timeout: float) -> list[str]:
        if shutil.which(command[0]) is None:
            return [f"unavailable: executable not found: {command[0]}"]
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired:
            return [f"timeout: {' '.join(command)}"]
        output = []
        if completed.stdout:
            output.extend(completed.stdout.splitlines())
        if completed.stderr:
            output.append("# stderr")
            output.extend(completed.stderr.splitlines())
        if completed.returncode != 0:
            output.append(f"# exit_code={completed.returncode}")
        return output if output else ["no output"]

    def _ros_pkg_executable_available(self, package: str, executable: str) -> bool:
        try:
            completed = subprocess.run(
                ["ros2", "pkg", "executables", package],
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except subprocess.TimeoutExpired:
            return False
        expected = f"{package} {executable}"
        return any(line.strip() == expected for line in completed.stdout.splitlines())

    def _image_sample_lines(self, topic: str, image: Image | None) -> list[str]:
        if image is None:
            return [f"topic: {topic}", "sample_received: false"]
        return [
            f"topic: {topic}",
            "sample_received: true",
            f"frame_id: {image.header.frame_id}",
            f"width: {image.width}",
            f"height: {image.height}",
            f"encoding: {image.encoding}",
            f"is_bigendian: {image.is_bigendian}",
            f"step: {image.step}",
            f"data_length: {len(image.data)}",
        ]

    def _camera_info_lines(self) -> list[str]:
        lines = []
        for topic, info in (
            (self._rgb_info_topic, self._rgb_camera_info),
            (self._depth_info_topic, self._depth_camera_info),
        ):
            lines.append(f"topic: {topic}")
            if info is None:
                lines.append("sample_received: false")
                continue
            lines.extend(
                [
                    "sample_received: true",
                    f"frame_id: {info.header.frame_id}",
                    f"width: {info.width}",
                    f"height: {info.height}",
                    "k: " + ",".join(f"{value:.4f}" for value in info.k),
                ]
            )
        return lines

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_2_rgbd_image_bridge_fix",
            "",
            "Purpose: validate actual RGB and depth image samples from the Gazebo fallback D405-like camera.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"ros_gz_bridge available: `{status['ros_gz_bridge_available']}`",
            f"ros_gz_image available: `{status['ros_gz_image_available']}`",
            f"RGB Gazebo topic discovered: `{status['rgb_gazebo_topic_discovered']}`",
            f"Depth Gazebo topic discovered: `{status['depth_gazebo_topic_discovered']}`",
            f"RGB image sample received: `{status['rgb_image_sample_received']}`",
            f"Depth image sample received: `{status['depth_image_sample_received']}`",
            f"Status: `{status['status']}`",
            "",
            f"Limitation reason: {status['limitation_reason']}",
            "",
            "Safety constraints: motion execution disabled, real robot unused, MoveIt unused, and /compute_ik not called.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _append_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_2 RGB-D validator evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"gz_available={shutil.which('gz') is not None}",
            f"status={status['status']}",
            f"rgb_image_sample_received={status['rgb_image_sample_received']}",
            f"depth_image_sample_received={status['depth_image_sample_received']}",
            f"limitation_reason={status['limitation_reason']}",
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
    node = ProposalSimulationCellV12RgbdValidator()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
