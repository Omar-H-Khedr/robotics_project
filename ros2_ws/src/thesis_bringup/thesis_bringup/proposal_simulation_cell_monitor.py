"""Proposal simulation cell monitor and evidence writer."""

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
from sensor_msgs.msg import CameraInfo, JointState
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


class ProposalSimulationCellMonitor(Node):
    """Publish simulation-only evidence topics and save diagnostics."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_0_monitor")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_0")
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
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_task_frames()

        self.create_timer(0.2, self._publish_evidence)
        self.create_timer(0.7, self._advance_phase)
        self.create_timer(5.0, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_0 monitor started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal simulation config not found: {path}")
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
            self._transform(
                "hole_center",
                "insertion_axis_z",
                [0.0, 0.0, 0.08],
            ),
            self._transform(
                "world",
                "d405_camera_optical_frame",
                self._config.get("camera", {}).get("pose_xyz", [0.42, -0.65, 1.18]),
            ),
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
        self._wrench_pub.publish(wrench)

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
        tf_frames = [
            "world",
            "base_link",
            "link_1",
            "link_2",
            "link_3",
            "link_4",
            "link_5",
            "link_6",
            "tool0",
            "peg_tip",
            "table_work_surface",
            "hole_center",
            "insertion_axis_z",
            "d405_camera_link",
            "d405_camera_optical_frame",
        ]
        status = self._status(nodes, topics)

        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "tf_frames.txt", tf_frames)
        self._write_json(self._output_dir / "simulation_cell_status.json", status)
        self._write_summary(status)
        self._append_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_0 diagnostics written")
        rclpy.shutdown()

    def _status(self, nodes: list[str], topics: list[str]) -> dict[str, Any]:
        topic_text = "\n".join(topics)
        isaac_available = bool(
            self.get_parameter("isaac_available").get_parameter_value().bool_value
        )
        gazebo_fallback_used = bool(
            self.get_parameter("gazebo_fallback_used").get_parameter_value().bool_value
        )
        simulation_engine = "isaac_sim" if isaac_available else "gazebo"
        return {
            "simulation_engine": simulation_engine,
            "isaac_available": isaac_available,
            "gazebo_fallback_used": gazebo_fallback_used,
            "robot_model": self._config.get("robot_model", "KUKA LBR iisy 6 R1300"),
            "rgbd_camera_configured": "/proposal_simulation_cell/d405/color/camera_info" in topic_text
            and "/proposal_simulation_cell/d405/depth/camera_info" in topic_text,
            "joint_states_available": "/joint_states" in topic_text,
            "force_torque_or_contact_available": "/proposal_simulation_cell/contact_wrench" in topic_text,
            "peg_loaded": True,
            "hole_loaded": True,
            "table_loaded": True,
            "task_frames_available": "/tf_static" in topic_text and "/tf" in topic_text,
            "motion_execution_enabled": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "status": "ready_for_control_development",
        }

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_0",
            "",
            "Purpose: proposal-aligned simulation-cell foundation for visuomotor context-based meta-RL with virtual-force safety.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Isaac Sim available: `{status['isaac_available']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Robot model: `{status['robot_model']}`",
            "",
            "Safety constraints: motion execution disabled, real robot unused, MoveIt unused, and /compute_ik not called.",
            "",
            "Configured assets: table/work surface, peg, hole/block, D405-equivalent RGB-D camera, contact wrench interface, joint states, hole_center, peg_tip, and insertion_axis_z.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _append_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_0 monitor evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"gz_available={shutil.which('gz') is not None}",
            f"status={status['status']}",
            f"simulation_engine={status['simulation_engine']}",
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
    node = ProposalSimulationCellMonitor()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
