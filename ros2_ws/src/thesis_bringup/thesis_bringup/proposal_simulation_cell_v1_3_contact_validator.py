"""Contact physics validation for proposal_simulation_cell_v1_3."""

from __future__ import annotations

import csv
import json
import math
import re
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
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
    "verify_collision_bodies",
    "verify_contact_topics",
    "record_contact_wrench_samples",
    "verify_rgbd_samples",
    "write_contact_diagnostics",
)
REQUIRED_TASK_FRAMES = (
    "world",
    "base_link",
    "tool0",
    "peg_tip",
    "hole_center",
    "insertion_axis_z",
    "contact_frame",
)
REQUIRED_TOPICS = (
    "/joint_states",
    "/tf",
    "/tf_static",
    "/proposal_simulation_cell/d405/color/image_raw",
    "/proposal_simulation_cell/d405/depth/image_rect_raw",
    "/proposal_simulation_cell/contact_wrench",
    "/proposal_simulation_cell/task_phase",
    "/proposal_simulation_cell/contact_state",
)


class ProposalSimulationCellV13ContactValidator(Node):
    """Validate contact-rich simulation foundations without executing robot motion."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_3_contact_validator")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_3")
        self.declare_parameter("isaac_available", False)
        self.declare_parameter("gazebo_fallback_used", True)
        self.declare_parameter("ros_gz_bridge_available", False)
        self.declare_parameter("ros_gz_image_available", False)
        self.declare_parameter("world_path", "")

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
        self._last_contact_state = "initializing"
        self._rgb_image: Image | None = None
        self._depth_image: Image | None = None
        self._rgb_camera_info: CameraInfo | None = None
        self._depth_camera_info: CameraInfo | None = None
        self._wrench_rows: list[dict[str, float | str]] = []
        self._gz_contact_output: list[str] = []
        self._gz_contact_topics_seen: list[str] = []
        self._contact_poll_attempted = False
        self._max_force = 0.0
        self._max_torque = 0.0
        self._latest_observed_force = (0.0, 0.0, 0.0)
        self._latest_observed_torque = (0.0, 0.0, 0.0)

        camera = self._config.get("camera", {})
        contact = self._config.get("contact_interface", {})
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
        self._wrench_topic = str(
            contact.get("wrench_topic", "/proposal_simulation_cell/contact_wrench")
        )
        self._state_topic = str(contact.get("state_topic", "/proposal_simulation_cell/contact_state"))
        self._contact_frame = str(contact.get("frame_id", "contact_frame"))
        self._gz_contact_topics = [
            str(topic) for topic in contact.get("gazebo_contact_topics", []) if str(topic)
        ]
        safety = self._config.get("safety", {})
        self._force_threshold = float(safety.get("contact_detection_force_threshold_n", 0.1))
        self._max_allowed_force = float(safety.get("max_allowed_force_n", 50.0))
        self._max_allowed_torque = float(safety.get("max_allowed_torque_nm", 5.0))
        self._contact_timeout = float(safety.get("contact_timeout_sec", 8.0))
        self._expected_axis = str(safety.get("expected_contact_axis", "z"))

        self._phase_pub = self.create_publisher(String, "/proposal_simulation_cell/task_phase", 10)
        self._contact_state_pub = self.create_publisher(String, self._state_topic, 10)
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._wrench_pub = self.create_publisher(WrenchStamped, self._wrench_topic, 10)
        self.create_subscription(Image, self._rgb_topic, self._on_rgb_image, 10)
        self.create_subscription(Image, self._depth_topic, self._on_depth_image, 10)
        self.create_subscription(CameraInfo, self._rgb_info_topic, self._on_rgb_camera_info, 10)
        self.create_subscription(CameraInfo, self._depth_info_topic, self._on_depth_camera_info, 10)

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_task_frames()

        self.create_timer(0.2, self._publish_evidence)
        self.create_timer(0.8, self._advance_phase)
        self.create_timer(1.0, self._poll_gz_contacts)
        self.create_timer(max(self._contact_timeout, 8.0) + 2.0, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_3 contact validator started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.3 config not found: {path}")
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
            self._transform("world", "hole_center", frames.get("hole_center_xyz", [0.52, -0.20, 0.83])),
            self._transform("hole_center", "insertion_axis_z", [0.0, 0.0, 0.08]),
            self._transform(
                "world",
                "contact_frame",
                frames.get("contact_frame_xyz", [0.52, -0.20, 0.84]),
            ),
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
        state = String()
        state.data = self._last_contact_state
        self._contact_state_pub.publish(state)

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
        wrench.header.frame_id = self._contact_frame
        wrench.wrench.force.x = self._latest_observed_force[0]
        wrench.wrench.force.y = self._latest_observed_force[1]
        wrench.wrench.force.z = self._latest_observed_force[2]
        wrench.wrench.torque.x = self._latest_observed_torque[0]
        wrench.wrench.torque.y = self._latest_observed_torque[1]
        wrench.wrench.torque.z = self._latest_observed_torque[2]
        self._last_wrench = wrench
        self._wrench_pub.publish(wrench)
        elapsed = time.monotonic() - self._start_time
        self._wrench_rows.append(
            {
                "elapsed_sec": f"{elapsed:.3f}",
                "frame_id": wrench.header.frame_id,
                "force_x_n": f"{wrench.wrench.force.x:.6f}",
                "force_y_n": f"{wrench.wrench.force.y:.6f}",
                "force_z_n": f"{wrench.wrench.force.z:.6f}",
                "torque_x_nm": f"{wrench.wrench.torque.x:.6f}",
                "torque_y_nm": f"{wrench.wrench.torque.y:.6f}",
                "torque_z_nm": f"{wrench.wrench.torque.z:.6f}",
                "force_magnitude_n": f"{self._force_magnitude(wrench):.6f}",
                "torque_magnitude_nm": f"{self._torque_magnitude(wrench):.6f}",
            }
        )

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

    def _poll_gz_contacts(self) -> None:
        if (
            self._finished
            or self._contact_poll_attempted
            or time.monotonic() - self._start_time < 5.0
            or time.monotonic() - self._start_time > self._contact_timeout
        ):
            return
        self._contact_poll_attempted = True
        output, seen_topics = self._read_gz_contacts(timeout=1.5)
        self._gz_contact_topics_seen = sorted(set(self._gz_contact_topics_seen + seen_topics))
        if output:
            self._gz_contact_output = output
        force, torque = self._parse_contact_wrench(output)
        force_mag = math.sqrt(sum(value * value for value in force))
        torque_mag = math.sqrt(sum(value * value for value in torque))
        if force_mag > 1.0e-9 or torque_mag > 1.0e-9:
            self._latest_observed_force = force
            self._latest_observed_torque = torque
            self._max_force = max(self._max_force, force_mag)
            self._max_torque = max(self._max_torque, torque_mag)
            axis_value = {"x": force[0], "y": force[1], "z": force[2]}.get(self._expected_axis, 0.0)
            threshold_state = "above_threshold" if force_mag >= self._force_threshold else "below_threshold"
            self._last_contact_state = (
                f"contact_observed_{threshold_state} axis={self._expected_axis} "
                f"axis_force_n={axis_value:.6f} force_magnitude_n={force_mag:.6f} "
                f"torque_magnitude_nm={torque_mag:.6f}"
            )
        elif seen_topics:
            self._last_contact_state = "contact_reporting_available_no_nonzero_wrench"
        else:
            self._last_contact_state = "waiting_for_gazebo_contact_reporting"

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        output, seen_topics = self._read_gz_contacts(timeout=5.0)
        self._gz_contact_topics_seen = sorted(set(self._gz_contact_topics_seen + seen_topics))
        if output:
            self._gz_contact_output = output
        force, torque = self._parse_contact_wrench(self._gz_contact_output)
        if math.sqrt(sum(value * value for value in force)) > 1.0e-9:
            self._latest_observed_force = force
            self._latest_observed_torque = torque
            force_mag = math.sqrt(sum(value * value for value in force))
            torque_mag = math.sqrt(sum(value * value for value in torque))
            axis_value = {"x": force[0], "y": force[1], "z": force[2]}.get(self._expected_axis, 0.0)
            threshold_state = "above_threshold" if force_mag >= self._force_threshold else "below_threshold"
            self._last_contact_state = (
                f"contact_observed_{threshold_state} axis={self._expected_axis} "
                f"axis_force_n={axis_value:.6f} force_magnitude_n={force_mag:.6f} "
                f"torque_magnitude_nm={torque_mag:.6f}"
            )
            self._publish_wrench(self.get_clock().now().to_msg())

        nodes = sorted(name for name in self.get_node_names() if name)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(
            f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types()
        )
        tf_frames = list(REQUIRED_TASK_FRAMES)
        collision_status = self._collision_status()
        status = self._status(nodes, topics, tf_frames, collision_status)

        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", tf_frames)
        self._write_wrench_csv()
        self._write_lines(self._output_dir / "contact_wrench_sample.txt", self._wrench_sample_lines())
        self._write_lines(self._output_dir / "contact_state_sample.txt", [self._last_contact_state])
        self._write_json(self._output_dir / "contact_physics_status.json", status)
        self._write_summary(status)
        self._append_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_3 contact diagnostics written")
        rclpy.shutdown()

    def _status(
        self,
        nodes: list[str],
        topics: list[str],
        tf_frames: list[str],
        collision_status: dict[str, bool],
    ) -> dict[str, Any]:
        topic_names = {line.split(" ", 1)[0] for line in topics}
        force = self._force_magnitude(self._last_wrench)
        torque = self._torque_magnitude(self._last_wrench)
        self._max_force = max(self._max_force, force)
        self._max_torque = max(self._max_torque, torque)
        contact_wrench_sample_available = self._last_wrench is not None
        nonzero_contact_wrench_observed = self._max_force > 1.0e-9 or self._max_torque > 1.0e-9
        contact_reporting_available = bool(self._gz_contact_topics_seen)
        rgb_image_sample_received = self._rgb_image is not None
        depth_image_sample_received = self._depth_image is not None
        task_frames_available = all(frame in tf_frames for frame in REQUIRED_TASK_FRAMES)
        safety_violation_count = int(
            self._max_force > self._max_allowed_force or self._max_torque > self._max_allowed_torque
        )

        validation_ready = (
            collision_status["peg_collision_configured"]
            and collision_status["hole_collision_configured"]
            and collision_status["table_collision_configured"]
            and self._wrench_topic in topic_names
            and contact_wrench_sample_available
            and rgb_image_sample_received
            and depth_image_sample_received
            and "/joint_states" in topic_names
            and task_frames_available
        )
        if validation_ready and nonzero_contact_wrench_observed:
            status = "validated"
            limitation_reason = "none"
        elif validation_ready:
            status = "validated_with_contact_reporting_limitation"
            if contact_reporting_available:
                limitation_reason = (
                    "Gazebo contact topics were discovered, but no non-zero contact wrench "
                    "sample was available through the current contact sensor output during "
                    "the validation window."
                )
            else:
                limitation_reason = (
                    "Gazebo contact sensor topics were configured in SDF, but contact reporting "
                    "did not expose a readable contact message during the validation window."
                )
        else:
            status = "failed"
            limitation_reason = "required simulation diagnostics were incomplete"

        return {
            "simulation_engine": "gazebo",
            "isaac_available": bool(
                self.get_parameter("isaac_available").get_parameter_value().bool_value
            ),
            "gazebo_fallback_used": bool(
                self.get_parameter("gazebo_fallback_used").get_parameter_value().bool_value
            ),
            "robot_model": self._config.get("robot_model", "KUKA LBR iisy 6 R1300"),
            "peg_collision_configured": collision_status["peg_collision_configured"],
            "hole_collision_configured": collision_status["hole_collision_configured"],
            "table_collision_configured": collision_status["table_collision_configured"],
            "contact_wrench_topic_available": self._wrench_topic in topic_names,
            "contact_wrench_sample_available": contact_wrench_sample_available,
            "nonzero_contact_wrench_observed": nonzero_contact_wrench_observed,
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "contact_detection_threshold_n": self._force_threshold,
            "safety_violation_count": safety_violation_count,
            "rgb_image_sample_received": rgb_image_sample_received,
            "depth_image_sample_received": depth_image_sample_received,
            "joint_states_available": "/joint_states" in topic_names and self._last_joint_state is not None,
            "task_frames_available": task_frames_available,
            "motion_execution_enabled": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "expected_contact_axis": self._expected_axis,
            "max_allowed_force_n": self._max_allowed_force,
            "max_allowed_torque_nm": self._max_allowed_torque,
            "contact_state_topic_available": self._state_topic in topic_names,
            "gazebo_contact_topics_configured": self._gz_contact_topics,
            "gazebo_contact_topics_seen": self._gz_contact_topics_seen,
            "limitation_reason": limitation_reason,
            "status": status,
        }

    def _collision_status(self) -> dict[str, bool]:
        world_path = Path(self.get_parameter("world_path").get_parameter_value().string_value)
        status = {
            "peg_collision_configured": False,
            "hole_collision_configured": False,
            "table_collision_configured": False,
        }
        if not world_path.is_file():
            return status
        root = ET.parse(world_path).getroot()
        collision_names = {
            collision.get("name", "")
            for collision in root.findall(".//collision")
            if collision.get("name")
        }
        status["peg_collision_configured"] = "peg_collision" in collision_names
        status["hole_collision_configured"] = "hole_collision" in collision_names
        status["table_collision_configured"] = "table_collision" in collision_names
        return status

    def _read_gz_contacts(self, timeout: float) -> tuple[list[str], list[str]]:
        topics = self._run_command(["gz", "topic", "-l"], timeout=timeout)
        topic_names = [line.strip() for line in topics if line.strip().startswith("/")]
        discovered_contact_topics = [
            topic for topic in topic_names if "/sensor/" in topic and topic.endswith("/contact")
        ]
        candidate_topics = list(dict.fromkeys([*self._gz_contact_topics, *discovered_contact_topics]))
        seen_topics = [topic for topic in candidate_topics if topic in topic_names]
        output = ["# gz topic -l", *topics]
        for topic in candidate_topics:
            output.extend(["", f"# gz topic -i -t {topic}"])
            output.extend(self._run_command(["gz", "topic", "-i", "-t", topic], timeout=timeout))
            if topic not in seen_topics:
                continue
            output.extend(["", f"# gz topic -e -t {topic}"])
            output.extend(self._run_command(["gz", "topic", "-e", "-t", topic], timeout=timeout))
        return output, seen_topics

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
        except subprocess.TimeoutExpired as error:
            lines = []
            if error.stdout:
                stdout = error.stdout.decode() if isinstance(error.stdout, bytes) else error.stdout
                lines.extend(stdout.splitlines())
            if error.stderr:
                stderr = error.stderr.decode() if isinstance(error.stderr, bytes) else error.stderr
                lines.append("# stderr")
                lines.extend(stderr.splitlines())
            return lines if lines else [f"timeout: {' '.join(command)}"]
        output = []
        if completed.stdout:
            output.extend(completed.stdout.splitlines())
        if completed.stderr:
            output.append("# stderr")
            output.extend(completed.stderr.splitlines())
        if completed.returncode != 0:
            output.append(f"# exit_code={completed.returncode}")
        return output if output else ["no output"]

    def _parse_contact_wrench(self, lines: list[str]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        vectors: dict[str, list[tuple[float, float, float]]] = {"force": [], "torque": []}
        active: str | None = None
        values: dict[str, float] = {}
        for raw_line in lines:
            line = raw_line.strip()
            if line.startswith("force {"):
                active = "force"
                values = {}
                continue
            if line.startswith("torque {"):
                active = "torque"
                values = {}
                continue
            if active and line == "}":
                if {"x", "y", "z"}.issubset(values):
                    vectors[active].append((values["x"], values["y"], values["z"]))
                active = None
                values = {}
                continue
            if active:
                match = re.match(r"([xyz]):\s*(-?[0-9]+(?:\.[0-9]+)?(?:[eE][-+]?[0-9]+)?)", line)
                if match:
                    values[match.group(1)] = float(match.group(2))
        force = self._largest_vector(vectors["force"])
        torque = self._largest_vector(vectors["torque"])
        return force, torque

    def _largest_vector(self, vectors: list[tuple[float, float, float]]) -> tuple[float, float, float]:
        if not vectors:
            return (0.0, 0.0, 0.0)
        return max(vectors, key=lambda vector: math.sqrt(sum(value * value for value in vector)))

    def _force_magnitude(self, wrench: WrenchStamped | None) -> float:
        if wrench is None:
            return 0.0
        force = wrench.wrench.force
        return math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z)

    def _torque_magnitude(self, wrench: WrenchStamped | None) -> float:
        if wrench is None:
            return 0.0
        torque = wrench.wrench.torque
        return math.sqrt(torque.x * torque.x + torque.y * torque.y + torque.z * torque.z)

    def _wrench_sample_lines(self) -> list[str]:
        if self._last_wrench is None:
            return ["unavailable"]
        wrench = self._last_wrench.wrench
        return [
            f"topic: {self._wrench_topic}",
            f"frame_id: {self._last_wrench.header.frame_id}",
            f"force: {wrench.force.x:.6f},{wrench.force.y:.6f},{wrench.force.z:.6f}",
            f"torque: {wrench.torque.x:.6f},{wrench.torque.y:.6f},{wrench.torque.z:.6f}",
            f"force_magnitude_n: {self._force_magnitude(self._last_wrench):.6f}",
            f"torque_magnitude_nm: {self._torque_magnitude(self._last_wrench):.6f}",
        ]

    def _write_wrench_csv(self) -> None:
        path = self._output_dir / "contact_wrench_samples.csv"
        fields = [
            "elapsed_sec",
            "frame_id",
            "force_x_n",
            "force_y_n",
            "force_z_n",
            "torque_x_nm",
            "torque_y_nm",
            "torque_z_nm",
            "force_magnitude_n",
            "torque_magnitude_nm",
        ]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(self._wrench_rows)

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_3_contact_physics_validation",
            "",
            "Purpose: validate proposal-required contact-rich simulation foundations without robot control.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Collision bodies configured: `{status['peg_collision_configured'] and status['hole_collision_configured'] and status['table_collision_configured']}`",
            f"Contact wrench sample available: `{status['contact_wrench_sample_available']}`",
            f"Nonzero contact wrench observed: `{status['nonzero_contact_wrench_observed']}`",
            f"Max observed force N: `{status['max_observed_force_n']:.6f}`",
            f"Max observed torque Nm: `{status['max_observed_torque_nm']:.6f}`",
            f"RGB-D samples received: `{status['rgb_image_sample_received'] and status['depth_image_sample_received']}`",
            f"Task frames available: `{status['task_frames_available']}`",
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
            "proposal_simulation_cell_v1_3 contact validator evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"gz_available={shutil.which('gz') is not None}",
            f"status={status['status']}",
            f"contact_state={self._last_contact_state}",
            f"max_observed_force_n={status['max_observed_force_n']:.6f}",
            f"max_observed_torque_nm={status['max_observed_torque_nm']:.6f}",
            f"limitation_reason={status['limitation_reason']}",
            "phases=" + ",".join(PHASES),
            "",
            "# contact evidence",
            *self._gz_contact_output[:200],
        ]
        (self._output_dir / "run.log").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _write_lines(self, path: Path, lines: list[str]) -> None:
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV13ContactValidator()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
