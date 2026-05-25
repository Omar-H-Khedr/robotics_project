"""Safety and virtual-force diagnostic interface for proposal_simulation_cell_v1_5."""

from __future__ import annotations

import csv
import json
import math
import re
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped, TwistStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


PHASES = (
    "initialize_safety_interface",
    "read_simulated_contact_wrench",
    "classify_contact_state",
    "compute_virtual_force_suggestion",
    "compute_admittance_suggestion",
    "write_safety_virtual_force_diagnostics",
)
TF_FRAMES = (
    "world",
    "base_link",
    "tool0",
    "peg_tip",
    "hole_center",
    "insertion_axis_z",
    "contact_frame",
)


class ProposalSimulationCellV15SafetyVirtualForceNode(Node):
    """Publish diagnostic safety and virtual-force suggestions without robot execution."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_5_safety_virtual_force_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_5")
        self.declare_parameter("isaac_available", False)
        self.declare_parameter("gazebo_fallback_used", True)
        self.declare_parameter("ros_gz_bridge_available", False)
        self.declare_parameter("world_path", "")

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_5")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        robot = self._config.get("robot", {})
        contact = self._config.get("contact", {})
        safety = self._config.get("safety", {})
        virtual_force = self._config.get("virtual_force", {})
        admittance = self._config.get("admittance", {})

        self._robot_model = str(robot.get("model", "KUKA LBR iisy 6 R1300"))
        self._joint_state_topic = str(robot.get("joint_state_topic", "/joint_states"))
        self._tf_topic = str(robot.get("tf_topic", "/tf"))
        self._tf_static_topic = str(robot.get("tf_static_topic", "/tf_static"))
        self._task_phase_topic = str(robot.get("task_phase_topic", "/proposal_simulation_cell/task_phase"))
        self._wrench_topic = str(contact.get("wrench_topic", "/proposal_simulation_cell/contact_wrench"))
        self._contact_state_topic = str(contact.get("state_topic", "/proposal_simulation_cell/contact_state"))
        self._safety_status_topic = str(safety.get("safety_status_topic", "/proposal_simulation_cell/safety_status"))
        self._virtual_force_topic = str(virtual_force.get("topic", "/proposal_simulation_cell/virtual_force_command"))
        self._admittance_topic = str(
            admittance.get("topic", "/proposal_simulation_cell/admittance_command_suggestion")
        )
        self._contact_frame = str(contact.get("frame_id", "contact_frame"))
        self._expected_axis = str(contact.get("expected_contact_axis", "z"))
        self._gz_contact_topics = [
            str(topic) for topic in contact.get("gazebo_contact_topics", []) if str(topic)
        ]

        self._contact_threshold = float(contact.get("contact_detection_force_threshold_n", 0.1))
        self._max_allowed_force = float(safety.get("max_allowed_force_n", 50.0))
        self._max_allowed_torque = float(safety.get("max_allowed_torque_nm", 5.0))
        self._warning_force = float(safety.get("warning_force_threshold_n", 25.0))
        self._emergency_force = float(safety.get("emergency_stop_force_threshold_n", 45.0))
        self._command_output_enabled = bool(safety.get("command_output_enabled", False))
        self._motion_execution_enabled = bool(safety.get("motion_execution_enabled", False))
        self._virtual_stiffness = float(virtual_force.get("virtual_stiffness_n_per_m", 1200.0))
        self._virtual_damping = float(virtual_force.get("virtual_damping_ns_per_m", 80.0))
        self._max_virtual_force = float(virtual_force.get("max_commanded_force_n", 20.0))
        self._admittance_mass = float(admittance.get("admittance_mass_kg", 2.0))
        self._admittance_damping = float(admittance.get("admittance_damping_ns_per_m", 120.0))
        self._max_velocity = float(admittance.get("max_commanded_velocity_mps", 0.02))
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 12.0))

        self._start_time = time.monotonic()
        self._phase_index = 0
        self._finished = False
        self._last_wrench: WrenchStamped | None = None
        self._last_joint_state: JointState | None = None
        self._last_tf: TFMessage | None = None
        self._last_tf_static: TFMessage | None = None
        self._last_task_phase = ""
        self._last_contact_state = "no_contact"
        self._last_safety_payload: dict[str, Any] = {}
        self._last_virtual_force = (0.0, 0.0, 0.0)
        self._last_admittance_velocity = (0.0, 0.0, 0.0)
        self._max_observed_force = 0.0
        self._max_observed_torque = 0.0
        self._safety_violation_count = 0
        self._gz_contact_output: list[str] = []
        self._gz_contact_topics_seen: list[str] = []
        self._poll_attempted = False
        self._safety_rows: list[dict[str, str]] = []
        self._contact_rows: list[dict[str, str]] = []
        self._virtual_force_rows: list[dict[str, str]] = []
        self._admittance_rows: list[dict[str, str]] = []

        self._phase_pub = self.create_publisher(String, self._task_phase_topic, 10)
        self._joint_state_pub = self.create_publisher(JointState, self._joint_state_topic, 10)
        self._contact_wrench_pub = self.create_publisher(WrenchStamped, self._wrench_topic, 10)
        self._safety_status_pub = self.create_publisher(String, self._safety_status_topic, 10)
        self._contact_state_pub = self.create_publisher(String, self._contact_state_topic, 10)
        self._virtual_force_pub = self.create_publisher(WrenchStamped, self._virtual_force_topic, 10)
        self._admittance_pub = self.create_publisher(TwistStamped, self._admittance_topic, 10)

        self.create_subscription(WrenchStamped, self._wrench_topic, self._on_wrench, 10)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 10)
        self.create_subscription(TFMessage, self._tf_topic, self._on_tf, 10)
        self.create_subscription(TFMessage, self._tf_static_topic, self._on_tf_static, 10)
        self.create_subscription(String, self._task_phase_topic, self._on_task_phase, 10)

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_task_frames()

        self.create_timer(self._sample_period, self._publish_diagnostics)
        self.create_timer(self._sample_period, self._publish_joint_state)
        self.create_timer(0.8, self._advance_phase)
        self.create_timer(1.0, self._poll_gz_contacts_once)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_5 safety virtual-force node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.5 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _publish_static_task_frames(self) -> None:
        transforms = [
            self._transform("world", "hole_center", [0.52, -0.20, 0.83]),
            self._transform("hole_center", "insertion_axis_z", [0.0, 0.0, 0.08]),
            self._transform("world", "contact_frame", [0.52, -0.20, 0.84]),
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

    def _on_wrench(self, message: WrenchStamped) -> None:
        self._last_wrench = message

    def _on_joint_state(self, message: JointState) -> None:
        self._last_joint_state = message

    def _on_tf(self, message: TFMessage) -> None:
        self._last_tf = message

    def _on_tf_static(self, message: TFMessage) -> None:
        self._last_tf_static = message

    def _on_task_phase(self, message: String) -> None:
        self._last_task_phase = message.data

    def _advance_phase(self) -> None:
        if self._phase_index >= len(PHASES):
            return
        message = String()
        message.data = PHASES[self._phase_index]
        self._phase_pub.publish(message)
        self.get_logger().info(f"phase={message.data}")
        self._phase_index += 1

    def _publish_joint_state(self) -> None:
        if self._finished:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"]
        message.position = [0.0, -0.55, 0.72, 0.0, 0.85, 0.0]
        message.velocity = [0.0] * 6
        message.effort = [0.0] * 6
        self._joint_state_pub.publish(message)

    def _poll_gz_contacts_once(self) -> None:
        elapsed = time.monotonic() - self._start_time
        if self._finished or self._poll_attempted or elapsed < 5.0:
            return
        self._poll_attempted = True
        output, seen_topics = self._read_gz_contacts(timeout=3.0)
        self._gz_contact_output = output
        self._gz_contact_topics_seen = seen_topics
        force, torque = self._parse_contact_wrench(output)
        self._publish_contact_wrench(force, torque)

    def _publish_contact_wrench(
        self,
        force: tuple[float, float, float],
        torque: tuple[float, float, float],
    ) -> None:
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = self._contact_frame
        wrench.wrench.force.x = force[0]
        wrench.wrench.force.y = force[1]
        wrench.wrench.force.z = force[2]
        wrench.wrench.torque.x = torque[0]
        wrench.wrench.torque.y = torque[1]
        wrench.wrench.torque.z = torque[2]
        self._last_wrench = wrench
        self._contact_wrench_pub.publish(wrench)

    def _publish_diagnostics(self) -> None:
        if self._finished:
            return
        if self._last_wrench is None:
            self._publish_contact_wrench((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

        force = self._force_tuple(self._last_wrench)
        torque = self._torque_tuple(self._last_wrench)
        force_mag = math.sqrt(sum(value * value for value in force))
        torque_mag = math.sqrt(sum(value * value for value in torque))
        self._max_observed_force = max(self._max_observed_force, force_mag)
        self._max_observed_torque = max(self._max_observed_torque, torque_mag)
        state = self._classify_contact(force_mag, torque_mag)
        if state == "safety_violation" and self._last_contact_state != "safety_violation":
            self._safety_violation_count += 1
        self._last_contact_state = state
        virtual_force = self._virtual_force_suggestion(force)
        admittance_velocity = self._admittance_velocity_suggestion(force)
        self._last_virtual_force = virtual_force
        self._last_admittance_velocity = admittance_velocity

        safety_payload = {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "contact_state": state,
            "force_magnitude_n": force_mag,
            "torque_magnitude_nm": torque_mag,
            "max_allowed_force_n": self._max_allowed_force,
            "max_allowed_torque_nm": self._max_allowed_torque,
            "warning_force_threshold_n": self._warning_force,
            "emergency_stop_force_threshold_n": self._emergency_force,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
        }
        self._last_safety_payload = safety_payload

        safety_msg = String()
        safety_msg.data = json.dumps(safety_payload, sort_keys=True)
        contact_msg = String()
        contact_msg.data = state
        self._safety_status_pub.publish(safety_msg)
        self._contact_state_pub.publish(contact_msg)
        self._publish_virtual_force(virtual_force)
        self._publish_admittance_velocity(admittance_velocity)
        self._record_rows(force, torque, force_mag, torque_mag, state, virtual_force, admittance_velocity)

    def _classify_contact(self, force_mag: float, torque_mag: float) -> str:
        if force_mag <= 1.0e-9 and torque_mag <= 1.0e-9:
            return "no_contact"
        if force_mag < self._contact_threshold:
            return "contact_below_threshold"
        if (
            force_mag >= self._emergency_force
            or force_mag > self._max_allowed_force
            or torque_mag > self._max_allowed_torque
        ):
            return "safety_violation"
        if force_mag >= self._warning_force:
            return "warning"
        return "contact_valid"

    def _virtual_force_suggestion(
        self, force: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        axis_index = {"x": 0, "y": 1, "z": 2}.get(self._expected_axis, 2)
        axis_force = force[axis_index]
        if abs(axis_force) <= 1.0e-9:
            return (0.0, 0.0, 0.0)
        suggested_axis_force = -max(-self._max_virtual_force, min(self._max_virtual_force, axis_force))
        values = [0.0, 0.0, 0.0]
        values[axis_index] = suggested_axis_force
        return (values[0], values[1], values[2])

    def _admittance_velocity_suggestion(
        self, force: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        axis_index = {"x": 0, "y": 1, "z": 2}.get(self._expected_axis, 2)
        axis_force = force[axis_index]
        if abs(axis_force) <= 1.0e-9 or self._admittance_damping <= 0.0:
            return (0.0, 0.0, 0.0)
        velocity = -axis_force / self._admittance_damping
        velocity = max(-self._max_velocity, min(self._max_velocity, velocity))
        values = [0.0, 0.0, 0.0]
        values[axis_index] = velocity
        return (values[0], values[1], values[2])

    def _publish_virtual_force(self, force: tuple[float, float, float]) -> None:
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._contact_frame
        message.wrench.force.x = force[0]
        message.wrench.force.y = force[1]
        message.wrench.force.z = force[2]
        self._virtual_force_pub.publish(message)

    def _publish_admittance_velocity(self, velocity: tuple[float, float, float]) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._contact_frame
        message.twist.linear.x = velocity[0]
        message.twist.linear.y = velocity[1]
        message.twist.linear.z = velocity[2]
        self._admittance_pub.publish(message)

    def _record_rows(
        self,
        force: tuple[float, float, float],
        torque: tuple[float, float, float],
        force_mag: float,
        torque_mag: float,
        state: str,
        virtual_force: tuple[float, float, float],
        admittance_velocity: tuple[float, float, float],
    ) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._safety_rows.append(
            {
                "elapsed_sec": elapsed,
                "contact_state": state,
                "force_magnitude_n": f"{force_mag:.6f}",
                "torque_magnitude_nm": f"{torque_mag:.6f}",
                "command_output_enabled": "false",
                "motion_execution_enabled": "false",
            }
        )
        self._contact_rows.append(
            {
                "elapsed_sec": elapsed,
                "contact_state": state,
                "force_x_n": f"{force[0]:.6f}",
                "force_y_n": f"{force[1]:.6f}",
                "force_z_n": f"{force[2]:.6f}",
                "torque_x_nm": f"{torque[0]:.6f}",
                "torque_y_nm": f"{torque[1]:.6f}",
                "torque_z_nm": f"{torque[2]:.6f}",
                "force_magnitude_n": f"{force_mag:.6f}",
                "torque_magnitude_nm": f"{torque_mag:.6f}",
            }
        )
        self._virtual_force_rows.append(
            {
                "elapsed_sec": elapsed,
                "frame_id": self._contact_frame,
                "suggested_force_x_n": f"{virtual_force[0]:.6f}",
                "suggested_force_y_n": f"{virtual_force[1]:.6f}",
                "suggested_force_z_n": f"{virtual_force[2]:.6f}",
                "command_output_enabled": "false",
            }
        )
        self._admittance_rows.append(
            {
                "elapsed_sec": elapsed,
                "frame_id": self._contact_frame,
                "suggested_linear_x_mps": f"{admittance_velocity[0]:.6f}",
                "suggested_linear_y_mps": f"{admittance_velocity[1]:.6f}",
                "suggested_linear_z_mps": f"{admittance_velocity[2]:.6f}",
                "motion_execution_enabled": "false",
            }
        )

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        if not self._poll_attempted:
            output, seen_topics = self._read_gz_contacts(timeout=5.0)
            self._gz_contact_output = output
            self._gz_contact_topics_seen = seen_topics
            force, torque = self._parse_contact_wrench(output)
            self._publish_contact_wrench(force, torque)
            self._publish_diagnostics()
        self._finished = True

        nodes = sorted(name for name in self.get_node_names() if name)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(
            f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types()
        )
        tf_frames = self._tf_frame_names()
        status = self._status(topics)

        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", tf_frames)
        self._write_csv(self._output_dir / "safety_status_samples.csv", self._safety_rows)
        self._write_csv(self._output_dir / "contact_state_samples.csv", self._contact_rows)
        self._write_csv(self._output_dir / "virtual_force_command_samples.csv", self._virtual_force_rows)
        self._write_csv(
            self._output_dir / "admittance_command_suggestion_samples.csv",
            self._admittance_rows,
        )
        self._write_json(self._output_dir / "safety_virtual_force_status.json", status)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_5 safety virtual-force diagnostics written")
        rclpy.shutdown()

    def _status(self, topics: list[str]) -> dict[str, Any]:
        topic_names = {line.split(" ", 1)[0] for line in topics}
        required_outputs_available = (
            self._safety_status_topic in topic_names
            and self._contact_state_topic in topic_names
            and self._virtual_force_topic in topic_names
            and self._admittance_topic in topic_names
        )
        contact_wrench_sample_available = self._last_wrench is not None
        status = (
            "safety_virtual_force_interface_validated"
            if required_outputs_available and contact_wrench_sample_available
            else "failed"
        )
        return {
            "simulation_engine": "gazebo",
            "gazebo_fallback_used": bool(
                self.get_parameter("gazebo_fallback_used").get_parameter_value().bool_value
            ),
            "isaac_available": bool(
                self.get_parameter("isaac_available").get_parameter_value().bool_value
            ),
            "robot_model": self._robot_model,
            "contact_wrench_topic_available": self._wrench_topic in topic_names,
            "contact_wrench_sample_available": contact_wrench_sample_available,
            "safety_status_topic_available": self._safety_status_topic in topic_names,
            "contact_state_topic_available": self._contact_state_topic in topic_names,
            "virtual_force_command_topic_available": self._virtual_force_topic in topic_names,
            "admittance_command_suggestion_topic_available": self._admittance_topic in topic_names,
            "max_observed_force_n": self._max_observed_force,
            "max_observed_torque_nm": self._max_observed_torque,
            "contact_detection_threshold_n": self._contact_threshold,
            "warning_force_threshold_n": self._warning_force,
            "emergency_stop_force_threshold_n": self._emergency_force,
            "safety_violation_count": self._safety_violation_count,
            "virtual_force_interface_available": self._virtual_force_topic in topic_names,
            "admittance_interface_available": self._admittance_topic in topic_names,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "latest_contact_state": self._last_contact_state,
            "expected_contact_axis": self._expected_axis,
            "max_allowed_force_n": self._max_allowed_force,
            "max_allowed_torque_nm": self._max_allowed_torque,
            "joint_states_available": self._last_joint_state is not None,
            "tf_available": self._last_tf is not None,
            "tf_static_available": self._last_tf_static is not None,
            "task_phase_available": bool(self._last_task_phase),
            "gazebo_contact_topics_seen": self._gz_contact_topics_seen,
            "virtual_stiffness_n_per_m": self._virtual_stiffness,
            "virtual_damping_ns_per_m": self._virtual_damping,
            "admittance_mass_kg": self._admittance_mass,
            "admittance_damping_ns_per_m": self._admittance_damping,
            "max_commanded_velocity_mps": self._max_velocity,
            "status": status,
        }

    def _tf_frame_names(self) -> list[str]:
        frames = set(TF_FRAMES)
        if self._last_tf:
            frames.update(transform.child_frame_id for transform in self._last_tf.transforms)
        if self._last_tf_static:
            frames.update(transform.child_frame_id for transform in self._last_tf_static.transforms)
        return sorted(frames)

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
        return self._largest_vector(vectors["force"]), self._largest_vector(vectors["torque"])

    def _largest_vector(self, vectors: list[tuple[float, float, float]]) -> tuple[float, float, float]:
        if not vectors:
            return (0.0, 0.0, 0.0)
        return max(vectors, key=lambda vector: math.sqrt(sum(value * value for value in vector)))

    def _force_tuple(self, wrench: WrenchStamped | None) -> tuple[float, float, float]:
        if wrench is None:
            return (0.0, 0.0, 0.0)
        force = wrench.wrench.force
        return (force.x, force.y, force.z)

    def _torque_tuple(self, wrench: WrenchStamped | None) -> tuple[float, float, float]:
        if wrench is None:
            return (0.0, 0.0, 0.0)
        torque = wrench.wrench.torque
        return (torque.x, torque.y, torque.z)

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_5_safety_virtual_force_interface",
            "",
            "Purpose: validate simulation-only safety filtering and virtual-force/admittance diagnostic suggestions.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Contact wrench sample available: `{status['contact_wrench_sample_available']}`",
            f"Safety status topic available: `{status['safety_status_topic_available']}`",
            f"Contact state topic available: `{status['contact_state_topic_available']}`",
            f"Virtual-force topic available: `{status['virtual_force_command_topic_available']}`",
            f"Admittance suggestion topic available: `{status['admittance_command_suggestion_topic_available']}`",
            f"Max observed force N: `{status['max_observed_force_n']:.6f}`",
            f"Max observed torque Nm: `{status['max_observed_torque_nm']:.6f}`",
            f"Latest contact state: `{status['latest_contact_state']}`",
            f"Status: `{status['status']}`",
            "",
            "Safety constraints: command_output_enabled=false, motion_execution_enabled=false, real robot unused, MoveIt unused, /compute_ik not called, and no controller execution.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_5 safety virtual-force evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"latest_contact_state={status['latest_contact_state']}",
            f"max_observed_force_n={status['max_observed_force_n']:.6f}",
            f"max_observed_torque_nm={status['max_observed_torque_nm']:.6f}",
            f"command_output_enabled={str(status['command_output_enabled']).lower()}",
            f"motion_execution_enabled={str(status['motion_execution_enabled']).lower()}",
            "phases=" + ",".join(PHASES),
            "",
            "# contact evidence",
            *self._gz_contact_output[:200],
        ]
        (self._output_dir / "run.log").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _write_csv(self, path: Path, rows: list[dict[str, str]]) -> None:
        fields = list(rows[0].keys()) if rows else ["elapsed_sec"]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _write_lines(self, path: Path, lines: list[str]) -> None:
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV15SafetyVirtualForceNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
