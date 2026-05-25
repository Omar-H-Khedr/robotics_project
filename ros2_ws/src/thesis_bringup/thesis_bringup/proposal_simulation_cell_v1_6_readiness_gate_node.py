"""Simulation-only readiness gates for proposal_simulation_cell_v1_6."""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class ProposalSimulationCellV16ReadinessGateNode(Node):
    """Evaluate readiness from validated diagnostic signals without robot motion."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_6_readiness_gate_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_6")
        self.declare_parameter("isaac_available", False)
        self.declare_parameter("gazebo_fallback_used", True)
        self.declare_parameter("ros_gz_bridge_available", False)
        self.declare_parameter("ros_gz_image_available", False)

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_6")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        robot = self._config.get("robot", {})
        sensors = self._config.get("sensors", {})
        contact = self._config.get("contact", {})
        safety = self._config.get("safety", {})
        virtual_force = self._config.get("virtual_force", {})
        admittance = self._config.get("admittance", {})
        readiness = self._config.get("readiness_gates", {})

        self._robot_model = str(robot.get("model", "KUKA LBR iisy 6 R1300"))
        self._joint_state_topic = str(robot.get("joint_state_topic", "/joint_states"))
        self._tf_topic = str(robot.get("tf_topic", "/tf"))
        self._tf_static_topic = str(robot.get("tf_static_topic", "/tf_static"))
        self._task_phase_topic = str(robot.get("task_phase_topic", "/proposal_simulation_cell/task_phase"))
        self._rgb_topic = str(sensors.get("rgb_image_topic", "/proposal_simulation_cell/d405/color/image_raw"))
        self._depth_topic = str(
            sensors.get("depth_image_topic", "/proposal_simulation_cell/d405/depth/image_rect_raw")
        )
        self._wrench_topic = str(contact.get("wrench_topic", "/proposal_simulation_cell/contact_wrench"))
        self._contact_state_topic = str(contact.get("state_topic", "/proposal_simulation_cell/contact_state"))
        self._safety_status_topic = str(safety.get("safety_status_topic", "/proposal_simulation_cell/safety_status"))
        self._virtual_force_topic = str(virtual_force.get("topic", "/proposal_simulation_cell/virtual_force_command"))
        self._admittance_topic = str(
            admittance.get("topic", "/proposal_simulation_cell/admittance_command_suggestion")
        )
        self._readiness_gates_topic = str(
            readiness.get("readiness_gates_topic", "/proposal_simulation_cell/readiness_gates")
        )
        self._proposal_status_topic = str(
            readiness.get("proposal_readiness_status_topic", "/proposal_simulation_cell/proposal_readiness_status")
        )
        self._safety_report_topic = str(
            readiness.get("safety_gate_report_topic", "/proposal_simulation_cell/safety_gate_report")
        )

        self._max_allowed_torque = float(safety.get("max_allowed_torque_nm", 5.0))
        self._emergency_force = float(safety.get("emergency_stop_force_threshold_n", 45.0))
        self._max_safety_violations = int(safety.get("max_safety_violation_count", 0))
        self._command_output_enabled = bool(safety.get("command_output_enabled", False))
        self._motion_execution_enabled = bool(safety.get("motion_execution_enabled", False))
        self._real_robot_allowed = bool(safety.get("real_robot_allowed", False))
        self._moveit_allowed = bool(safety.get("moveit_allowed", False))
        self._compute_ik_allowed = bool(safety.get("compute_ik_allowed", False))
        self._controller_execution_allowed = bool(safety.get("controller_execution_allowed", False))
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 14.0))
        self._success_status = str(validation.get("status_success", "safety_gate_readiness_validated"))
        self._simulation_engine = str(validation.get("simulation_engine", "gazebo"))

        self._start_time = time.monotonic()
        self._finished = False
        self._last_rgb: Image | None = None
        self._last_depth: Image | None = None
        self._last_joint_state: JointState | None = None
        self._last_tf: TFMessage | None = None
        self._last_tf_static: TFMessage | None = None
        self._last_wrench: WrenchStamped | None = None
        self._last_contact_state: String | None = None
        self._last_safety_status: String | None = None
        self._last_virtual_force: WrenchStamped | None = None
        self._last_admittance: TwistStamped | None = None
        self._last_task_phase = ""
        self._max_observed_force = 0.0
        self._max_observed_torque = 0.0
        self._safety_violation_count = 0
        self._last_status: dict[str, Any] = {}
        self._gate_rows: list[dict[str, str]] = []
        self._proposal_rows: list[dict[str, str]] = []
        self._safety_rows: list[dict[str, str]] = []

        self._readiness_pub = self.create_publisher(String, self._readiness_gates_topic, 10)
        self._proposal_pub = self.create_publisher(String, self._proposal_status_topic, 10)
        self._safety_report_pub = self.create_publisher(String, self._safety_report_topic, 10)

        self.create_subscription(Image, self._rgb_topic, self._on_rgb, 10)
        self.create_subscription(Image, self._depth_topic, self._on_depth, 10)
        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 10)
        self.create_subscription(TFMessage, self._tf_topic, self._on_tf, 10)
        self.create_subscription(TFMessage, self._tf_static_topic, self._on_tf_static, 10)
        self.create_subscription(WrenchStamped, self._wrench_topic, self._on_wrench, 10)
        self.create_subscription(String, self._contact_state_topic, self._on_contact_state, 10)
        self.create_subscription(String, self._safety_status_topic, self._on_safety_status, 10)
        self.create_subscription(WrenchStamped, self._virtual_force_topic, self._on_virtual_force, 10)
        self.create_subscription(TwistStamped, self._admittance_topic, self._on_admittance, 10)
        self.create_subscription(String, self._task_phase_topic, self._on_task_phase, 10)

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_6 readiness gate node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.6 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _on_rgb(self, message: Image) -> None:
        self._last_rgb = message

    def _on_depth(self, message: Image) -> None:
        self._last_depth = message

    def _on_joint_state(self, message: JointState) -> None:
        self._last_joint_state = message

    def _on_tf(self, message: TFMessage) -> None:
        self._last_tf = message

    def _on_tf_static(self, message: TFMessage) -> None:
        self._last_tf_static = message

    def _on_wrench(self, message: WrenchStamped) -> None:
        self._last_wrench = message
        force_mag = self._force_magnitude(message)
        torque_mag = self._torque_magnitude(message)
        self._max_observed_force = max(self._max_observed_force, force_mag)
        self._max_observed_torque = max(self._max_observed_torque, torque_mag)

    def _on_contact_state(self, message: String) -> None:
        self._last_contact_state = message

    def _on_safety_status(self, message: String) -> None:
        self._last_safety_status = message
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if bool(payload.get("safety_violation", False)) or payload.get("contact_state") == "safety_violation":
            self._safety_violation_count += 1
        self._max_observed_force = max(
            self._max_observed_force,
            float(payload.get("force_magnitude_n", 0.0)),
            float(payload.get("max_observed_force_n", 0.0)),
        )
        self._max_observed_torque = max(
            self._max_observed_torque,
            float(payload.get("torque_magnitude_nm", 0.0)),
            float(payload.get("max_observed_torque_nm", 0.0)),
        )

    def _on_virtual_force(self, message: WrenchStamped) -> None:
        self._last_virtual_force = message

    def _on_admittance(self, message: TwistStamped) -> None:
        self._last_admittance = message

    def _on_task_phase(self, message: String) -> None:
        self._last_task_phase = message.data

    def _evaluate_and_publish(self) -> None:
        if self._finished:
            return
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        status = self._status(topics)
        self._last_status = status
        self._publish_json(self._readiness_pub, self._readiness_payload(status))
        self._publish_json(self._proposal_pub, self._proposal_payload(status))
        self._publish_json(self._safety_report_pub, self._safety_payload(status))
        self._record_rows(status)

    def _status(self, topics: list[str]) -> dict[str, Any]:
        topic_names = {line.split(" ", 1)[0] for line in topics}
        rgb_received = self._last_rgb is not None
        depth_received = self._last_depth is not None
        joint_states_available = self._joint_state_topic in topic_names and self._last_joint_state is not None
        tf_available = (
            self._tf_topic in topic_names
            and self._tf_static_topic in topic_names
            and (self._last_tf is not None or self._last_tf_static is not None)
        )
        contact_wrench_topic_available = self._wrench_topic in topic_names
        contact_wrench_sample_available = self._last_wrench is not None
        contact_state_topic_available = self._contact_state_topic in topic_names
        contact_state_sample_available = self._last_contact_state is not None
        safety_status_topic_available = self._safety_status_topic in topic_names
        safety_status_sample_available = self._last_safety_status is not None
        virtual_force_topic_available = self._virtual_force_topic in topic_names
        virtual_force_sample_available = self._last_virtual_force is not None
        admittance_topic_available = self._admittance_topic in topic_names
        admittance_sample_available = self._last_admittance is not None

        sensor_gate = rgb_received and depth_received and joint_states_available and tf_available
        contact_gate = (
            contact_wrench_topic_available
            and contact_wrench_sample_available
            and contact_state_topic_available
            and contact_state_sample_available
        )
        safety_gate = (
            safety_status_topic_available
            and safety_status_sample_available
            and self._safety_violation_count <= self._max_safety_violations
            and self._max_observed_force < self._emergency_force
            and self._max_observed_torque < self._max_allowed_torque
        )
        virtual_force_gate = (
            virtual_force_topic_available
            and virtual_force_sample_available
            and not self._command_output_enabled
        )
        admittance_gate = (
            admittance_topic_available
            and admittance_sample_available
            and not self._command_output_enabled
        )
        execution_disabled_gate = (
            not self._motion_execution_enabled
            and not self._real_robot_allowed
            and not self._moveit_allowed
            and not self._compute_ik_allowed
            and not self._controller_execution_allowed
        )
        proposal_gate = (
            sensor_gate
            and contact_gate
            and safety_gate
            and virtual_force_gate
            and admittance_gate
            and execution_disabled_gate
        )

        status_text = self._success_status if proposal_gate else "readiness_gate_pending"
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_fallback_used": bool(
                self.get_parameter("gazebo_fallback_used").get_parameter_value().bool_value
            ),
            "isaac_available": bool(
                self.get_parameter("isaac_available").get_parameter_value().bool_value
            ),
            "robot_model": self._robot_model,
            "sensor_gate_passed": sensor_gate,
            "contact_gate_passed": contact_gate,
            "safety_gate_passed": safety_gate,
            "virtual_force_gate_passed": virtual_force_gate,
            "admittance_gate_passed": admittance_gate,
            "execution_disabled_gate_passed": execution_disabled_gate,
            "proposal_readiness_gate_passed": proposal_gate,
            "rgb_image_sample_received": rgb_received,
            "depth_image_sample_received": depth_received,
            "joint_states_available": joint_states_available,
            "tf_available": tf_available,
            "contact_wrench_topic_available": contact_wrench_topic_available,
            "contact_wrench_sample_available": contact_wrench_sample_available,
            "contact_state_topic_available": contact_state_topic_available,
            "contact_state_sample_available": contact_state_sample_available,
            "safety_status_topic_available": safety_status_topic_available,
            "safety_status_sample_available": safety_status_sample_available,
            "virtual_force_command_topic_available": virtual_force_topic_available,
            "virtual_force_command_sample_available": virtual_force_sample_available,
            "admittance_command_suggestion_topic_available": admittance_topic_available,
            "admittance_command_suggestion_sample_available": admittance_sample_available,
            "max_observed_force_n": self._max_observed_force,
            "max_observed_torque_nm": self._max_observed_torque,
            "safety_violation_count": self._safety_violation_count,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "controller_execution_allowed": False,
            "task_phase_available": bool(self._last_task_phase),
            "latest_task_phase": self._last_task_phase,
            "status": status_text,
        }

    def _readiness_payload(self, status: dict[str, Any]) -> dict[str, Any]:
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "sensor_gate": status["sensor_gate_passed"],
            "contact_gate": status["contact_gate_passed"],
            "safety_gate": status["safety_gate_passed"],
            "virtual_force_gate": status["virtual_force_gate_passed"],
            "admittance_gate": status["admittance_gate_passed"],
            "execution_disabled_gate": status["execution_disabled_gate_passed"],
            "proposal_readiness_gate": status["proposal_readiness_gate_passed"],
            "status": status["status"],
        }

    def _proposal_payload(self, status: dict[str, Any]) -> dict[str, Any]:
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "proposal_readiness_gate_passed": status["proposal_readiness_gate_passed"],
            "ready_for_next_control_development_stage": status["proposal_readiness_gate_passed"],
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "status": status["status"],
        }

    def _safety_payload(self, status: dict[str, Any]) -> dict[str, Any]:
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "safety_gate_passed": status["safety_gate_passed"],
            "safety_status_sample_available": status["safety_status_sample_available"],
            "max_observed_force_n": status["max_observed_force_n"],
            "max_observed_torque_nm": status["max_observed_torque_nm"],
            "safety_violation_count": status["safety_violation_count"],
            "emergency_stop_force_threshold_n": self._emergency_force,
            "max_allowed_torque_nm": self._max_allowed_torque,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
        }

    def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        publisher.publish(message)

    def _record_rows(self, status: dict[str, Any]) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._gate_rows.append(
            {
                "elapsed_sec": elapsed,
                "sensor_gate_passed": self._bool(status["sensor_gate_passed"]),
                "contact_gate_passed": self._bool(status["contact_gate_passed"]),
                "safety_gate_passed": self._bool(status["safety_gate_passed"]),
                "virtual_force_gate_passed": self._bool(status["virtual_force_gate_passed"]),
                "admittance_gate_passed": self._bool(status["admittance_gate_passed"]),
                "execution_disabled_gate_passed": self._bool(status["execution_disabled_gate_passed"]),
                "proposal_readiness_gate_passed": self._bool(status["proposal_readiness_gate_passed"]),
                "status": str(status["status"]),
            }
        )
        self._proposal_rows.append(
            {
                "elapsed_sec": elapsed,
                "proposal_readiness_gate_passed": self._bool(status["proposal_readiness_gate_passed"]),
                "ready_for_next_control_development_stage": self._bool(
                    status["proposal_readiness_gate_passed"]
                ),
                "command_output_enabled": "false",
                "motion_execution_enabled": "false",
                "status": str(status["status"]),
            }
        )
        self._safety_rows.append(
            {
                "elapsed_sec": elapsed,
                "safety_gate_passed": self._bool(status["safety_gate_passed"]),
                "safety_status_sample_available": self._bool(status["safety_status_sample_available"]),
                "max_observed_force_n": f"{float(status['max_observed_force_n']):.6f}",
                "max_observed_torque_nm": f"{float(status['max_observed_torque_nm']):.6f}",
                "safety_violation_count": str(status["safety_violation_count"]),
                "command_output_enabled": "false",
                "motion_execution_enabled": "false",
            }
        )

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        status = self._status(topics)
        self._last_status = status
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", self._tf_frame_names())
        self._write_csv(self._output_dir / "readiness_gate_samples.csv", self._gate_rows)
        self._write_csv(self._output_dir / "proposal_readiness_status_samples.csv", self._proposal_rows)
        self._write_csv(self._output_dir / "safety_gate_report_samples.csv", self._safety_rows)
        self._write_json(self._output_dir / "readiness_gate_status.json", status)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_6 readiness gate diagnostics written")
        rclpy.shutdown()

    def _tf_frame_names(self) -> list[str]:
        frames = {"world", "base_link", "tool0", "peg_tip", "hole_center", "insertion_axis_z"}
        if self._last_tf:
            frames.update(transform.child_frame_id for transform in self._last_tf.transforms)
        if self._last_tf_static:
            frames.update(transform.child_frame_id for transform in self._last_tf_static.transforms)
        return sorted(frame for frame in frames if frame)

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_6_safety_gate_readiness",
            "",
            "Purpose: evaluate simulation-only readiness gates from validated diagnostic signals.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Sensor gate passed: `{status['sensor_gate_passed']}`",
            f"Contact gate passed: `{status['contact_gate_passed']}`",
            f"Safety gate passed: `{status['safety_gate_passed']}`",
            f"Virtual-force gate passed: `{status['virtual_force_gate_passed']}`",
            f"Admittance gate passed: `{status['admittance_gate_passed']}`",
            f"Execution-disabled gate passed: `{status['execution_disabled_gate_passed']}`",
            f"Proposal readiness gate passed: `{status['proposal_readiness_gate_passed']}`",
            f"Status: `{status['status']}`",
            "",
            "Safety constraints: command_output_enabled=false, motion_execution_enabled=false, real robot unused, MoveIt unused, /compute_ik not called, and no controller execution.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_6 safety gate readiness evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"sensor_gate_passed={str(status['sensor_gate_passed']).lower()}",
            f"contact_gate_passed={str(status['contact_gate_passed']).lower()}",
            f"safety_gate_passed={str(status['safety_gate_passed']).lower()}",
            f"virtual_force_gate_passed={str(status['virtual_force_gate_passed']).lower()}",
            f"admittance_gate_passed={str(status['admittance_gate_passed']).lower()}",
            f"execution_disabled_gate_passed={str(status['execution_disabled_gate_passed']).lower()}",
            f"proposal_readiness_gate_passed={str(status['proposal_readiness_gate_passed']).lower()}",
            f"max_observed_force_n={status['max_observed_force_n']:.6f}",
            f"max_observed_torque_nm={status['max_observed_torque_nm']:.6f}",
            "command_output_enabled=false",
            "motion_execution_enabled=false",
            "",
        ]
        (self._output_dir / "run.log").write_text("\n".join(lines), encoding="utf-8")

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

    def _force_magnitude(self, wrench: WrenchStamped) -> float:
        force = wrench.wrench.force
        return math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z)

    def _torque_magnitude(self, wrench: WrenchStamped) -> float:
        torque = wrench.wrench.torque
        return math.sqrt(torque.x * torque.x + torque.y * torque.y + torque.z * torque.z)

    def _bool(self, value: Any) -> str:
        return str(bool(value)).lower()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV16ReadinessGateNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
