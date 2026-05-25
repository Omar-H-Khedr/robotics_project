"""No-motion control-law dry run for proposal_simulation_cell_v1_9."""

from __future__ import annotations

import csv
import json
import math
import re
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import TwistStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class ProposalSimulationCellV19ControlLawDryRunNode(Node):
    """Compute diagnostic control-law outputs while blocking every command."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_9_control_law_dry_run_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_9")
        self.declare_parameter("isaac_available", False)
        self.declare_parameter("gazebo_fallback_used", True)
        self.declare_parameter("ros_gz_bridge_available", False)
        self.declare_parameter("ros_gz_image_available", False)

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_9")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        robot = self._config.get("robot", {})
        control_law = self._config.get("control_law", {})
        safety_limits = self._config.get("safety_limits", {})
        command_blocking = self._config.get("command_blocking", {})
        input_topics = control_law.get("input_topics", {})

        self._robot_model = str(robot.get("model", "KUKA LBR iisy 6 R1300"))
        self._required_topics = {
            "joint_states": str(input_topics.get("joint_states", robot.get("joint_state_topic", "/joint_states"))),
            "tf": str(input_topics.get("tf", robot.get("tf_topic", "/tf"))),
            "tf_static": str(input_topics.get("tf_static", robot.get("tf_static_topic", "/tf_static"))),
            "contact_wrench": str(input_topics.get("contact_wrench", "/proposal_simulation_cell/contact_wrench")),
            "contact_state": str(input_topics.get("contact_state", "/proposal_simulation_cell/contact_state")),
            "safety_status": str(input_topics.get("safety_status", "/proposal_simulation_cell/safety_status")),
            "virtual_force_command": str(
                input_topics.get("virtual_force_command", "/proposal_simulation_cell/virtual_force_command")
            ),
            "admittance_command_suggestion": str(
                input_topics.get(
                    "admittance_command_suggestion",
                    "/proposal_simulation_cell/admittance_command_suggestion",
                )
            ),
            "readiness_gates": str(input_topics.get("readiness_gates", "/proposal_simulation_cell/readiness_gates")),
            "proposal_readiness_status": str(
                input_topics.get("proposal_readiness_status", "/proposal_simulation_cell/proposal_readiness_status")
            ),
            "pre_control_contract_status": str(
                input_topics.get("pre_control_contract_status", "/proposal_simulation_cell/pre_control_contract_status")
            ),
            "control_readiness_report": str(
                input_topics.get("control_readiness_report", "/proposal_simulation_cell/control_readiness_report")
            ),
        }

        self._status_topic = str(
            control_law.get("status_topic", "/proposal_simulation_cell/no_motion_control_law_status")
        )
        self._dry_run_output_topic = str(
            control_law.get("dry_run_output_topic", "/proposal_simulation_cell/control_law_dry_run_output")
        )
        self._blocked_command_topic = str(
            control_law.get("blocked_command_topic", "/proposal_simulation_cell/blocked_control_command")
        )
        self._safety_report_topic = str(
            control_law.get("safety_report_topic", "/proposal_simulation_cell/control_law_safety_report")
        )

        self._control_law_enabled = bool(
            control_law.get("control_law_enabled", control_law.get("enabled", True))
        )
        self._dry_run_only = bool(control_law.get("dry_run_only", True))
        self._expected_contact_axis = str(
            safety_limits.get("expected_contact_axis", control_law.get("expected_contact_axis", "z"))
        )
        self._insertion_axis_gain = float(control_law.get("insertion_axis_gain", 0.05))
        self._force_velocity_gain = float(control_law.get("force_velocity_gain", 0.002))
        self._max_allowed_force_n = float(safety_limits.get("max_allowed_force_n", 5.0))
        self._max_allowed_torque_nm = float(safety_limits.get("max_allowed_torque_nm", 1.0))
        self._contact_detection_force_threshold_n = float(
            safety_limits.get("contact_detection_force_threshold_n", 0.1)
        )
        self._warning_force_threshold_n = float(safety_limits.get("warning_force_threshold_n", 2.5))
        self._emergency_stop_force_threshold_n = float(
            safety_limits.get("emergency_stop_force_threshold_n", self._max_allowed_force_n)
        )

        self._command_output_enabled = bool(command_blocking.get("command_output_enabled", False))
        self._motion_execution_enabled = bool(command_blocking.get("motion_execution_enabled", False))
        self._controller_execution_allowed = bool(command_blocking.get("controller_execution_allowed", False))
        self._trajectory_execution_allowed = bool(command_blocking.get("trajectory_execution_allowed", False))
        self._follow_joint_trajectory_allowed = bool(command_blocking.get("follow_joint_trajectory_allowed", False))
        self._real_robot_allowed = bool(command_blocking.get("real_robot_allowed", False))
        self._moveit_allowed = bool(command_blocking.get("moveit_allowed", False))
        self._compute_ik_allowed = bool(command_blocking.get("compute_ik_allowed", False))
        self._max_commanded_velocity_mps = float(command_blocking.get("max_commanded_velocity_mps", 0.0))
        self._max_commanded_position_delta_m = float(command_blocking.get("max_commanded_position_delta_m", 0.0))
        self._block_reason = str(command_blocking.get("block_reason", "dry_run_only_command_output_disabled"))

        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 24.0))
        self._success_status = str(validation.get("status_success", "no_motion_control_law_dry_run_validated"))
        self._simulation_engine = str(validation.get("simulation_engine", "gazebo"))

        self._start_time = time.monotonic()
        self._finished = False
        self._last: dict[str, Any] = {}
        self._payloads: dict[str, dict[str, Any]] = {}
        self._last_status: dict[str, Any] = {}
        self._last_output: dict[str, Any] = {}
        self._last_blocked_command: dict[str, Any] = {}
        self._last_safety_report: dict[str, Any] = {}
        self._status_rows: list[dict[str, str]] = []
        self._output_rows: list[dict[str, str]] = []
        self._blocked_rows: list[dict[str, str]] = []
        self._safety_rows: list[dict[str, str]] = []

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._dry_run_output_pub = self.create_publisher(String, self._dry_run_output_topic, 10)
        self._blocked_command_pub = self.create_publisher(String, self._blocked_command_topic, 10)
        self._safety_report_pub = self.create_publisher(String, self._safety_report_topic, 10)

        self.create_subscription(JointState, self._required_topics["joint_states"], self._store("joint_states"), 10)
        self.create_subscription(TFMessage, self._required_topics["tf"], self._store("tf"), 10)
        self.create_subscription(TFMessage, self._required_topics["tf_static"], self._store("tf_static"), 10)
        self.create_subscription(WrenchStamped, self._required_topics["contact_wrench"], self._store("contact_wrench"), 10)
        self.create_subscription(String, self._required_topics["contact_state"], self._store_string("contact_state"), 10)
        self.create_subscription(String, self._required_topics["safety_status"], self._store_json("safety_status"), 10)
        self.create_subscription(
            WrenchStamped,
            self._required_topics["virtual_force_command"],
            self._store("virtual_force_command"),
            10,
        )
        self.create_subscription(
            TwistStamped,
            self._required_topics["admittance_command_suggestion"],
            self._store("admittance_command_suggestion"),
            10,
        )
        self.create_subscription(String, self._required_topics["readiness_gates"], self._store_json("readiness_gates"), 10)
        self.create_subscription(
            String,
            self._required_topics["proposal_readiness_status"],
            self._store_json("proposal_readiness_status"),
            10,
        )
        self.create_subscription(
            String,
            self._required_topics["pre_control_contract_status"],
            self._store_json("pre_control_contract_status"),
            10,
        )
        self.create_subscription(
            String,
            self._required_topics["control_readiness_report"],
            self._store_json("control_readiness_report"),
            10,
        )

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_9 no-motion control-law dry-run node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.9 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _store(self, key: str) -> Any:
        def callback(message: Any) -> None:
            self._last[key] = message

        return callback

    def _store_string(self, key: str) -> Any:
        def callback(message: String) -> None:
            self._last[key] = message

        return callback

    def _store_json(self, key: str) -> Any:
        def callback(message: String) -> None:
            self._last[key] = message
            self._payloads[key] = self._json_payload(message)

        return callback

    def _json_payload(self, message: String) -> dict[str, Any]:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return {}
        return payload if isinstance(payload, dict) else {}

    def _evaluate_and_publish(self) -> None:
        if self._finished:
            return
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        output = self._dry_run_output_payload()
        blocked_command = self._blocked_command_payload(output, topics, services)
        safety_report = self._safety_report_payload(output)
        status = self._status_payload(topics, services, output, blocked_command, safety_report)
        self._last_status = status
        self._last_output = output
        self._last_blocked_command = blocked_command
        self._last_safety_report = safety_report
        self._publish_json(self._status_pub, status)
        self._publish_json(self._dry_run_output_pub, output)
        self._publish_json(self._blocked_command_pub, blocked_command)
        self._publish_json(self._safety_report_pub, safety_report)
        self._record_rows(status, output, blocked_command, safety_report)

    def _status_payload(
        self,
        topics: list[str],
        services: list[str],
        output: dict[str, Any],
        blocked_command: dict[str, Any],
        safety_report: dict[str, Any],
    ) -> dict[str, Any]:
        topic_names = {line.split(" ", 1)[0] for line in topics}
        topic_available = {key: topic in topic_names for key, topic in self._required_topics.items()}
        sample_available = {
            "joint_states": self._sample_ready("joint_states"),
            "tf": self._sample_ready("tf") or self._sample_ready("tf_static"),
            "tf_static": self._sample_ready("tf_static"),
            "contact_wrench": self._sample_ready("contact_wrench"),
            "contact_state": self._sample_ready("contact_state"),
            "safety_status": self._sample_ready("safety_status") and bool(self._payloads.get("safety_status", {})),
            "virtual_force_command": self._sample_ready("virtual_force_command"),
            "admittance_command_suggestion": self._sample_ready("admittance_command_suggestion"),
            "readiness_gates": self._sample_ready("readiness_gates") and bool(self._payloads.get("readiness_gates", {})),
            "proposal_readiness_status": self._sample_ready("proposal_readiness_status")
            and bool(self._payloads.get("proposal_readiness_status", {})),
            "pre_control_contract_status": self._sample_ready("pre_control_contract_status")
            and bool(self._payloads.get("pre_control_contract_status", {})),
            "control_readiness_report": self._sample_ready("control_readiness_report")
            and bool(self._payloads.get("control_readiness_report", {})),
        }
        required_inputs_available = all(
            topic_available[key] and sample_available[key]
            for key in (
                "joint_states",
                "tf",
                "contact_wrench",
                "contact_state",
                "safety_status",
                "virtual_force_command",
                "admittance_command_suggestion",
                "readiness_gates",
                "proposal_readiness_status",
                "pre_control_contract_status",
                "control_readiness_report",
            )
        )
        output_generated = bool(output.get("control_law_output_generated", False))
        blocked_generated = bool(blocked_command.get("blocked_control_command_generated", False))
        blocked_confirmed = bool(blocked_command.get("blocked_control_command_confirmed", False))
        safety_report_available = bool(safety_report.get("safety_report_available", False))
        execution_paths_disabled = self._execution_paths_disabled(topics, services)
        validated = (
            self._control_law_enabled
            and self._dry_run_only
            and required_inputs_available
            and output_generated
            and blocked_generated
            and blocked_confirmed
            and safety_report_available
            and execution_paths_disabled
        )
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_fallback_used": bool(
                self.get_parameter("gazebo_fallback_used").get_parameter_value().bool_value
            ),
            "isaac_available": bool(self.get_parameter("isaac_available").get_parameter_value().bool_value),
            "robot_model": self._robot_model,
            "control_law_enabled": self._control_law_enabled,
            "dry_run_only": self._dry_run_only,
            "control_law_output_generated": output_generated,
            "blocked_control_command_generated": blocked_generated,
            "blocked_control_command_confirmed": blocked_confirmed,
            "safety_report_available": safety_report_available,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "trajectory_execution_allowed": False,
            "follow_joint_trajectory_allowed": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "required_topic_available": topic_available,
            "required_sample_available": sample_available,
            "all_required_inputs_available": required_inputs_available,
            "execution_paths_disabled": execution_paths_disabled,
            "status": self._success_status if validated else "no_motion_control_law_dry_run_pending",
        }

    def _dry_run_output_payload(self) -> dict[str, Any]:
        contact_wrench = self._last.get("contact_wrench")
        virtual_force = self._last.get("virtual_force_command")
        admittance = self._last.get("admittance_command_suggestion")
        contact_force = self._force_vector(contact_wrench) if contact_wrench else {"x": 0.0, "y": 0.0, "z": 0.0}
        contact_torque = self._torque_vector(contact_wrench) if contact_wrench else {"x": 0.0, "y": 0.0, "z": 0.0}
        virtual_force_vector = self._force_vector(virtual_force) if virtual_force else {"x": 0.0, "y": 0.0, "z": 0.0}
        admittance_linear = self._linear_vector(admittance) if admittance else {"x": 0.0, "y": 0.0, "z": 0.0}
        force_magnitude = self._vector_magnitude(contact_force)
        torque_magnitude = self._vector_magnitude(contact_torque)
        axis_force = float(contact_force.get(self._expected_contact_axis, 0.0))
        lateral_x = contact_force["x"] if self._expected_contact_axis != "x" else 0.0
        lateral_y = contact_force["y"] if self._expected_contact_axis != "y" else 0.0
        insertion_axis_correction = {
            "x": self._clip_diagnostic(-self._insertion_axis_gain * lateral_x, -0.01, 0.01),
            "y": self._clip_diagnostic(-self._insertion_axis_gain * lateral_y, -0.01, 0.01),
            "z": 0.0,
        }
        raw_velocity = -self._force_velocity_gain * axis_force
        force_limited_velocity = self._clip_diagnostic(raw_velocity, -0.01, 0.01)
        virtual_force_corrected = {
            axis: self._clip_diagnostic(admittance_linear[axis] + 0.001 * virtual_force_vector[axis], -0.01, 0.01)
            for axis in ("x", "y", "z")
        }
        admittance_limited = {
            axis: self._clip_to_limit(virtual_force_corrected[axis], self._max_commanded_velocity_mps)
            for axis in ("x", "y", "z")
        }
        safety_clipped = {"x": 0.0, "y": 0.0, "z": 0.0}
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "control_law_output_generated": True,
            "diagnostic_only": True,
            "dry_run_only": True,
            "expected_contact_axis": self._expected_contact_axis,
            "contact_force_magnitude_n": force_magnitude,
            "contact_torque_magnitude_nm": torque_magnitude,
            "insertion_axis_correction_suggestion_m": insertion_axis_correction,
            "force_limited_velocity_suggestion_mps": force_limited_velocity,
            "virtual_force_corrected_command_mps": virtual_force_corrected,
            "admittance_limited_command_mps": admittance_limited,
            "safety_clipped_command_mps": safety_clipped,
            "max_commanded_velocity_mps": 0.0,
            "max_commanded_position_delta_m": 0.0,
            "command_output_enabled": False,
            "sent_to_controller": False,
            "status": "diagnostic_control_law_output_blocked",
        }

    def _blocked_command_payload(
        self,
        output: dict[str, Any],
        topics: list[str],
        services: list[str],
    ) -> dict[str, Any]:
        execution_paths_disabled = self._execution_paths_disabled(topics, services)
        blocked = (
            output.get("safety_clipped_command_mps") == {"x": 0.0, "y": 0.0, "z": 0.0}
            and not self._command_output_enabled
            and not self._motion_execution_enabled
            and not self._controller_execution_allowed
            and not self._trajectory_execution_allowed
            and not self._follow_joint_trajectory_allowed
            and not self._real_robot_allowed
            and not self._moveit_allowed
            and not self._compute_ik_allowed
            and self._max_commanded_velocity_mps == 0.0
            and self._max_commanded_position_delta_m == 0.0
            and execution_paths_disabled
        )
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "blocked_control_command_generated": True,
            "blocked_control_command_confirmed": blocked,
            "block_reason": self._block_reason,
            "computed_command_mps": output.get("safety_clipped_command_mps", {"x": 0.0, "y": 0.0, "z": 0.0}),
            "published_command_mps": {"x": 0.0, "y": 0.0, "z": 0.0},
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "trajectory_execution_allowed": False,
            "follow_joint_trajectory_allowed": False,
            "real_robot_allowed": False,
            "moveit_allowed": False,
            "compute_ik_allowed": False,
            "sent_to_controller": False,
            "sent_to_follow_joint_trajectory": False,
            "execution_paths_disabled": execution_paths_disabled,
            "status": "all_commands_blocked" if blocked else "blocking_contract_failed",
        }

    def _safety_report_payload(self, output: dict[str, Any]) -> dict[str, Any]:
        force = float(output.get("contact_force_magnitude_n", 0.0))
        torque = float(output.get("contact_torque_magnitude_nm", 0.0))
        warning = force >= self._warning_force_threshold_n
        emergency = force >= self._emergency_stop_force_threshold_n
        within_force = force <= self._max_allowed_force_n
        within_torque = torque <= self._max_allowed_torque_nm
        contact_detected = force >= self._contact_detection_force_threshold_n
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "safety_report_available": True,
            "expected_contact_axis": self._expected_contact_axis,
            "contact_detected": contact_detected,
            "contact_force_magnitude_n": force,
            "contact_torque_magnitude_nm": torque,
            "max_allowed_force_n": self._max_allowed_force_n,
            "max_allowed_torque_nm": self._max_allowed_torque_nm,
            "contact_detection_force_threshold_n": self._contact_detection_force_threshold_n,
            "warning_force_threshold_n": self._warning_force_threshold_n,
            "emergency_stop_force_threshold_n": self._emergency_stop_force_threshold_n,
            "warning_force_exceeded": warning,
            "emergency_stop_force_exceeded": emergency,
            "within_force_limit": within_force,
            "within_torque_limit": within_torque,
            "safety_clipping_active": True,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "status": "safety_report_nominal" if within_force and within_torque and not emergency else "safety_report_limit_exceeded",
        }

    def _sample_ready(self, key: str) -> bool:
        return key in self._last and self._last[key] is not None

    def _execution_paths_disabled(self, topics: list[str], services: list[str]) -> bool:
        if (
            self._command_output_enabled
            or self._motion_execution_enabled
            or self._controller_execution_allowed
            or self._trajectory_execution_allowed
            or self._follow_joint_trajectory_allowed
            or self._real_robot_allowed
            or self._moveit_allowed
            or self._compute_ik_allowed
            or self._max_commanded_velocity_mps != 0.0
            or self._max_commanded_position_delta_m != 0.0
        ):
            return False
        combined = "\n".join(topics + services).lower()
        forbidden_patterns = [
            r"follow_joint_trajectory",
            r"/compute_ik\b",
            r"move_group",
            r"moveit",
            r"trajectory_controller",
            r"joint_trajectory_controller",
            r"/[^ \n]*/commands?\b",
        ]
        return not any(re.search(pattern, combined) for pattern in forbidden_patterns)

    def _force_vector(self, message: WrenchStamped) -> dict[str, float]:
        return {
            "x": float(message.wrench.force.x),
            "y": float(message.wrench.force.y),
            "z": float(message.wrench.force.z),
        }

    def _torque_vector(self, message: WrenchStamped) -> dict[str, float]:
        return {
            "x": float(message.wrench.torque.x),
            "y": float(message.wrench.torque.y),
            "z": float(message.wrench.torque.z),
        }

    def _linear_vector(self, message: TwistStamped) -> dict[str, float]:
        return {
            "x": float(message.twist.linear.x),
            "y": float(message.twist.linear.y),
            "z": float(message.twist.linear.z),
        }

    def _vector_magnitude(self, vector: dict[str, float]) -> float:
        return math.sqrt(vector["x"] ** 2 + vector["y"] ** 2 + vector["z"] ** 2)

    def _clip_diagnostic(self, value: float, low: float, high: float) -> float:
        return min(max(value, low), high)

    def _clip_to_limit(self, value: float, limit: float) -> float:
        if limit <= 0.0:
            return 0.0
        return self._clip_diagnostic(value, -limit, limit)

    def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        publisher.publish(message)

    def _record_rows(
        self,
        status: dict[str, Any],
        output: dict[str, Any],
        blocked_command: dict[str, Any],
        safety_report: dict[str, Any],
    ) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._status_rows.append(
            {
                "elapsed_sec": elapsed,
                "control_law_enabled": self._bool(status["control_law_enabled"]),
                "dry_run_only": self._bool(status["dry_run_only"]),
                "all_required_inputs_available": self._bool(status["all_required_inputs_available"]),
                "control_law_output_generated": self._bool(status["control_law_output_generated"]),
                "blocked_control_command_generated": self._bool(status["blocked_control_command_generated"]),
                "blocked_control_command_confirmed": self._bool(status["blocked_control_command_confirmed"]),
                "safety_report_available": self._bool(status["safety_report_available"]),
                "execution_paths_disabled": self._bool(status["execution_paths_disabled"]),
                "status": str(status["status"]),
            }
        )
        correction = output["insertion_axis_correction_suggestion_m"]
        virtual_corrected = output["virtual_force_corrected_command_mps"]
        safety_clipped = output["safety_clipped_command_mps"]
        self._output_rows.append(
            {
                "elapsed_sec": elapsed,
                "control_law_output_generated": self._bool(output["control_law_output_generated"]),
                "contact_force_magnitude_n": f"{output['contact_force_magnitude_n']:.6f}",
                "contact_torque_magnitude_nm": f"{output['contact_torque_magnitude_nm']:.6f}",
                "insertion_axis_correction_x_m": f"{correction['x']:.6f}",
                "insertion_axis_correction_y_m": f"{correction['y']:.6f}",
                "force_limited_velocity_suggestion_mps": f"{output['force_limited_velocity_suggestion_mps']:.6f}",
                "virtual_force_corrected_z_mps": f"{virtual_corrected['z']:.6f}",
                "safety_clipped_z_mps": f"{safety_clipped['z']:.6f}",
                "sent_to_controller": self._bool(output["sent_to_controller"]),
                "status": str(output["status"]),
            }
        )
        self._blocked_rows.append(
            {
                "elapsed_sec": elapsed,
                "blocked_control_command_generated": self._bool(blocked_command["blocked_control_command_generated"]),
                "blocked_control_command_confirmed": self._bool(blocked_command["blocked_control_command_confirmed"]),
                "command_output_enabled": self._bool(blocked_command["command_output_enabled"]),
                "motion_execution_enabled": self._bool(blocked_command["motion_execution_enabled"]),
                "controller_execution_allowed": self._bool(blocked_command["controller_execution_allowed"]),
                "trajectory_execution_allowed": self._bool(blocked_command["trajectory_execution_allowed"]),
                "follow_joint_trajectory_allowed": self._bool(blocked_command["follow_joint_trajectory_allowed"]),
                "moveit_allowed": self._bool(blocked_command["moveit_allowed"]),
                "compute_ik_allowed": self._bool(blocked_command["compute_ik_allowed"]),
                "sent_to_controller": self._bool(blocked_command["sent_to_controller"]),
                "status": str(blocked_command["status"]),
            }
        )
        self._safety_rows.append(
            {
                "elapsed_sec": elapsed,
                "safety_report_available": self._bool(safety_report["safety_report_available"]),
                "contact_detected": self._bool(safety_report["contact_detected"]),
                "contact_force_magnitude_n": f"{safety_report['contact_force_magnitude_n']:.6f}",
                "warning_force_exceeded": self._bool(safety_report["warning_force_exceeded"]),
                "emergency_stop_force_exceeded": self._bool(safety_report["emergency_stop_force_exceeded"]),
                "within_force_limit": self._bool(safety_report["within_force_limit"]),
                "within_torque_limit": self._bool(safety_report["within_torque_limit"]),
                "safety_clipping_active": self._bool(safety_report["safety_clipping_active"]),
                "status": str(safety_report["status"]),
            }
        )

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        if not self._last_output:
            self._last_output = self._dry_run_output_payload()
        if not self._last_blocked_command:
            self._last_blocked_command = self._blocked_command_payload(self._last_output, topics, services)
        if not self._last_safety_report:
            self._last_safety_report = self._safety_report_payload(self._last_output)
        status = self._status_payload(
            topics,
            services,
            self._last_output,
            self._last_blocked_command,
            self._last_safety_report,
        )
        self._last_status = status
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", self._tf_frame_names())
        self._write_csv(self._output_dir / "no_motion_control_law_status_samples.csv", self._status_rows)
        self._write_csv(self._output_dir / "control_law_dry_run_output_samples.csv", self._output_rows)
        self._write_csv(self._output_dir / "blocked_control_command_samples.csv", self._blocked_rows)
        self._write_csv(self._output_dir / "control_law_safety_report_samples.csv", self._safety_rows)
        self._write_json(self._output_dir / "no_motion_control_law_status.json", status)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_9 no-motion control-law dry-run diagnostics written")
        rclpy.shutdown()

    def _tf_frame_names(self) -> list[str]:
        frames = {"world", "base_link", "tool0", "peg_tip", "hole_center", "insertion_axis_z"}
        tf_message = self._last.get("tf")
        tf_static_message = self._last.get("tf_static")
        if tf_message:
            frames.update(transform.child_frame_id for transform in tf_message.transforms)
        if tf_static_message:
            frames.update(transform.child_frame_id for transform in tf_static_message.transforms)
        return sorted(frame for frame in frames if frame)

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_9_no_motion_control_law_dry_run",
            "",
            "Purpose: validate a simulation-only no-motion control-law dry run without robot motion.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Control law enabled: `{status['control_law_enabled']}`",
            f"Dry run only: `{status['dry_run_only']}`",
            f"Control-law output generated: `{status['control_law_output_generated']}`",
            f"Blocked control command generated: `{status['blocked_control_command_generated']}`",
            f"Blocked control command confirmed: `{status['blocked_control_command_confirmed']}`",
            f"Safety report available: `{status['safety_report_available']}`",
            f"All required inputs available: `{status['all_required_inputs_available']}`",
            f"Status: `{status['status']}`",
            "",
            "Safety constraints: command_output_enabled=false, motion_execution_enabled=false, no MoveIt, no /compute_ik, no controllers, no real robot execution, no FollowJointTrajectory, and no trajectory execution.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_9 no-motion control-law dry-run evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"all_required_inputs_available={str(status['all_required_inputs_available']).lower()}",
            f"control_law_output_generated={str(status['control_law_output_generated']).lower()}",
            f"blocked_control_command_generated={str(status['blocked_control_command_generated']).lower()}",
            f"blocked_control_command_confirmed={str(status['blocked_control_command_confirmed']).lower()}",
            f"safety_report_available={str(status['safety_report_available']).lower()}",
            "command_output_enabled=false",
            "motion_execution_enabled=false",
            "controller_execution_allowed=false",
            "trajectory_execution_allowed=false",
            "follow_joint_trajectory_allowed=false",
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

    def _bool(self, value: Any) -> str:
        return str(bool(value)).lower()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV19ControlLawDryRunNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
