"""Simulation-only control-development scaffold for proposal_simulation_cell_v1_8."""

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
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class ProposalSimulationCellV18ControlScaffoldNode(Node):
    """Prepare diagnostic control inputs and blocked proposals without execution."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_8_control_scaffold_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_8")
        self.declare_parameter("isaac_available", False)
        self.declare_parameter("gazebo_fallback_used", True)
        self.declare_parameter("ros_gz_bridge_available", False)
        self.declare_parameter("ros_gz_image_available", False)

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_8")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        robot = self._config.get("robot", {})
        control_inputs = self._config.get("control_inputs", {})
        command_proposal = self._config.get("command_proposal", {})
        command_blocker = self._config.get("command_blocker", {})
        safety_gates = self._config.get("safety_gates", {})
        control_boundary = self._config.get("control_boundary", {})
        topics = control_inputs.get("topics", {})

        self._robot_model = str(robot.get("model", "KUKA LBR iisy 6 R1300"))
        self._required_topics = {
            "joint_states": str(topics.get("joint_states", robot.get("joint_state_topic", "/joint_states"))),
            "tf": str(topics.get("tf", robot.get("tf_topic", "/tf"))),
            "tf_static": str(topics.get("tf_static", robot.get("tf_static_topic", "/tf_static"))),
            "rgb_image": str(topics.get("rgb_image", "/proposal_simulation_cell/d405/color/image_raw")),
            "depth_image": str(topics.get("depth_image", "/proposal_simulation_cell/d405/depth/image_rect_raw")),
            "contact_wrench": str(topics.get("contact_wrench", "/proposal_simulation_cell/contact_wrench")),
            "contact_state": str(topics.get("contact_state", "/proposal_simulation_cell/contact_state")),
            "safety_status": str(topics.get("safety_status", "/proposal_simulation_cell/safety_status")),
            "virtual_force_command": str(
                topics.get("virtual_force_command", "/proposal_simulation_cell/virtual_force_command")
            ),
            "admittance_command_suggestion": str(
                topics.get(
                    "admittance_command_suggestion",
                    "/proposal_simulation_cell/admittance_command_suggestion",
                )
            ),
            "readiness_gates": str(
                topics.get(
                    "readiness_gates",
                    safety_gates.get("readiness_gates_topic", "/proposal_simulation_cell/readiness_gates"),
                )
            ),
            "proposal_readiness_status": str(
                topics.get(
                    "proposal_readiness_status",
                    safety_gates.get(
                        "proposal_readiness_status_topic",
                        "/proposal_simulation_cell/proposal_readiness_status",
                    ),
                )
            ),
            "safety_gate_report": str(
                topics.get(
                    "safety_gate_report",
                    safety_gates.get("safety_gate_report_topic", "/proposal_simulation_cell/safety_gate_report"),
                )
            ),
            "pre_control_contract_status": str(
                topics.get(
                    "pre_control_contract_status",
                    control_boundary.get(
                        "pre_control_contract_topic",
                        "/proposal_simulation_cell/pre_control_contract_status",
                    ),
                )
            ),
            "controller_boundary_report": str(
                topics.get(
                    "controller_boundary_report",
                    control_boundary.get(
                        "controller_boundary_report_topic",
                        "/proposal_simulation_cell/controller_boundary_report",
                    ),
                )
            ),
            "task_phase": str(topics.get("task_phase", robot.get("task_phase_topic", "/proposal_simulation_cell/task_phase"))),
        }
        self._required_flags = {
            "joint_states": bool(control_inputs.get("require_joint_states", True)),
            "tf": bool(control_inputs.get("require_tf", True)),
            "rgb_image": bool(control_inputs.get("require_rgb_image", True)),
            "depth_image": bool(control_inputs.get("require_depth_image", True)),
            "contact_wrench": bool(control_inputs.get("require_contact_wrench", True)),
            "contact_state": bool(control_inputs.get("require_contact_state", True)),
            "safety_status": bool(control_inputs.get("require_safety_status", True)),
            "readiness_gates": bool(control_inputs.get("require_readiness_gates", True)),
            "pre_control_contract_status": bool(control_inputs.get("require_pre_control_contract", True)),
            "controller_boundary_report": bool(control_inputs.get("require_controller_boundary_report", True)),
        }

        self._control_input_status_topic = str(
            control_boundary.get("status_topic", "/proposal_simulation_cell/control_input_status")
        )
        self._command_proposal_topic = str(
            command_proposal.get("topic", "/proposal_simulation_cell/control_command_proposal")
        )
        self._command_blocker_topic = str(
            command_blocker.get("topic", "/proposal_simulation_cell/command_blocker_status")
        )
        self._readiness_report_topic = str(
            control_boundary.get("readiness_report_topic", "/proposal_simulation_cell/control_readiness_report")
        )

        self._command_output_enabled = bool(control_boundary.get("command_output_enabled", False))
        self._motion_execution_enabled = bool(control_boundary.get("motion_execution_enabled", False))
        self._controller_execution_allowed = bool(control_boundary.get("controller_execution_allowed", False))
        self._trajectory_execution_allowed = bool(control_boundary.get("trajectory_execution_allowed", False))
        self._real_robot_allowed = bool(control_boundary.get("real_robot_allowed", False))
        self._moveit_allowed = bool(control_boundary.get("moveit_allowed", False))
        self._compute_ik_allowed = bool(control_boundary.get("compute_ik_allowed", False))
        self._follow_joint_trajectory_allowed = bool(
            control_boundary.get("follow_joint_trajectory_allowed", False)
        )
        self._max_commanded_velocity_mps = float(
            control_boundary.get(
                "max_commanded_velocity_mps",
                command_proposal.get("max_commanded_velocity_mps", 0.0),
            )
        )
        self._max_commanded_position_delta_m = float(
            control_boundary.get(
                "max_commanded_position_delta_m",
                command_proposal.get("max_commanded_position_delta_m", 0.0),
            )
        )
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 20.0))
        self._success_status = str(validation.get("status_success", "control_development_scaffold_validated"))
        self._simulation_engine = str(validation.get("simulation_engine", "gazebo"))

        self._start_time = time.monotonic()
        self._finished = False
        self._last: dict[str, Any] = {}
        self._payloads: dict[str, dict[str, Any]] = {}
        self._last_status: dict[str, Any] = {}
        self._last_proposal: dict[str, Any] = {}
        self._last_blocker: dict[str, Any] = {}
        self._last_report: dict[str, Any] = {}
        self._input_rows: list[dict[str, str]] = []
        self._proposal_rows: list[dict[str, str]] = []
        self._blocker_rows: list[dict[str, str]] = []
        self._report_rows: list[dict[str, str]] = []

        self._input_status_pub = self.create_publisher(String, self._control_input_status_topic, 10)
        self._proposal_pub = self.create_publisher(String, self._command_proposal_topic, 10)
        self._blocker_pub = self.create_publisher(String, self._command_blocker_topic, 10)
        self._readiness_report_pub = self.create_publisher(String, self._readiness_report_topic, 10)

        self.create_subscription(JointState, self._required_topics["joint_states"], self._store("joint_states"), 10)
        self.create_subscription(TFMessage, self._required_topics["tf"], self._store("tf"), 10)
        self.create_subscription(TFMessage, self._required_topics["tf_static"], self._store("tf_static"), 10)
        self.create_subscription(Image, self._required_topics["rgb_image"], self._store("rgb_image"), 10)
        self.create_subscription(Image, self._required_topics["depth_image"], self._store("depth_image"), 10)
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
            self._required_topics["safety_gate_report"],
            self._store_json("safety_gate_report"),
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
            self._required_topics["controller_boundary_report"],
            self._store_json("controller_boundary_report"),
            10,
        )
        self.create_subscription(String, self._required_topics["task_phase"], self._store_string("task_phase"), 10)

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_8 control scaffold node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.8 config not found: {path}")
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
        status = self._status(topics, services)
        proposal = self._proposal_payload(status)
        blocker = self._blocker_payload(status, topics, services)
        report = self._readiness_report_payload(status, proposal, blocker)
        self._last_status = status
        self._last_proposal = proposal
        self._last_blocker = blocker
        self._last_report = report
        self._publish_json(self._input_status_pub, status)
        self._publish_json(self._proposal_pub, proposal)
        self._publish_json(self._blocker_pub, blocker)
        self._publish_json(self._readiness_report_pub, report)
        self._record_rows(status, proposal, blocker, report)

    def _status(self, topics: list[str], services: list[str]) -> dict[str, Any]:
        topic_names = {line.split(" ", 1)[0] for line in topics}
        sample_available = {
            "joint_states": self._sample_ready("joint_states"),
            "tf": self._sample_ready("tf") or self._sample_ready("tf_static"),
            "tf_static": self._required_topics["tf_static"] in topic_names and self._sample_ready("tf_static"),
            "rgb_image": self._sample_ready("rgb_image"),
            "depth_image": self._sample_ready("depth_image"),
            "contact_wrench": self._sample_ready("contact_wrench"),
            "contact_state": self._sample_ready("contact_state"),
            "safety_status": self._sample_ready("safety_status") and bool(self._payloads.get("safety_status", {})),
            "virtual_force_command": self._sample_ready("virtual_force_command"),
            "admittance_command_suggestion": self._sample_ready("admittance_command_suggestion"),
            "readiness_gates": self._sample_ready("readiness_gates") and bool(self._payloads.get("readiness_gates", {})),
            "proposal_readiness_status": self._sample_ready("proposal_readiness_status")
            and bool(self._payloads.get("proposal_readiness_status", {})),
            "safety_gate_report": self._sample_ready("safety_gate_report") and bool(self._payloads.get("safety_gate_report", {})),
            "pre_control_contract_status": self._sample_ready("pre_control_contract_status")
            and bool(self._payloads.get("pre_control_contract_status", {})),
            "controller_boundary_report": self._sample_ready("controller_boundary_report")
            and bool(self._payloads.get("controller_boundary_report", {})),
            "task_phase": self._sample_ready("task_phase") or self._required_topics["task_phase"] in topic_names,
        }
        topic_available = {key: topic in topic_names for key, topic in self._required_topics.items()}
        required_results = {
            "joint_states": topic_available["joint_states"] and sample_available["joint_states"],
            "tf": topic_available["tf"] and sample_available["tf"],
            "rgb_image": topic_available["rgb_image"] and sample_available["rgb_image"],
            "depth_image": topic_available["depth_image"] and sample_available["depth_image"],
            "contact_wrench": topic_available["contact_wrench"] and sample_available["contact_wrench"],
            "contact_state": topic_available["contact_state"] and sample_available["contact_state"],
            "safety_status": topic_available["safety_status"] and sample_available["safety_status"],
            "readiness_gates": topic_available["readiness_gates"] and sample_available["readiness_gates"],
            "pre_control_contract_status": topic_available["pre_control_contract_status"]
            and sample_available["pre_control_contract_status"],
            "controller_boundary_report": topic_available["controller_boundary_report"]
            and sample_available["controller_boundary_report"],
        }
        all_required_inputs_available = all(
            required_results[key] if self._required_flags[key] else True
            for key in self._required_flags
        )
        pre_control_contract_available = required_results["pre_control_contract_status"]
        controller_boundary_report_available = required_results["controller_boundary_report"]
        readiness_payload = self._payloads.get("readiness_gates", {})
        proposal_readiness_payload = self._payloads.get("proposal_readiness_status", {})
        pre_control_payload = self._payloads.get("pre_control_contract_status", {})
        controller_boundary_payload = self._payloads.get("controller_boundary_report", {})
        safety_gate_checker_available = bool(
            readiness_payload.get("safety_gate", False)
            or readiness_payload.get("safety_gate_passed", False)
            or proposal_readiness_payload.get("safety_gate_passed", False)
        )
        control_boundary_checker_available = bool(
            controller_boundary_report_available
            and (
                controller_boundary_payload.get("controller_execution_forbidden", False)
                or pre_control_payload.get("future_controller_boundary_contract_passed", False)
            )
        )
        execution_paths_disabled = self._execution_paths_disabled(topics, services)
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_fallback_used": bool(
                self.get_parameter("gazebo_fallback_used").get_parameter_value().bool_value
            ),
            "isaac_available": bool(self.get_parameter("isaac_available").get_parameter_value().bool_value),
            "robot_model": self._robot_model,
            "control_input_monitor_available": True,
            "control_command_proposal_available": True,
            "command_blocker_available": True,
            "safety_gate_checker_available": safety_gate_checker_available,
            "control_boundary_checker_available": control_boundary_checker_available,
            "control_readiness_report_available": True,
            "all_required_inputs_available": all_required_inputs_available,
            "pre_control_contract_available": pre_control_contract_available,
            "controller_boundary_report_available": controller_boundary_report_available,
            "required_topic_available": topic_available,
            "required_sample_available": sample_available,
            "required_input_results": required_results,
            "command_proposal_generated": True,
            "command_proposal_blocked": True,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "trajectory_execution_allowed": False,
            "follow_joint_trajectory_allowed": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "execution_paths_disabled": execution_paths_disabled,
            "max_commanded_velocity_mps": 0.0,
            "max_commanded_position_delta_m": 0.0,
            "status": self._success_status
            if (
                all_required_inputs_available
                and pre_control_contract_available
                and controller_boundary_report_available
                and safety_gate_checker_available
                and control_boundary_checker_available
                and execution_paths_disabled
            )
            else "control_development_scaffold_pending",
        }

    def _sample_ready(self, key: str) -> bool:
        return key in self._last and self._last[key] is not None

    def _execution_paths_disabled(self, topics: list[str], services: list[str]) -> bool:
        if (
            self._command_output_enabled
            or self._motion_execution_enabled
            or self._controller_execution_allowed
            or self._trajectory_execution_allowed
            or self._real_robot_allowed
            or self._moveit_allowed
            or self._compute_ik_allowed
            or self._follow_joint_trajectory_allowed
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

    def _proposal_payload(self, status: dict[str, Any]) -> dict[str, Any]:
        virtual_force = self._last.get("virtual_force_command")
        admittance = self._last.get("admittance_command_suggestion")
        force_mag = self._wrench_force_magnitude(virtual_force) if virtual_force else 0.0
        admittance_speed = self._twist_linear_magnitude(admittance) if admittance else 0.0
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "proposal_source": "virtual_force_and_admittance_diagnostics",
            "diagnostic_only": True,
            "proposal_generated": True,
            "proposal_blocked": True,
            "block_reason": "command_output_disabled_by_control_development_scaffold",
            "virtual_force_magnitude_n": force_mag,
            "admittance_linear_speed_mps": admittance_speed,
            "command_output_enabled": False,
            "max_commanded_velocity_mps": 0.0,
            "max_commanded_position_delta_m": 0.0,
            "sent_to_controller": False,
            "status": "blocked_diagnostic_proposal"
            if status["all_required_inputs_available"]
            else "blocked_waiting_for_inputs",
        }

    def _blocker_payload(self, status: dict[str, Any], topics: list[str], services: list[str]) -> dict[str, Any]:
        execution_paths_disabled = self._execution_paths_disabled(topics, services)
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "trajectory_execution_allowed": False,
            "follow_joint_trajectory_allowed": False,
            "real_robot_allowed": False,
            "moveit_allowed": False,
            "compute_ik_allowed": False,
            "command_proposal_blocked": True,
            "execution_paths_disabled": execution_paths_disabled,
            "status": "blocking_all_command_outputs" if execution_paths_disabled else "forbidden_interface_detected",
        }

    def _readiness_report_payload(
        self,
        status: dict[str, Any],
        proposal: dict[str, Any],
        blocker: dict[str, Any],
    ) -> dict[str, Any]:
        ready = (
            status["all_required_inputs_available"]
            and status["pre_control_contract_available"]
            and status["controller_boundary_report_available"]
            and bool(proposal["proposal_blocked"])
            and bool(blocker["execution_paths_disabled"])
        )
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "control_input_monitor_available": True,
            "control_command_proposal_available": True,
            "command_blocker_available": True,
            "safety_gate_checker_available": status["safety_gate_checker_available"],
            "control_boundary_checker_available": status["control_boundary_checker_available"],
            "control_readiness_report_available": True,
            "all_required_inputs_available": status["all_required_inputs_available"],
            "command_proposal_generated": True,
            "command_proposal_blocked": True,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "trajectory_execution_allowed": False,
            "follow_joint_trajectory_allowed": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "ready_for_future_controller_development": ready,
            "status": status["status"] if ready else "control_development_scaffold_pending",
        }

    def _wrench_force_magnitude(self, message: WrenchStamped) -> float:
        return math.sqrt(
            message.wrench.force.x**2 + message.wrench.force.y**2 + message.wrench.force.z**2
        )

    def _twist_linear_magnitude(self, message: TwistStamped) -> float:
        return math.sqrt(
            message.twist.linear.x**2 + message.twist.linear.y**2 + message.twist.linear.z**2
        )

    def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        publisher.publish(message)

    def _record_rows(
        self,
        status: dict[str, Any],
        proposal: dict[str, Any],
        blocker: dict[str, Any],
        report: dict[str, Any],
    ) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._input_rows.append(
            {
                "elapsed_sec": elapsed,
                "joint_states": self._bool(status["required_input_results"]["joint_states"]),
                "tf": self._bool(status["required_input_results"]["tf"]),
                "rgb_image": self._bool(status["required_input_results"]["rgb_image"]),
                "depth_image": self._bool(status["required_input_results"]["depth_image"]),
                "contact_wrench": self._bool(status["required_input_results"]["contact_wrench"]),
                "contact_state": self._bool(status["required_input_results"]["contact_state"]),
                "safety_status": self._bool(status["required_input_results"]["safety_status"]),
                "readiness_gates": self._bool(status["required_input_results"]["readiness_gates"]),
                "pre_control_contract": self._bool(status["pre_control_contract_available"]),
                "controller_boundary_report": self._bool(status["controller_boundary_report_available"]),
                "all_required_inputs_available": self._bool(status["all_required_inputs_available"]),
                "status": str(status["status"]),
            }
        )
        self._proposal_rows.append(
            {
                "elapsed_sec": elapsed,
                "proposal_generated": self._bool(proposal["proposal_generated"]),
                "proposal_blocked": self._bool(proposal["proposal_blocked"]),
                "command_output_enabled": self._bool(proposal["command_output_enabled"]),
                "virtual_force_magnitude_n": f"{proposal['virtual_force_magnitude_n']:.6f}",
                "admittance_linear_speed_mps": f"{proposal['admittance_linear_speed_mps']:.6f}",
                "sent_to_controller": self._bool(proposal["sent_to_controller"]),
                "status": str(proposal["status"]),
            }
        )
        self._blocker_rows.append(
            {
                "elapsed_sec": elapsed,
                "command_output_enabled": self._bool(blocker["command_output_enabled"]),
                "motion_execution_enabled": self._bool(blocker["motion_execution_enabled"]),
                "controller_execution_allowed": self._bool(blocker["controller_execution_allowed"]),
                "trajectory_execution_allowed": self._bool(blocker["trajectory_execution_allowed"]),
                "follow_joint_trajectory_allowed": self._bool(blocker["follow_joint_trajectory_allowed"]),
                "real_robot_allowed": self._bool(blocker["real_robot_allowed"]),
                "moveit_allowed": self._bool(blocker["moveit_allowed"]),
                "compute_ik_allowed": self._bool(blocker["compute_ik_allowed"]),
                "command_proposal_blocked": self._bool(blocker["command_proposal_blocked"]),
                "execution_paths_disabled": self._bool(blocker["execution_paths_disabled"]),
                "status": str(blocker["status"]),
            }
        )
        self._report_rows.append(
            {
                "elapsed_sec": elapsed,
                "control_input_monitor_available": self._bool(report["control_input_monitor_available"]),
                "control_command_proposal_available": self._bool(report["control_command_proposal_available"]),
                "command_blocker_available": self._bool(report["command_blocker_available"]),
                "safety_gate_checker_available": self._bool(report["safety_gate_checker_available"]),
                "control_boundary_checker_available": self._bool(report["control_boundary_checker_available"]),
                "control_readiness_report_available": self._bool(report["control_readiness_report_available"]),
                "command_proposal_blocked": self._bool(report["command_proposal_blocked"]),
                "ready_for_future_controller_development": self._bool(
                    report["ready_for_future_controller_development"]
                ),
                "status": str(report["status"]),
            }
        )

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        status = self._status(topics, services)
        self._last_status = status
        if not self._last_proposal:
            self._last_proposal = self._proposal_payload(status)
        if not self._last_blocker:
            self._last_blocker = self._blocker_payload(status, topics, services)
        if not self._last_report:
            self._last_report = self._readiness_report_payload(status, self._last_proposal, self._last_blocker)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", self._tf_frame_names())
        self._write_csv(self._output_dir / "control_input_status_samples.csv", self._input_rows)
        self._write_csv(self._output_dir / "control_command_proposal_samples.csv", self._proposal_rows)
        self._write_csv(self._output_dir / "command_blocker_status_samples.csv", self._blocker_rows)
        self._write_csv(self._output_dir / "control_readiness_report_samples.csv", self._report_rows)
        self._write_json(self._output_dir / "control_development_scaffold_status.json", status)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_8 control scaffold diagnostics written")
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
            "# proposal_simulation_cell_v1_8_control_development_scaffold",
            "",
            "Purpose: validate a simulation-only control-development scaffold without robot motion.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Control input monitor available: `{status['control_input_monitor_available']}`",
            f"Control command proposal available: `{status['control_command_proposal_available']}`",
            f"Command blocker available: `{status['command_blocker_available']}`",
            f"Safety gate checker available: `{status['safety_gate_checker_available']}`",
            f"Control boundary checker available: `{status['control_boundary_checker_available']}`",
            f"Control readiness report available: `{status['control_readiness_report_available']}`",
            f"All required inputs available: `{status['all_required_inputs_available']}`",
            f"Command proposal blocked: `{status['command_proposal_blocked']}`",
            f"Status: `{status['status']}`",
            "",
            "Safety constraints: command_output_enabled=false, motion_execution_enabled=false, no MoveIt, no /compute_ik, no controllers, no real robot execution, no FollowJointTrajectory, and no trajectory execution.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_8 control development scaffold evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"all_required_inputs_available={str(status['all_required_inputs_available']).lower()}",
            f"pre_control_contract_available={str(status['pre_control_contract_available']).lower()}",
            f"controller_boundary_report_available={str(status['controller_boundary_report_available']).lower()}",
            f"command_proposal_generated={str(status['command_proposal_generated']).lower()}",
            f"command_proposal_blocked={str(status['command_proposal_blocked']).lower()}",
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
    node = ProposalSimulationCellV18ControlScaffoldNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
