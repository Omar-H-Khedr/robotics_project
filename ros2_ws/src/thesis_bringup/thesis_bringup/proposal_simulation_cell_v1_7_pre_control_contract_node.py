"""Simulation-only pre-control contract for proposal_simulation_cell_v1_7."""

from __future__ import annotations

import csv
import json
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


class ProposalSimulationCellV17PreControlContractNode(Node):
    """Validate a diagnostic-only interface boundary without robot execution."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_7_pre_control_contract_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_7")
        self.declare_parameter("isaac_available", False)
        self.declare_parameter("gazebo_fallback_used", True)
        self.declare_parameter("ros_gz_bridge_available", False)
        self.declare_parameter("ros_gz_image_available", False)

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_7")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        robot = self._config.get("robot", {})
        required_inputs = self._config.get("required_inputs", {})
        required_topics = required_inputs.get("topics", {})
        allowed_outputs = self._config.get("allowed_outputs", {})
        forbidden_outputs = self._config.get("forbidden_outputs", {})
        safety_constraints = self._config.get("safety_constraints", {})
        readiness = self._config.get("readiness_dependencies", {})
        future_boundary = self._config.get("future_controller_boundary", {})

        self._robot_model = str(robot.get("model", "KUKA LBR iisy 6 R1300"))
        self._required_topics = {
            "joint_states": str(required_topics.get("joint_states", robot.get("joint_state_topic", "/joint_states"))),
            "tf": str(required_topics.get("tf", robot.get("tf_topic", "/tf"))),
            "tf_static": str(required_topics.get("tf_static", robot.get("tf_static_topic", "/tf_static"))),
            "rgb_image": str(
                required_topics.get("rgb_image", "/proposal_simulation_cell/d405/color/image_raw")
            ),
            "depth_image": str(
                required_topics.get("depth_image", "/proposal_simulation_cell/d405/depth/image_rect_raw")
            ),
            "contact_wrench": str(
                required_topics.get("contact_wrench", "/proposal_simulation_cell/contact_wrench")
            ),
            "contact_state": str(
                required_topics.get("contact_state", "/proposal_simulation_cell/contact_state")
            ),
            "safety_status": str(
                required_topics.get("safety_status", "/proposal_simulation_cell/safety_status")
            ),
            "virtual_force_command": str(
                required_topics.get("virtual_force_command", "/proposal_simulation_cell/virtual_force_command")
            ),
            "admittance_command_suggestion": str(
                required_topics.get(
                    "admittance_command_suggestion",
                    "/proposal_simulation_cell/admittance_command_suggestion",
                )
            ),
            "readiness_gates": str(
                required_topics.get(
                    "readiness_gates",
                    readiness.get("readiness_gates_topic", "/proposal_simulation_cell/readiness_gates"),
                )
            ),
            "proposal_readiness_status": str(
                required_topics.get(
                    "proposal_readiness_status",
                    readiness.get(
                        "proposal_readiness_status_topic",
                        "/proposal_simulation_cell/proposal_readiness_status",
                    ),
                )
            ),
            "safety_gate_report": str(
                required_topics.get(
                    "safety_gate_report",
                    readiness.get("safety_gate_report_topic", "/proposal_simulation_cell/safety_gate_report"),
                )
            ),
            "task_phase": str(
                required_topics.get("task_phase", robot.get("task_phase_topic", "/proposal_simulation_cell/task_phase"))
            ),
        }
        self._required_flags = {
            "require_rgb_image": bool(required_inputs.get("require_rgb_image", True)),
            "require_depth_image": bool(required_inputs.get("require_depth_image", True)),
            "require_joint_states": bool(required_inputs.get("require_joint_states", True)),
            "require_tf": bool(required_inputs.get("require_tf", True)),
            "require_contact_wrench": bool(required_inputs.get("require_contact_wrench", True)),
            "require_contact_state": bool(required_inputs.get("require_contact_state", True)),
            "require_safety_status": bool(required_inputs.get("require_safety_status", True)),
            "require_virtual_force_interface": bool(
                required_inputs.get("require_virtual_force_interface", True)
            ),
            "require_admittance_interface": bool(required_inputs.get("require_admittance_interface", True)),
            "require_readiness_gates": bool(required_inputs.get("require_readiness_gates", True)),
            "require_proposal_readiness_status": bool(
                required_inputs.get("require_proposal_readiness_status", True)
            ),
        }
        self._allowed_output_topics = [
            str(topic) for topic in allowed_outputs.get("topics", []) if str(topic)
        ]
        self._status_topic = "/proposal_simulation_cell/pre_control_contract_status"
        self._boundary_topic = "/proposal_simulation_cell/controller_boundary_report"
        if self._status_topic not in self._allowed_output_topics:
            self._allowed_output_topics.append(self._status_topic)
        if self._boundary_topic not in self._allowed_output_topics:
            self._allowed_output_topics.append(self._boundary_topic)

        self._forbidden_interfaces = [
            str(item) for item in forbidden_outputs.get("interfaces", []) if str(item)
        ]
        self._safety_constraints = safety_constraints
        self._readiness_dependencies = readiness
        self._future_boundary = future_boundary
        self._command_output_enabled = bool(
            forbidden_outputs.get(
                "command_output_enabled",
                safety_constraints.get("command_output_enabled", False),
            )
        )
        self._motion_execution_enabled = bool(
            forbidden_outputs.get(
                "motion_execution_enabled",
                safety_constraints.get("motion_execution_enabled", False),
            )
        )
        self._controller_execution_allowed = bool(
            forbidden_outputs.get(
                "controller_execution_allowed",
                safety_constraints.get("controller_execution_allowed", False),
            )
        )
        self._trajectory_execution_allowed = bool(
            forbidden_outputs.get(
                "trajectory_execution_allowed",
                safety_constraints.get("trajectory_execution_allowed", False),
            )
        )
        self._real_robot_allowed = bool(
            forbidden_outputs.get("real_robot_allowed", safety_constraints.get("real_robot_allowed", False))
        )
        self._moveit_allowed = bool(
            forbidden_outputs.get("moveit_allowed", safety_constraints.get("moveit_allowed", False))
        )
        self._compute_ik_allowed = bool(
            forbidden_outputs.get("compute_ik_allowed", safety_constraints.get("compute_ik_allowed", False))
        )
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 18.0))
        self._success_status = str(validation.get("status_success", "pre_control_contract_validated"))
        self._simulation_engine = str(validation.get("simulation_engine", "gazebo"))

        self._start_time = time.monotonic()
        self._finished = False
        self._last: dict[str, Any] = {}
        self._last_readiness_payload: dict[str, Any] = {}
        self._last_proposal_payload: dict[str, Any] = {}
        self._last_safety_report_payload: dict[str, Any] = {}
        self._status_rows: list[dict[str, str]] = []
        self._boundary_rows: list[dict[str, str]] = []
        self._last_status: dict[str, Any] = {}

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._boundary_pub = self.create_publisher(String, self._boundary_topic, 10)

        self.create_subscription(Image, self._required_topics["rgb_image"], self._store("rgb_image"), 10)
        self.create_subscription(Image, self._required_topics["depth_image"], self._store("depth_image"), 10)
        self.create_subscription(JointState, self._required_topics["joint_states"], self._store("joint_states"), 10)
        self.create_subscription(TFMessage, self._required_topics["tf"], self._store("tf"), 10)
        self.create_subscription(TFMessage, self._required_topics["tf_static"], self._store("tf_static"), 10)
        self.create_subscription(WrenchStamped, self._required_topics["contact_wrench"], self._store("contact_wrench"), 10)
        self.create_subscription(String, self._required_topics["contact_state"], self._store("contact_state"), 10)
        self.create_subscription(String, self._required_topics["safety_status"], self._store("safety_status"), 10)
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
        self.create_subscription(String, self._required_topics["readiness_gates"], self._on_readiness_gates, 10)
        self.create_subscription(
            String,
            self._required_topics["proposal_readiness_status"],
            self._on_proposal_readiness_status,
            10,
        )
        self.create_subscription(
            String,
            self._required_topics["safety_gate_report"],
            self._on_safety_gate_report,
            10,
        )
        self.create_subscription(String, self._required_topics["task_phase"], self._store("task_phase"), 10)

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_7 pre-control contract node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.7 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _store(self, key: str) -> Any:
        def callback(message: Any) -> None:
            self._last[key] = message

        return callback

    def _on_readiness_gates(self, message: String) -> None:
        self._last["readiness_gates"] = message
        self._last_readiness_payload = self._json_payload(message)

    def _on_proposal_readiness_status(self, message: String) -> None:
        self._last["proposal_readiness_status"] = message
        self._last_proposal_payload = self._json_payload(message)

    def _on_safety_gate_report(self, message: String) -> None:
        self._last["safety_gate_report"] = message
        self._last_safety_report_payload = self._json_payload(message)

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
        self._last_status = status
        boundary = self._boundary_payload(status)
        self._publish_json(self._status_pub, status)
        self._publish_json(self._boundary_pub, boundary)
        self._record_rows(status, boundary)

    def _status(self, topics: list[str], services: list[str]) -> dict[str, Any]:
        topic_names = {line.split(" ", 1)[0] for line in topics}
        service_names = {line.split(" ", 1)[0] for line in services}
        required_input_samples = {
            "rgb_image": self._sample_ready("rgb_image"),
            "depth_image": self._sample_ready("depth_image"),
            "joint_states": self._sample_ready("joint_states"),
            "tf": self._sample_ready("tf") or self._sample_ready("tf_static"),
            "tf_static": self._required_topics["tf_static"] in topic_names and self._sample_ready("tf_static"),
            "contact_wrench": self._sample_ready("contact_wrench"),
            "contact_state": self._sample_ready("contact_state"),
            "safety_status": self._sample_ready("safety_status"),
            "virtual_force_command": self._sample_ready("virtual_force_command"),
            "admittance_command_suggestion": self._sample_ready("admittance_command_suggestion"),
            "readiness_gates": self._sample_ready("readiness_gates"),
            "proposal_readiness_status": self._sample_ready("proposal_readiness_status"),
            "safety_gate_report": self._sample_ready("safety_gate_report"),
            "task_phase": self._required_topics["task_phase"] in topic_names,
        }
        required_topic_available = {
            key: topic in topic_names
            for key, topic in self._required_topics.items()
        }
        required_input_results = {
            "rgb_image": required_topic_available["rgb_image"] and required_input_samples["rgb_image"],
            "depth_image": required_topic_available["depth_image"] and required_input_samples["depth_image"],
            "joint_states": required_topic_available["joint_states"] and required_input_samples["joint_states"],
            "tf": required_topic_available["tf"] and required_input_samples["tf"],
            "contact_wrench": required_topic_available["contact_wrench"] and required_input_samples["contact_wrench"],
            "contact_state": required_topic_available["contact_state"] and required_input_samples["contact_state"],
            "safety_status": required_topic_available["safety_status"] and required_input_samples["safety_status"],
            "virtual_force_command": (
                required_topic_available["virtual_force_command"]
                and required_input_samples["virtual_force_command"]
            ),
            "admittance_command_suggestion": (
                required_topic_available["admittance_command_suggestion"]
                and required_input_samples["admittance_command_suggestion"]
            ),
            "readiness_gates": (
                required_topic_available["readiness_gates"]
                and required_input_samples["readiness_gates"]
            ),
            "proposal_readiness_status": (
                required_topic_available["proposal_readiness_status"]
                and required_input_samples["proposal_readiness_status"]
            ),
            "safety_gate_report": (
                required_topic_available["safety_gate_report"]
                and required_input_samples["safety_gate_report"]
            ),
        }
        required_checks = [
            required_input_results["rgb_image"] if self._required_flags["require_rgb_image"] else True,
            required_input_results["depth_image"] if self._required_flags["require_depth_image"] else True,
            required_input_results["joint_states"] if self._required_flags["require_joint_states"] else True,
            required_input_results["tf"] if self._required_flags["require_tf"] else True,
            required_input_results["contact_wrench"] if self._required_flags["require_contact_wrench"] else True,
            required_input_results["contact_state"] if self._required_flags["require_contact_state"] else True,
            required_input_results["safety_status"] if self._required_flags["require_safety_status"] else True,
            required_input_results["virtual_force_command"]
            if self._required_flags["require_virtual_force_interface"]
            else True,
            required_input_results["admittance_command_suggestion"]
            if self._required_flags["require_admittance_interface"]
            else True,
            required_input_results["readiness_gates"] if self._required_flags["require_readiness_gates"] else True,
            required_input_results["proposal_readiness_status"]
            if self._required_flags["require_proposal_readiness_status"]
            else True,
            required_input_results["safety_gate_report"],
        ]
        all_required_inputs_available = all(required_checks)

        readiness_gates_available = required_input_results["readiness_gates"] and bool(self._last_readiness_payload)
        proposal_readiness_status_available = (
            required_input_results["proposal_readiness_status"] and bool(self._last_proposal_payload)
        )
        upstream_readiness_passed = bool(
            self._last_readiness_payload.get("proposal_readiness_gate", False)
            or self._last_proposal_payload.get("proposal_readiness_gate_passed", False)
        )

        forbidden_outputs_disabled = self._forbidden_outputs_disabled(topics, services)
        diagnostic_outputs_only = self._diagnostic_outputs_only(topic_names)
        input_signal_contract_passed = all_required_inputs_available
        output_suggestion_contract_passed = diagnostic_outputs_only
        safety_constraint_contract_passed = forbidden_outputs_disabled
        execution_block_contract_passed = forbidden_outputs_disabled
        readiness_dependency_contract_passed = (
            readiness_gates_available and proposal_readiness_status_available and upstream_readiness_passed
        )
        future_controller_boundary_contract_passed = (
            forbidden_outputs_disabled
            and bool(self._future_boundary.get("may_read_required_inputs", True))
            and bool(self._future_boundary.get("may_publish_diagnostic_suggestions_only", True))
            and not bool(self._future_boundary.get("may_publish_motion_commands", False))
            and not bool(self._future_boundary.get("may_call_moveit", False))
            and not bool(self._future_boundary.get("may_call_compute_ik", False))
            and not bool(self._future_boundary.get("may_send_follow_joint_trajectory", False))
            and not bool(self._future_boundary.get("may_command_real_robot", False))
        )
        passed = (
            input_signal_contract_passed
            and output_suggestion_contract_passed
            and safety_constraint_contract_passed
            and execution_block_contract_passed
            and readiness_dependency_contract_passed
            and future_controller_boundary_contract_passed
        )
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_fallback_used": bool(
                self.get_parameter("gazebo_fallback_used").get_parameter_value().bool_value
            ),
            "isaac_available": bool(
                self.get_parameter("isaac_available").get_parameter_value().bool_value
            ),
            "robot_model": self._robot_model,
            "input_signal_contract_passed": input_signal_contract_passed,
            "output_suggestion_contract_passed": output_suggestion_contract_passed,
            "safety_constraint_contract_passed": safety_constraint_contract_passed,
            "execution_block_contract_passed": execution_block_contract_passed,
            "readiness_dependency_contract_passed": readiness_dependency_contract_passed,
            "future_controller_boundary_contract_passed": future_controller_boundary_contract_passed,
            "all_required_inputs_available": all_required_inputs_available,
            "diagnostic_outputs_only": diagnostic_outputs_only,
            "forbidden_outputs_disabled": forbidden_outputs_disabled,
            "readiness_gates_available": readiness_gates_available,
            "proposal_readiness_status_available": proposal_readiness_status_available,
            "required_topic_available": required_topic_available,
            "required_input_samples_available": required_input_samples,
            "allowed_diagnostic_output_topics": list(self._allowed_output_topics),
            "forbidden_execution_interfaces": list(self._forbidden_interfaces),
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "trajectory_execution_allowed": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "status": self._success_status if passed else "pre_control_contract_pending",
        }

    def _sample_ready(self, key: str) -> bool:
        return key in self._last and self._last[key] is not None

    def _diagnostic_outputs_only(self, topic_names: set[str]) -> bool:
        required_allowed = {
            "/proposal_simulation_cell/virtual_force_command",
            "/proposal_simulation_cell/admittance_command_suggestion",
            self._status_topic,
            self._boundary_topic,
        }
        return required_allowed.issubset(topic_names) and self._forbidden_outputs_disabled([], [])

    def _forbidden_outputs_disabled(self, topics: list[str], services: list[str]) -> bool:
        if (
            self._command_output_enabled
            or self._motion_execution_enabled
            or self._controller_execution_allowed
            or self._trajectory_execution_allowed
            or self._real_robot_allowed
            or self._moveit_allowed
            or self._compute_ik_allowed
        ):
            return False
        combined = "\n".join(topics + services).lower()
        forbidden_patterns = [
            r"follow_joint_trajectory",
            r"/compute_ik\b",
            r"move_group",
            r"moveit",
            r"trajectory_controller",
            r"joint_trajectory",
            r"/[^ \n]*controller[^ \n]*/commands?\b",
            r"/[^ \n]*(?:scaled_)?joint_trajectory_controller\b",
        ]
        return not any(re.search(pattern, combined) for pattern in forbidden_patterns)

    def _boundary_payload(self, status: dict[str, Any]) -> dict[str, Any]:
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "future_controller_may_read_required_inputs": True,
            "future_controller_may_publish_diagnostic_suggestions_only": True,
            "motion_commands_forbidden": True,
            "moveit_forbidden": True,
            "compute_ik_forbidden": True,
            "controller_execution_forbidden": True,
            "real_robot_execution_forbidden": True,
            "follow_joint_trajectory_forbidden": True,
            "pre_control_contract_valid": status["status"] == self._success_status,
            "status": status["status"],
        }

    def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        publisher.publish(message)

    def _record_rows(self, status: dict[str, Any], boundary: dict[str, Any]) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._status_rows.append(
            {
                "elapsed_sec": elapsed,
                "input_signal_contract_passed": self._bool(status["input_signal_contract_passed"]),
                "output_suggestion_contract_passed": self._bool(status["output_suggestion_contract_passed"]),
                "safety_constraint_contract_passed": self._bool(status["safety_constraint_contract_passed"]),
                "execution_block_contract_passed": self._bool(status["execution_block_contract_passed"]),
                "readiness_dependency_contract_passed": self._bool(status["readiness_dependency_contract_passed"]),
                "future_controller_boundary_contract_passed": self._bool(
                    status["future_controller_boundary_contract_passed"]
                ),
                "all_required_inputs_available": self._bool(status["all_required_inputs_available"]),
                "diagnostic_outputs_only": self._bool(status["diagnostic_outputs_only"]),
                "forbidden_outputs_disabled": self._bool(status["forbidden_outputs_disabled"]),
                "status": str(status["status"]),
            }
        )
        self._boundary_rows.append(
            {
                "elapsed_sec": elapsed,
                "future_controller_may_read_required_inputs": self._bool(
                    boundary["future_controller_may_read_required_inputs"]
                ),
                "future_controller_may_publish_diagnostic_suggestions_only": self._bool(
                    boundary["future_controller_may_publish_diagnostic_suggestions_only"]
                ),
                "motion_commands_forbidden": self._bool(boundary["motion_commands_forbidden"]),
                "moveit_forbidden": self._bool(boundary["moveit_forbidden"]),
                "compute_ik_forbidden": self._bool(boundary["compute_ik_forbidden"]),
                "controller_execution_forbidden": self._bool(boundary["controller_execution_forbidden"]),
                "real_robot_execution_forbidden": self._bool(boundary["real_robot_execution_forbidden"]),
                "follow_joint_trajectory_forbidden": self._bool(
                    boundary["follow_joint_trajectory_forbidden"]
                ),
                "pre_control_contract_valid": self._bool(boundary["pre_control_contract_valid"]),
                "status": str(boundary["status"]),
            }
        )

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        status = self._status(topics, services)
        self._last_status = status
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", self._tf_frame_names())
        self._write_csv(self._output_dir / "pre_control_contract_status_samples.csv", self._status_rows)
        self._write_csv(self._output_dir / "controller_boundary_report_samples.csv", self._boundary_rows)
        self._write_json(self._output_dir / "pre_control_contract_status.json", status)
        contract = self._contract_payload()
        self._write_json(self._output_dir / "pre_control_contract.json", contract)
        self._write_yaml(self._output_dir / "pre_control_contract.yaml", contract)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_7 pre-control contract diagnostics written")
        rclpy.shutdown()

    def _contract_payload(self) -> dict[str, Any]:
        return {
            "input_signal_contract": {
                "required_input_topics": dict(self._required_topics),
                "required_flags": dict(self._required_flags),
            },
            "output_suggestion_contract": {
                "allowed_diagnostic_output_topics": list(self._allowed_output_topics),
                "diagnostic_only": True,
            },
            "safety_constraint_contract": dict(self._safety_constraints),
            "execution_block_contract": {
                "forbidden_execution_interfaces": list(self._forbidden_interfaces),
                "command_output_enabled": False,
                "motion_execution_enabled": False,
                "controller_execution_allowed": False,
                "trajectory_execution_allowed": False,
                "real_robot_allowed": False,
                "moveit_allowed": False,
                "compute_ik_allowed": False,
            },
            "readiness_dependency_contract": dict(self._readiness_dependencies),
            "future_controller_boundary_contract": dict(self._future_boundary),
        }

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
            "# proposal_simulation_cell_v1_7_pre_control_contract",
            "",
            "Purpose: validate a simulation-only pre-control interface contract.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Input signal contract passed: `{status['input_signal_contract_passed']}`",
            f"Output suggestion contract passed: `{status['output_suggestion_contract_passed']}`",
            f"Safety constraint contract passed: `{status['safety_constraint_contract_passed']}`",
            f"Execution block contract passed: `{status['execution_block_contract_passed']}`",
            f"Readiness dependency contract passed: `{status['readiness_dependency_contract_passed']}`",
            f"Future controller boundary contract passed: `{status['future_controller_boundary_contract_passed']}`",
            f"Status: `{status['status']}`",
            "",
            "Safety constraints: command_output_enabled=false, motion_execution_enabled=false, no MoveIt, no /compute_ik, no controllers, no real robot execution, no FollowJointTrajectory, and no trajectory execution.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_7 pre-control contract evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"input_signal_contract_passed={str(status['input_signal_contract_passed']).lower()}",
            f"output_suggestion_contract_passed={str(status['output_suggestion_contract_passed']).lower()}",
            f"safety_constraint_contract_passed={str(status['safety_constraint_contract_passed']).lower()}",
            f"execution_block_contract_passed={str(status['execution_block_contract_passed']).lower()}",
            f"readiness_dependency_contract_passed={str(status['readiness_dependency_contract_passed']).lower()}",
            f"future_controller_boundary_contract_passed={str(status['future_controller_boundary_contract_passed']).lower()}",
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

    def _write_yaml(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(yaml.safe_dump(payload, sort_keys=True), encoding="utf-8")

    def _write_lines(self, path: Path, lines: list[str]) -> None:
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _bool(self, value: Any) -> str:
        return str(bool(value)).lower()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV17PreControlContractNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
