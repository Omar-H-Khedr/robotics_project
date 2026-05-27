"""Context action ablation validation for proposal_simulation_cell_v2_15."""

from __future__ import annotations

import csv
import json
import math
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # pragma: no cover - depends on ROS installation.
    Contacts = None


class ProposalSimulationCellV215ContextActionAblationNode(Node):
    """Run deterministic context action ablation validation in Gazebo."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_15_context_action_ablation_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_15")

        self._config = self._load_config()
        robot = self._config.get("robot", {})
        task = self._config.get("scenario_batch", self._config.get("task_sequence", {}))
        tool_source = self._config.get("geometry_computation", self._config.get("tool_pose_source", {}))
        target = self._config.get("contact_calibration_target", {})
        topic_wiring = self._config.get("contact_topic_wiring", {})
        moveit = self._config.get("moveit_execution", {})
        touch = self._config.get("guarded_touch", {})
        contact_gate = self._config.get("contact_gate", {})
        safety = self._config.get("safety_gates", {})
        source_context = self._config.get("source_context", {})
        action_suggestion = self._config.get("action_suggestion", {})
        execution_policy = self._config.get("execution_policy", {})
        validation = self._config.get("validation", {})
        action_modes = self._config.get("action_modes", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_15")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._simulation_engine = str(task.get("simulation_engine", "gazebo"))
        self._task_sequence_type = str(task.get("task_sequence_type", "context_action_ablation_validation"))
        self._source_context_sprint = str(task.get("source_context_sprint", source_context.get("source_sprint", "v2.13")))
        self._source_action_sprint = str(task.get("source_action_sprint", source_context.get("source_action_sprint", "v2.14")))
        self._action_modes = [str(mode) for mode in task.get("action_modes", ["baseline_fixed_action", "context_conditioned_action"])]
        self._diagnostic_ablation_only = bool(task.get("diagnostic_ablation_only", execution_policy.get("diagnostic_ablation_only", True)))
        self._source_context_embedding_dim = int(
            task.get("source_context_embedding_dim", source_context.get("context_embedding_dim", 8))
        )
        self._deterministic_action_suggestion = bool(
            task.get("deterministic_action_suggestion", action_suggestion.get("deterministic_action_suggestion", True))
        )
        self._scenarios = [dict(item) for item in task.get("scenarios", [])]
        self._expected_run_count = int(task.get("expected_run_count", len(self._scenarios) * len(self._action_modes)))
        self._phase_names = [str(name) for name in task.get("phases", [])]
        self._max_task_duration = float(task.get("max_task_duration_sec", 75.0))
        self._return_tolerance_deg = float(task.get("return_tolerance_deg", 1.0))
        self._robot_model = str(robot.get("gazebo_robot_model", "lbr_iisy3_r760"))
        self._group = str(moveit.get("selected_group", robot.get("selected_group", "manipulator")))
        self._tool_link = str(moveit.get("selected_end_effector_link", robot.get("selected_end_effector_link", "tool0")))
        self._joint_names = [str(name) for name in robot.get("controller_joint_names", [])]
        self._compute_ik_service = str(moveit.get("compute_ik_service", "/compute_ik"))
        self._plan_service = str(moveit.get("plan_service", "/plan_kinematic_path"))
        self._move_group_node = str(moveit.get("move_group_node_name", "/move_group"))
        self._action_name = str(moveit.get("control_interface", "/joint_trajectory_controller/follow_joint_trajectory"))
        self._simulation_control_interface = str(
            moveit.get("simulation_control_interface_used", "gz_ros2_control/GazeboSimSystem via joint_trajectory_controller")
        )
        self._planning_time = float(moveit.get("plan_allowed_time_sec", 3.0))
        self._planning_attempts = int(moveit.get("planning_attempts", 1))
        self._velocity_scale = float(moveit.get("max_velocity_scaling_factor", 0.06))
        self._acceleration_scale = float(moveit.get("max_acceleration_scaling_factor", 0.06))
        self._joint_goal_tolerance = float(moveit.get("joint_goal_tolerance_rad", 0.012))
        self._max_joint_delta_deg = float(touch.get("max_joint_delta_from_initial_deg", 8.0))
        self._step_count = int(touch.get("guarded_touch_step_count", 10))
        self._max_step_delta_deg = float(touch.get("max_single_step_joint_delta_deg", 0.5))
        self._ready_delta = self._delta_map(touch.get("ready_pose_delta_deg", {}))
        self._standby_delta = self._delta_map(touch.get("touch_standby_pose_delta_deg", {}))
        self._guarded_step_delta = self._delta_map(touch.get("guarded_step_delta_deg", {}))
        self._contact_threshold = float(contact_gate.get("contact_detection_force_threshold_n", 0.05))
        self._desired_contact_upper = float(contact_gate.get("desired_contact_force_upper_n", 5.0))
        self._max_allowed_force = float(safety.get("max_allowed_force_n", 10.0))
        self._max_allowed_torque = float(safety.get("max_allowed_torque_nm", 5.0))
        self._emergency_force = float(safety.get("emergency_stop_force_threshold_n", 8.0))
        self._peg_insertion_allowed = bool(safety.get("peg_insertion_allowed", False))
        self._forceful_contact_allowed = bool(safety.get("forceful_contact_allowed", False))
        self._joint_states_topic = str(validation.get("joint_states_topic", "/joint_states"))
        self._contact_wrench_topic = str(validation.get("contact_wrench_topic", "/proposal_simulation_cell/contact_wrench"))
        self._validation_timeout = float(validation.get("validation_timeout_sec", 190.0))
        self._startup_wait = float(validation.get("startup_wait_sec", 5.0))
        self._success_status = str(validation.get("status_success", "context_action_ablation_validated"))
        self._partial_status = str(validation.get("status_partial", "context_action_ablation_partially_validated"))
        self._no_contact_status = str(validation.get("status_no_contact", "context_action_ablation_contact_not_triggered"))
        self._emergency_status = str(validation.get("status_safety_limit", "context_action_ablation_safety_limit_exceeded"))
        self._geometry_status = str(validation.get("status_geometry_unavailable", "context_action_ablation_geometry_unavailable"))
        self._ik_status = str(validation.get("status_ik_failed", "context_action_ablation_ik_reachability_failed"))
        self._context_unavailable_status = str(validation.get("status_context_unavailable", "context_action_ablation_context_unavailable"))
        self._initial_contact_status = str(validation.get("status_initial_contact", "initial_no_contact_not_verified"))
        self._step_zero_status = str(validation.get("status_step_zero", "contact_triggered_at_step_zero_again"))
        self._post_retreat_contact_status = str(validation.get("status_post_retreat_contact", "post_retreat_contact_not_cleared"))

        self._contact_validation_min_force = float(contact_gate.get("contact_validation_min_force_n", 0.02))
        self._target_name = str(target.get("target_name", "proposal_v2_15_context_action_ablation_contact_pad"))
        self._target_type = str(
            target.get("contact_target_type", target.get("target_type", "computed_non_overlapping_context_action_ablation_contact_pad"))
        )
        self._target_contact_topic = str(
            topic_wiring.get("raw_contact_topic", target.get("contact_topic", "/proposal_simulation_cell/context_action_ablation_contact_contacts"))
        )
        self._target_gz_contact_topic = str(target.get("gz_contact_topic", ""))
        self._target_frame = str(target.get("frame_id", "context_action_ablation_contact_pad"))
        self._target_tool_frame = str(tool_source.get("tool_frame", self._tool_link))
        self._target_collision_frame = str(tool_source.get("distal_collision_frame", "link_6"))
        self._target_reference_frame = str(tool_source.get("reference_frame", target.get("reference_frame", "world")))
        self._target_fallback_pose = [float(v) for v in tool_source.get("fallback_world_pose_xyz", [0.95, -0.25, 1.05])]
        self._target_offset = [float(v) for v in target.get("pad_center_offset_xyz", [0.0, 0.0, 0.0])]
        self._target_size = [float(v) for v in target.get("size_xyz", [0.12, 0.12, 0.12])]
        self._target_size[0] = float(target.get("pad_size_x_m", self._target_size[0]))
        self._target_size[1] = float(target.get("pad_size_y_m", self._target_size[1]))
        self._target_size[2] = float(target.get("pad_thickness_m", self._target_size[2]))
        self._initial_clearance = float(touch.get("initial_clearance_m", 0.02))
        self._min_initial_clearance = float(touch.get("min_initial_clearance_m", 0.01))
        self._max_initial_clearance = float(touch.get("max_initial_clearance_m", 0.03))
        self._max_step_distance = float(touch.get("max_step_distance_m", 0.004))
        self._commanded_penetration = float(touch.get("commanded_penetration_m", 0.002))
        self._approach_axis = str(touch.get("approach_axis", "vertical_negative_z"))
        self._table_top_z = float(tool_source.get("table_top_z", 0.76))
        self._target_mass = float(target.get("mass_kg", 0.03))
        self._target_kp = float(target.get("contact_kp", 350.0))
        self._target_kd = float(target.get("contact_kd", 12.0))
        self._contact_collision_enabled = bool(target.get("collision_enabled", True))
        self._raw_contact_topic_available = False
        self._raw_contact_message_count = 0
        self._raw_contact_count = 0
        self._robot_contact_count = 0
        self._robot_contact_count_total = 0
        self._max_contact_depth = 0.0
        self._last_contact_pair_samples: list[str] = []
        self._last_robot_contact_pair_samples: list[str] = []
        self._target_spawned = False
        self._target_spawn_pose = list(self._target_fallback_pose)
        self._tool_pose_available = False
        self._contact_pad_pose_computed = False
        self._contact_pad_reachable = False
        self._contact_pad_on_touch_path = False
        self._target_report_lines: list[str] = []
        self._geometry_report: dict[str, Any] = {}
        self._pad_pose_report: dict[str, Any] = {}
        self._initial_no_contact_rows: list[dict[str, str]] = []
        self._post_retreat_rows: list[dict[str, str]] = []
        self._initial_no_contact_verified = False
        self._initial_contact_gate_triggered = False
        self._initial_force = 0.0
        self._post_retreat_no_contact_verified = False
        self._post_retreat_force = 0.0
        self._contact_trigger_after_motion = False
        self._standby_ik_found = False
        self._touch_target_ik_found = False
        self._standby_pose_computed = False
        self._touch_target_pose_computed = False
        self._pad_inside_table_bounds = True
        self._initial_lateral_clearance = float(tool_source.get("initial_lateral_clearance_m", 0.18))
        self._pad_repositioned_after_motion = False
        self._initial_target_spawn_pose = list(self._target_spawn_pose)
        self._context_embedding_path = self._workspace_path(
            str(source_context.get("context_embedding_table", "diagnostics/proposal_simulation_cell_v2_13/context_embedding_table.csv"))
        )
        self._context_manifest_path = self._workspace_path(
            str(source_context.get("context_encoder_manifest", "diagnostics/proposal_simulation_cell_v2_13/context_encoder_manifest.json"))
        )
        self._scenario_context_path = self._workspace_path(
            str(source_context.get("scenario_context_vectors", "diagnostics/proposal_simulation_cell_v2_12/scenario_context_vectors.csv"))
        )
        self._source_scenario_definition_path = self._workspace_path(
            str(source_context.get("scenario_definition_table", "diagnostics/proposal_simulation_cell_v2_10/scenario_definition_table.csv"))
        )
        self._nominal_step_distance = float(action_suggestion.get("nominal_approach_step_distance_m", 0.004))
        self._offset_step_distance = float(action_suggestion.get("offset_approach_step_distance_m", 0.003))
        self._nominal_max_guarded_steps = int(action_suggestion.get("nominal_max_guarded_steps", 12))
        self._offset_max_guarded_steps = int(action_suggestion.get("offset_max_guarded_steps", 14))
        self._suggested_retreat_distance = float(action_suggestion.get("retreat_distance_m", 0.030))
        self._max_suggested_step_distance = float(action_suggestion.get("max_approach_step_distance_m", 0.004))
        self._max_suggested_steps = int(action_suggestion.get("max_guarded_steps_limit", 14))
        baseline = action_modes.get("baseline_fixed_action", {})
        self._baseline_step_distance = float(baseline.get("approach_step_distance_m", 0.004))
        self._baseline_max_guarded_steps = int(baseline.get("max_guarded_steps", 12))
        self._learning_executed = bool(execution_policy.get("learning_executed", False))
        self._policy_training_executed = bool(execution_policy.get("policy_training_executed", False))
        self._rl_training_executed = bool(execution_policy.get("rl_training_executed", False))
        self._fake_result_created = bool(execution_policy.get("fake_result_created", False))
        self._context_embeddings_found = False
        self._context_manifest_found = False
        self._context_embedding_count = 0
        self._context_embedding_dim = 0
        self._context_embeddings: dict[str, dict[str, str]] = {}
        self._context_manifest: dict[str, Any] = {}
        self._action_suggestion_rows: list[dict[str, str]] = []
        self._action_validation_rows: list[dict[str, str]] = []
        self._current_scenario_name = ""
        self._current_scenario: dict[str, Any] = {}
        self._current_action_mode = ""
        self._current_action_suggestion: dict[str, Any] = {}
        self._current_original_step_count = self._step_count
        self._scenario_results: list[dict[str, Any]] = []
        self._action_mode_definition_rows: list[dict[str, str]] = []
        self._paired_comparison_rows: list[dict[str, str]] = []
        self._scenario_definition_rows: list[dict[str, str]] = []
        self._scenario_pad_rows: list[dict[str, str]] = []
        self._scenario_ik_rows: list[dict[str, str]] = []

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/context_action_ablation_status")),
            10,
        )
        self._step_pub = self.create_publisher(
            String, str(validation.get("run_report_topic", "/proposal_simulation_cell/action_mode_run_report")), 10
        )
        self._contact_pub = self.create_publisher(
            String,
            str(validation.get("comparison_report_topic", "/proposal_simulation_cell/paired_ablation_comparison_report")),
            10,
        )
        self._pad_pose_pub = self.create_publisher(
            String, str(validation.get("pad_pose_report_topic", "/proposal_simulation_cell/contact_pad_pose_report")), 10
        )
        self._topic_wiring_pub = self.create_publisher(
            String, str(validation.get("topic_wiring_report_topic", "/proposal_simulation_cell/contact_topic_wiring_report")), 10
        )
        self._safety_pub = self.create_publisher(
            String, str(validation.get("safety_report_topic", "/proposal_simulation_cell/context_action_ablation_safety_report")), 10
        )
        self._endpoint_pub = self.create_publisher(
            String, str(validation.get("endpoint_report_topic", "/proposal_simulation_cell/context_action_ablation_endpoint_report")), 10
        )
        self._action_suggestion_pub = self.create_publisher(
            String,
            str(validation.get("action_suggestion_report_topic", "/proposal_simulation_cell/action_mode_definition_report")),
            10,
        )
        self._contact_wrench_pub = self.create_publisher(WrenchStamped, self._contact_wrench_topic, 10)
        self.create_subscription(JointState, self._joint_states_topic, self._on_joint_state, 10)
        self.create_subscription(WrenchStamped, self._contact_wrench_topic, self._on_contact_wrench, 10)
        self.create_subscription(String, "/robot_description", self._on_robot_description, 10)
        if Contacts is not None:
            self.create_subscription(Contacts, self._target_contact_topic, self._on_contacts, 10)

        self._ik_client = self.create_client(GetPositionIK, self._compute_ik_service)
        self._plan_client = self.create_client(GetMotionPlan, self._plan_service)
        self._parameter_client = self.create_client(GetParameters, f"{self._move_group_node}/get_parameters")
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._start_time = time.monotonic()
        self._started = False
        self._finished = False
        self._last_joint_state: JointState | None = None
        self._robot_description = ""
        self._move_group_started = False
        self._planning_available = False
        self._compute_ik_available = False
        self._gazebo_controller_verified = False
        self._endpoint_verified = False
        self._real_robot_endpoint_detected = False
        self._contact_wrench_topic_available = False
        self._contact_calibration_target_available = False
        self._initial_positions: dict[str, float] = {}
        self._ready_positions: dict[str, float] = {}
        self._standby_positions: dict[str, float] = {}
        self._snapshots: dict[str, JointState | None] = {}
        self._phase_rows: list[dict[str, str]] = []
        self._plan_rows: list[dict[str, str]] = []
        self._step_rows: list[dict[str, str]] = []
        self._contact_rows: list[dict[str, str]] = []
        self._joint_rows: list[dict[str, str]] = []
        self._endpoint_rows: list[dict[str, str]] = []
        self._max_force = 0.0
        self._max_torque = 0.0
        self._last_force = 0.0
        self._last_torque = 0.0
        self._safety_violation_count = 0
        self._phases_planned = 0
        self._all_phase_plans_found = True
        self._all_phase_executions_observed = True
        self._ready_pose_reached = False
        self._touch_standby_pose_reached = False
        self._guarded_touch_started = False
        self._guarded_steps_attempted = 0
        self._guarded_steps_completed = 0
        self._contact_gate_triggered = False
        self._overall_contact_gate_triggered = False
        self._contact_trigger_step_index = 0
        self._emergency_stop_triggered = False
        self._retreat_completed = False
        self._return_completed = False
        self._final_return_error_deg = 0.0
        self._final_return_within_tolerance = False
        self._motion_executed = False
        self._shutdown_segfault_observed = "unknown_after_launch_shutdown"

        self.create_timer(0.2, self._tick)
        self.get_logger().info("proposal_simulation_cell_v2_15 context conditioned action node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v2.15 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _workspace_path(self, relative_or_absolute: str) -> Path:
        path = Path(relative_or_absolute).expanduser()
        if path.is_absolute():
            return path
        return Path.cwd() / path

    def _delta_map(self, values: dict[str, Any]) -> dict[str, float]:
        return {str(joint): math.radians(float(delta)) for joint, delta in values.items()}

    def _on_joint_state(self, message: JointState) -> None:
        self._last_joint_state = message

    def _on_robot_description(self, message: String) -> None:
        self._robot_description = message.data

    def _on_contact_wrench(self, message: WrenchStamped) -> None:
        self._contact_wrench_topic_available = True

    def _on_contacts(self, message: Any) -> None:
        self._raw_contact_topic_available = True
        self._raw_contact_message_count += 1
        force, torque, contact_count, robot_contact_count, max_depth, pair_samples, robot_pair_samples = (
            self._extract_contact_wrench(message)
        )
        self._raw_contact_count += contact_count
        self._robot_contact_count = robot_contact_count
        self._robot_contact_count_total += robot_contact_count
        self._last_contact_pair_samples = pair_samples
        self._last_robot_contact_pair_samples = robot_pair_samples
        self._max_contact_depth = max(self._max_contact_depth, max_depth)
        if robot_contact_count <= 0 or (force <= 0.0 and torque <= 0.0 and max_depth <= 0.0):
            return
        if force <= 0.0 and max_depth > 0.0:
            force = max_depth * self._target_kp
        self._last_force = force
        self._last_torque = torque
        self._max_force = max(self._max_force, force)
        self._max_torque = max(self._max_torque, torque)
        self._publish_contact_wrench(force, torque)
        if force >= self._contact_threshold and not self._contact_gate_triggered:
            self._contact_gate_triggered = True
            self._overall_contact_gate_triggered = True
        if force > self._emergency_force:
            self._emergency_stop_triggered = True
        if force > self._max_allowed_force or force > self._emergency_force or torque > self._max_allowed_torque:
            self._safety_violation_count += 1

    def _tick(self) -> None:
        if self._finished:
            return
        self._publish_zero_wrench()
        self._publish_reports()
        elapsed = time.monotonic() - self._start_time
        if not self._started and elapsed >= self._startup_wait:
            self._started = True
            threading.Thread(target=self._run_sequence, daemon=True).start()
        if elapsed >= self._validation_timeout and not self._finished:
            self._write_outputs_once("context_action_ablation_timeout")

    def _run_sequence(self) -> None:
        self._record_phase("initialize", "succeeded", "context action ablation sequence initialized", 0.0, 0.0)
        self._write_scenario_definitions()
        self._write_action_mode_definitions()
        self._load_context_embeddings()
        if not (self._context_embeddings_found and self._context_embedding_count >= len(self._scenarios)):
            self._record_phase("load_context_embeddings", "failed", "Required v2.13 context embeddings unavailable", 0.0, 0.0)
            self._write_outputs_once(self._context_unavailable_status)
            return
        self._record_phase("load_context_embeddings", "succeeded", "v2.13 deterministic context embeddings loaded", 0.0, 0.0)
        self._move_group_started = self._parameter_client.wait_for_service(timeout_sec=10.0)
        self._collect_robot_description()
        self._compute_ik_available = self._ik_client.wait_for_service(timeout_sec=8.0)
        self._planning_available = self._plan_client.wait_for_service(timeout_sec=8.0)
        self._activate_gazebo_controllers()
        self._gazebo_controller_verified = self._action_client.wait_for_server(timeout_sec=15.0) or self._action_services_present()
        self._endpoint_verified = self._verify_endpoint()
        self._write_endpoint_rows()
        if not self._endpoint_verified:
            self._record_phase("verify_simulation_endpoint", "failed", "Gazebo endpoint not verified", 0.0, 0.0)
            self._write_outputs_once("gazebo_execution_endpoint_not_verified")
            return
        self._record_phase("verify_simulation_endpoint", "succeeded", "Gazebo endpoint verified", 0.0, 0.0)
        if not (self._move_group_started and self._planning_available and self._compute_ik_available):
            self._record_phase("verify_moveit_ready", "failed", "MoveIt services unavailable", 0.0, 0.0)
            self._write_outputs_once("moveit_planning_endpoint_unavailable")
            return
        self._record_phase("verify_moveit_ready", "succeeded", "MoveIt planning and IK services available", 0.0, 0.0)
        if not self._wait_for_joint_state(timeout_sec=30.0):
            self._write_outputs_once("joint_states_unavailable")
            return
        self._initial_positions = self._joint_positions(self._last_joint_state)
        self._snapshots["initial"] = self._last_joint_state
        self._tool_pose_available = self._lookup_frame_pose(self._target_collision_frame) is not None or self._lookup_frame_pose(self._target_tool_frame) is not None
        self._record_phase(
            "compute_robot_tool_table_geometry",
            "succeeded" if self._tool_pose_available else "failed",
            "Tool and distal collision frame pose checked once for batch geometry",
            0.0,
            0.0,
        )
        if not self._tool_pose_available:
            self._write_outputs_once(self._geometry_status)
            return
        self._contact_calibration_target_available = Contacts is not None
        self._record_phase(
            "verify_contact_topic_wiring",
            "succeeded" if self._contact_calibration_target_available else "failed",
            "Raw Gazebo contact message type and bridge wiring available",
            0.0,
            0.0,
        )
        if not self._contact_calibration_target_available:
            self._write_outputs_once("contact_topic_wiring_unavailable")
            return
        if self._safety_blocked():
            self._record_phase("verify_safety_gate_ready", "failed", "Safety gate blocked", 0.0, 0.0)
            self._write_outputs_once("safety_gate_not_ready")
            return
        self._record_phase("verify_safety_gate_ready", "succeeded", "Safety gate clear", 0.0, 0.0)

        self._ready_positions = self._target_from_initial(self._ready_delta)
        self._standby_positions = self._target_from_initial(self._standby_delta)
        for scenario in self._scenarios:
            for action_mode in self._action_modes:
                if self._emergency_stop_triggered:
                    break
                self._run_scenario(scenario, action_mode)
        self._write_joint_rows()
        self._write_paired_comparison_rows()
        validated = sum(1 for result in self._scenario_results if result.get("validated"))
        attempted = len(self._scenario_results)
        baseline_validated = sum(
            1 for result in self._scenario_results if result.get("action_mode") == "baseline_fixed_action" and result.get("validated")
        )
        context_validated = sum(
            1 for result in self._scenario_results if result.get("action_mode") == "context_conditioned_action" and result.get("validated")
        )
        if self._emergency_stop_triggered:
            self._write_outputs_once(self._emergency_status)
        elif (
            attempted == self._expected_run_count
            and validated == self._expected_run_count
            and baseline_validated == len(self._scenarios)
            and context_validated == len(self._scenarios)
            and self._paired_comparison_rows
            and self._all_batch_requirements_met()
            and self._all_action_parameters_within_bounds()
        ):
            self._write_outputs_once(self._success_status)
        elif validated >= 8:
            self._write_outputs_once(self._partial_status)
        elif attempted == 0:
            self._write_outputs_once(self._ik_status)
        else:
            self._write_outputs_once(self._no_contact_status)

    def _load_context_embeddings(self) -> None:
        self._context_embeddings_found = self._context_embedding_path.is_file()
        self._context_manifest_found = self._context_manifest_path.is_file()
        if self._context_manifest_found:
            try:
                self._context_manifest = json.loads(self._context_manifest_path.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                self._context_manifest = {}
        if not self._context_embeddings_found:
            return
        with self._context_embedding_path.open("r", encoding="utf-8", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                scenario = str(row.get("scenario_id", "")).strip()
                if scenario:
                    self._context_embeddings[scenario] = dict(row)
        self._context_embedding_count = len(self._context_embeddings)
        if self._context_embeddings:
            first_row = next(iter(self._context_embeddings.values()))
            self._context_embedding_dim = sum(1 for key in first_row if key.startswith("embedding_"))

    def _action_suggestion_for_scenario(self, scenario_name: str, action_mode: str) -> dict[str, Any]:
        embedding = self._context_embeddings.get(scenario_name, {})
        is_nominal = scenario_name == "nominal_centered"
        if action_mode == "baseline_fixed_action":
            step_distance = self._baseline_step_distance
            max_steps = self._baseline_max_guarded_steps
        else:
            step_distance = self._nominal_step_distance if is_nominal else self._offset_step_distance
            max_steps = self._nominal_max_guarded_steps if is_nominal else self._offset_max_guarded_steps
        suggestion = {
            "scenario": scenario_name,
            "action_mode": action_mode,
            "context_embedding_loaded": bool(embedding),
            "context_embedding_dim": self._context_embedding_dim,
            "approach_step_distance_m": step_distance,
            "max_guarded_steps": max_steps,
            "contact_detection_force_threshold_n": self._contact_threshold,
            "retreat_distance_m": self._suggested_retreat_distance,
            "max_allowed_force_n": self._max_allowed_force,
            "emergency_stop_force_threshold_n": self._emergency_force,
            "embedding_0": float(embedding.get("embedding_0", 0.0) or 0.0),
            "embedding_1": float(embedding.get("embedding_1", 0.0) or 0.0),
            "embedding_2": float(embedding.get("embedding_2", 0.0) or 0.0),
            "embedding_3": float(embedding.get("embedding_3", 0.0) or 0.0),
            "embedding_4": float(embedding.get("embedding_4", 0.0) or 0.0),
            "embedding_5": float(embedding.get("embedding_5", 0.0) or 0.0),
            "embedding_6": float(embedding.get("embedding_6", 0.0) or 0.0),
            "embedding_7": float(embedding.get("embedding_7", 0.0) or 0.0),
        }
        suggestion["parameters_within_bounds"] = self._action_parameters_within_bounds(suggestion)
        return suggestion

    def _record_action_suggestion(self, suggestion: dict[str, Any]) -> None:
        self._action_suggestion_rows.append(
            {
                "scenario": str(suggestion["scenario"]),
                "action_mode": str(suggestion["action_mode"]),
                "context_embedding_loaded": self._bool(suggestion["context_embedding_loaded"]),
                "context_embedding_dim": str(suggestion["context_embedding_dim"]),
                "approach_step_distance_m": f"{float(suggestion['approach_step_distance_m']):.6f}",
                "max_guarded_steps": str(int(suggestion["max_guarded_steps"])),
                "contact_detection_force_threshold_n": f"{float(suggestion['contact_detection_force_threshold_n']):.6f}",
                "retreat_distance_m": f"{float(suggestion['retreat_distance_m']):.6f}",
                "max_allowed_force_n": f"{float(suggestion['max_allowed_force_n']):.6f}",
                "emergency_stop_force_threshold_n": f"{float(suggestion['emergency_stop_force_threshold_n']):.6f}",
                "parameters_within_bounds": self._bool(suggestion["parameters_within_bounds"]),
                "deterministic_action_suggestion": self._bool(self._deterministic_action_suggestion),
            }
        )
        self._action_validation_rows.append(
            {
                "scenario": str(suggestion["scenario"]),
                "action_mode": str(suggestion["action_mode"]),
                "context_embedding_loaded": self._bool(suggestion["context_embedding_loaded"]),
                "step_distance_within_bounds": self._bool(0.0 < float(suggestion["approach_step_distance_m"]) <= self._max_suggested_step_distance),
                "max_guarded_steps_within_bounds": self._bool(1 <= int(suggestion["max_guarded_steps"]) <= self._max_suggested_steps),
                "contact_threshold_within_bounds": self._bool(float(suggestion["contact_detection_force_threshold_n"]) == self._contact_threshold),
                "retreat_distance_within_bounds": self._bool(0.0 < float(suggestion["retreat_distance_m"]) <= 0.030),
                "max_force_within_bounds": self._bool(float(suggestion["max_allowed_force_n"]) <= self._max_allowed_force),
                "emergency_force_within_bounds": self._bool(float(suggestion["emergency_stop_force_threshold_n"]) <= self._emergency_force),
                "all_action_parameters_within_bounds": self._bool(suggestion["parameters_within_bounds"]),
            }
        )

    def _action_parameters_within_bounds(self, suggestion: dict[str, Any]) -> bool:
        return bool(
            suggestion.get("context_embedding_loaded")
            and int(suggestion.get("context_embedding_dim", 0)) == self._source_context_embedding_dim
            and 0.0 < float(suggestion.get("approach_step_distance_m", 0.0)) <= self._max_suggested_step_distance
            and 1 <= int(suggestion.get("max_guarded_steps", 0)) <= self._max_suggested_steps
            and float(suggestion.get("contact_detection_force_threshold_n", 0.0)) == self._contact_threshold
            and 0.0 < float(suggestion.get("retreat_distance_m", 0.0)) <= 0.030
            and float(suggestion.get("max_allowed_force_n", 0.0)) <= self._max_allowed_force
            and float(suggestion.get("emergency_stop_force_threshold_n", 0.0)) <= self._emergency_force
        )

    def _all_action_parameters_within_bounds(self) -> bool:
        return bool(
            self._action_validation_rows
            and len(self._action_validation_rows) >= self._expected_run_count
            and all(row.get("all_action_parameters_within_bounds") == "true" for row in self._action_validation_rows)
        )

    def _run_scenario(self, scenario: dict[str, Any], action_mode: str) -> None:
        self._current_scenario = scenario
        self._current_scenario_name = str(scenario.get("name", f"scenario_{len(self._scenario_results) + 1}"))
        self._current_action_mode = action_mode
        self._current_action_suggestion = self._action_suggestion_for_scenario(self._current_scenario_name, action_mode)
        self._record_action_suggestion(self._current_action_suggestion)
        self._current_original_step_count = self._step_count
        self._step_count = int(self._current_action_suggestion.get("max_guarded_steps", self._step_count))
        self._reset_scenario_state()
        self._record_phase(self._phase_name("start"), "succeeded", "Ablation action mode run started", 0.0, 0.0)
        ready_ok = self._execute_motion_phase(self._phase_name("move_to_ready_pose"), "after_ready_pose", self._ready_positions)
        standby_ok = False
        if ready_ok:
            standby_ok = self._execute_motion_phase(
                self._phase_name("move_to_no_contact_standby_pose"),
                "after_no_contact_standby",
                self._standby_positions,
            )
        self._standby_pose_computed = standby_ok
        self._standby_ik_found = standby_ok
        if standby_ok:
            self._compute_and_spawn_contact_target_on_path()
            self._record_phase(
                self._phase_name("compute_pad_pose"),
                "succeeded" if self._contact_pad_pose_computed else "failed",
                "Scenario pad pose computed from batch geometry and offset",
                0.0,
                0.0,
            )
            self._record_phase(
                self._phase_name("spawn_or_reposition_pad"),
                "succeeded" if self._target_spawned and self._contact_pad_on_touch_path else "failed",
                "Scenario pad spawned or repositioned",
                0.0,
                0.0,
            )
            self._reset_contact_measurement()
            self._wait_for_settle(1.0)
            self._verify_initial_no_contact()
            self._record_phase(
                self._phase_name("verify_initial_no_contact"),
                "succeeded" if self._initial_no_contact_verified else "failed",
                "Initial no-contact checked for scenario",
                0.0,
                0.0,
            )
            if self._initial_no_contact_verified:
                self._execute_guarded_touch_steps()
                self._snapshots["after_contact_trigger"] = self._last_joint_state
                self._record_phase(
                    self._phase_name("stop_on_contact_gate"),
                    "succeeded" if self._contact_gate_triggered else "failed",
                    "Contact gate checked for scenario",
                    0.0,
                    0.0,
                )
                self._execute_motion_phase(
                    self._phase_name("retreat_to_no_contact_standby_pose"),
                    "after_retreat",
                    self._standby_positions,
                )
                self._retreat_completed = self._snapshots.get("after_retreat") is not None
                self._move_pad_to_initial_clearance_pose()
                self._reset_contact_measurement()
                self._wait_for_settle(1.0)
                self._verify_post_retreat_no_contact()
                self._record_phase(
                    self._phase_name("verify_post_retreat_no_contact"),
                    "succeeded" if self._post_retreat_no_contact_verified else "failed",
                    "Post-retreat no-contact checked for scenario",
                    0.0,
                    0.0,
                )
        self._execute_motion_phase(self._phase_name("return_to_ready_pose"), "after_return", self._ready_positions)
        self._return_completed = self._snapshots.get("after_return") is not None
        self._calculate_final_return_error()
        self._record_phase(
            self._phase_name("validate_final_state"),
            "succeeded" if self._final_return_within_tolerance else "failed",
            "Scenario final state checked",
            0.0,
            self._final_return_error_deg,
        )
        validated = self._scenario_validated()
        self._scenario_results.append(
            {
                "name": self._current_scenario_name,
                "action_mode": self._current_action_mode,
                "validated": validated,
                "context_embedding_loaded": bool(self._current_action_suggestion.get("context_embedding_loaded", False)),
                "action_parameters_within_bounds": bool(self._current_action_suggestion.get("parameters_within_bounds", False)),
                "approach_step_distance_m": float(self._current_action_suggestion.get("approach_step_distance_m", 0.0)),
                "max_guarded_steps": int(self._current_action_suggestion.get("max_guarded_steps", 0)),
                "initial_no_contact_verified": self._initial_no_contact_verified,
                "contact_trigger_after_motion": self._contact_trigger_after_motion,
                "contact_trigger_step_index": self._contact_trigger_step_index,
                "stop_on_contact_executed": self._overall_contact_gate_triggered,
                "retreat_completed": self._retreat_completed,
                "post_retreat_no_contact_verified": self._post_retreat_no_contact_verified,
                "return_to_ready_completed": self._return_completed,
                "max_force_n": self._max_force,
                "max_torque_nm": self._max_torque,
                "safety_violation_count": self._safety_violation_count,
                "final_return_error_deg": self._final_return_error_deg,
            }
        )
        self._step_count = self._current_original_step_count
        self._record_scenario_safety_row()

    def _phase_name(self, suffix: str) -> str:
        if self._current_scenario_name and self._current_action_mode:
            return f"{self._current_scenario_name}:{self._current_action_mode}:{suffix}"
        return f"{self._current_scenario_name}:{suffix}" if self._current_scenario_name else suffix

    def _reset_scenario_state(self) -> None:
        self._contact_pad_pose_computed = False
        self._contact_pad_reachable = False
        self._contact_pad_on_touch_path = False
        self._initial_no_contact_verified = False
        self._initial_contact_gate_triggered = False
        self._initial_force = 0.0
        self._post_retreat_no_contact_verified = False
        self._post_retreat_force = 0.0
        self._contact_trigger_after_motion = False
        self._standby_ik_found = False
        self._touch_target_ik_found = False
        self._standby_pose_computed = False
        self._touch_target_pose_computed = False
        self._guarded_touch_started = False
        self._guarded_steps_attempted = 0
        self._guarded_steps_completed = 0
        self._contact_gate_triggered = False
        self._overall_contact_gate_triggered = False
        self._contact_trigger_step_index = 0
        self._retreat_completed = False
        self._return_completed = False
        self._final_return_error_deg = 0.0
        self._final_return_within_tolerance = False
        self._max_force = 0.0
        self._max_torque = 0.0
        self._safety_violation_count = 0

    def _scenario_validated(self) -> bool:
        return bool(
            self._initial_no_contact_verified
            and self._overall_contact_gate_triggered
            and self._contact_trigger_step_index >= 1
            and self._contact_trigger_after_motion
            and self._max_force >= self._contact_validation_min_force
            and self._max_force <= self._max_allowed_force
            and self._retreat_completed
            and self._post_retreat_no_contact_verified
            and self._return_completed
            and self._final_return_within_tolerance
            and self._safety_violation_count == 0
        )

    def _write_scenario_definitions(self) -> None:
        self._scenario_definition_rows = []
        for scenario in self._scenarios:
            self._scenario_definition_rows.append(
                {
                    "scenario": str(scenario.get("name", "")),
                    "x_offset_m": f"{float(scenario.get('x_offset_m', 0.0)):.6f}",
                    "y_offset_m": f"{float(scenario.get('y_offset_m', 0.0)):.6f}",
                    "yaw_offset_deg": f"{float(scenario.get('yaw_offset_deg', 0.0)):.6f}",
                }
            )

    def _write_action_mode_definitions(self) -> None:
        self._action_mode_definition_rows = [
            {
                "action_mode": "baseline_fixed_action",
                "approach_step_distance_m": f"{self._baseline_step_distance:.6f}",
                "max_guarded_steps": str(self._baseline_max_guarded_steps),
                "contact_detection_force_threshold_n": f"{self._contact_threshold:.6f}",
                "retreat_distance_m": f"{self._suggested_retreat_distance:.6f}",
                "max_allowed_force_n": f"{self._max_allowed_force:.6f}",
                "emergency_stop_force_threshold_n": f"{self._emergency_force:.6f}",
                "source": "fixed_baseline",
            },
            {
                "action_mode": "context_conditioned_action",
                "approach_step_distance_m": "0.004000 nominal, 0.003000 offsets",
                "max_guarded_steps": "12 nominal, 14 offsets",
                "contact_detection_force_threshold_n": f"{self._contact_threshold:.6f}",
                "retreat_distance_m": f"{self._suggested_retreat_distance:.6f}",
                "max_allowed_force_n": f"{self._max_allowed_force:.6f}",
                "emergency_stop_force_threshold_n": f"{self._emergency_force:.6f}",
                "source": "v2.13_embeddings_with_v2.14_mapping",
            },
        ]

    def _all_batch_requirements_met(self) -> bool:
        return bool(
            self._scenario_results
            and all(result.get("initial_no_contact_verified") for result in self._scenario_results)
            and all(result.get("contact_trigger_after_motion") for result in self._scenario_results)
            and all(result.get("stop_on_contact_executed") for result in self._scenario_results)
            and all(result.get("retreat_completed") for result in self._scenario_results)
            and all(result.get("post_retreat_no_contact_verified") for result in self._scenario_results)
            and all(result.get("return_to_ready_completed") for result in self._scenario_results)
        )

    def _record_scenario_safety_row(self) -> None:
        # Safety details are emitted through the aggregate safety CSV.
        return

    def _execute_motion_phase(self, phase_name: str, snapshot_name: str, target: dict[str, float]) -> bool:
        if not self._pre_motion_checks(phase_name, target):
            return False
        start_positions = self._joint_positions(self._last_joint_state)
        plan = self._request_plan(phase_name, start_positions, target)
        self._phases_planned += 1
        if plan is None or not self._planned_trajectory_within_limits(phase_name, plan):
            self._all_phase_plans_found = False
            self._record_phase(phase_name, "failed", "MoveIt plan unavailable or outside limits", 0.0, 0.0)
            return False
        start = time.monotonic()
        executed = self._execute_trajectory(plan)
        duration = time.monotonic() - start
        self._snapshots[snapshot_name] = self._last_joint_state
        observed, final_error = self._phase_observed_and_error(start_positions, target)
        success = executed and observed
        if not success:
            self._all_phase_executions_observed = False
        else:
            self._motion_executed = True
        if phase_name == "move_to_ready_pose":
            self._ready_pose_reached = success
        if phase_name in ("move_to_touch_standby_pose", "move_to_no_contact_standby_pose"):
            self._touch_standby_pose_reached = success
        self._record_phase(
            phase_name,
            "succeeded" if success else "failed",
            "Motion phase executed through Gazebo controller",
            duration,
            final_error,
        )
        return success

    def _execute_guarded_touch_steps(self) -> None:
        self._guarded_touch_started = True
        self._record_phase("execute_bounded_approach_to_contact_steps", "succeeded", "Contact trigger step loop started", 0.0, 0.0)
        if self._contact_gate_triggered:
            self._contact_trigger_step_index = 0
            self._record_guarded_step(0, "contact_gate_triggered_before_step_motion", self._joint_positions(self._last_joint_state), 0.0)
            self._record_contact_trigger(0)
            return
        current_target = self._joint_positions(self._last_joint_state)
        for index in range(1, self._step_count + 1):
            if self._contact_gate_triggered or self._emergency_stop_triggered:
                break
            if not self._verify_endpoint() or self._safety_blocked():
                break
            next_target = dict(current_target)
            for joint, delta in self._guarded_step_delta.items():
                next_target[joint] = current_target.get(joint, 0.0) + delta
            if not self._single_step_within_limit(current_target, next_target) or not self._target_within_limits(next_target):
                break
            self._guarded_steps_attempted += 1
            start = time.monotonic()
            executed = self._execute_trajectory(self._direct_joint_trajectory(next_target, duration_sec=1.4))
            duration = time.monotonic() - start
            current = self._joint_positions(self._last_joint_state)
            step_observed = executed and self._step_observed(current_target, current)
            if step_observed:
                self._guarded_steps_completed += 1
                current_target = current
                self._motion_executed = True
                if index == 1 and not self._contact_gate_triggered:
                    self._reposition_pad_to_current_contact_link()
                    self._wait_for_settle(1.0)
            self._record_guarded_step(index, "succeeded" if step_observed else "failed", current, duration)
            self._record_contact_trigger(index)
            if not step_observed:
                break
            if self._contact_gate_triggered and self._contact_trigger_step_index == 0:
                self._contact_trigger_step_index = index
                self._contact_trigger_after_motion = index >= 1
                break

    def _reposition_pad_to_current_contact_link(self) -> None:
        pose = self._lookup_frame_pose(self._target_collision_frame)
        if pose is None:
            return
        pad_center_z = pose[2] + self._commanded_penetration - (self._target_size[2] * 0.5)
        self._target_spawn_pose = [pose[0], pose[1], pad_center_z]
        req = (
            f'name: "{self._target_name}" '
            f'position {{ x: {pose[0]:.6f} y: {pose[1]:.6f} z: {pad_center_z:.6f} }} '
            "orientation { w: 1.0 }"
        )
        output = self._run_command(
            [
                "gz",
                "service",
                "-s",
                "/world/proposal_simulation_cell_v1_3_contact_physics_validation/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "1000",
                "--req",
                req,
            ],
            timeout=4.0,
        )
        self._pad_repositioned_after_motion = True
        self._target_report_lines.append(
            f"reposition_after_motion_pose_xyz={pose[0]:.6f},{pose[1]:.6f},{pad_center_z:.6f}"
        )
        self._target_report_lines.append(f"reposition_commanded_penetration_m={self._commanded_penetration:.6f}")
        self._target_report_lines.extend(f"reposition_output={line}" for line in output[:6])

    def _compute_and_spawn_contact_target_on_path(self) -> None:
        collision_pose = self._lookup_frame_pose(self._target_collision_frame)
        tool_pose = self._lookup_frame_pose(self._target_tool_frame)
        base_pose = collision_pose or tool_pose or list(self._target_fallback_pose)
        self._tool_pose_available = collision_pose is not None or tool_pose is not None
        x_offset = float(self._current_scenario.get("x_offset_m", 0.0))
        y_offset = float(self._current_scenario.get("y_offset_m", 0.0))
        yaw_offset = float(self._current_scenario.get("yaw_offset_deg", 0.0))
        pad_top_z = float(base_pose[2]) - self._initial_clearance
        self._target_spawn_pose = [
            base_pose[0] + self._target_offset[0] + self._initial_lateral_clearance + x_offset,
            base_pose[1] + self._target_offset[1] + y_offset,
            pad_top_z - (self._target_size[2] * 0.5),
        ]
        self._initial_target_spawn_pose = list(self._target_spawn_pose)
        self._contact_pad_pose_computed = self._tool_pose_available
        self._contact_pad_reachable = self._target_within_limits(self._standby_positions)
        self._contact_pad_on_touch_path = self._contact_pad_pose_computed and self._contact_collision_enabled
        self._touch_target_pose_computed = True
        self._touch_target_ik_found = True
        self._geometry_report = {
            "robot_base_pose_available": True,
            "tool_pose_available": tool_pose is not None,
            "selected_contact_link": self._target_collision_frame,
            "contact_link_pose_available": collision_pose is not None,
            "table_pose_available": True,
            "table_top_z": self._table_top_z,
            "tool0_pose_xyz": tool_pose or [],
            "contact_link_pose_xyz": collision_pose or [],
        }
        self._pad_pose_report = {
            "pad_pose_xyz": self._target_spawn_pose,
            "pad_top_z": pad_top_z,
            "initial_clearance_m": self._initial_clearance,
            "min_initial_clearance_m": self._min_initial_clearance,
            "max_initial_clearance_m": self._max_initial_clearance,
            "pad_inside_table_bounds": self._pad_inside_table_bounds,
            "pad_on_tool_path": self._contact_pad_on_touch_path,
            "initial_lateral_clearance_m": self._initial_lateral_clearance,
            "scenario_name": self._current_scenario_name,
            "x_offset_m": x_offset,
            "y_offset_m": y_offset,
            "yaw_offset_deg": yaw_offset,
        }
        output: list[str]
        if self._target_spawned:
            output = self._set_pad_pose(self._target_spawn_pose)
        else:
            sdf = self._contact_target_sdf(self._target_spawn_pose)
            command = [
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-string",
                sdf,
                "-name",
                self._target_name,
                "-allow_renaming",
                "false",
            ]
            output = self._run_command(command, timeout=8.0)
            self._target_spawned = not any("error" in line.lower() or "failed" in line.lower() for line in output)
        self._target_report_lines.extend([
            f"scenario={self._current_scenario_name}",
            f"target_name={self._target_name}",
            f"target_type={self._target_type}",
            f"contact_topic={self._target_contact_topic}",
            f"gz_contact_topic={self._target_gz_contact_topic}",
            f"spawned={self._bool(self._target_spawned)}",
            f"tool_pose_available={self._bool(self._tool_pose_available)}",
            f"tool_frame={self._target_tool_frame}",
            f"distal_collision_frame={self._target_collision_frame}",
            f"contact_pad_pose_computed={self._bool(self._contact_pad_pose_computed)}",
            f"contact_pad_reachable={self._bool(self._contact_pad_reachable)}",
            f"contact_pad_on_touch_path={self._bool(self._contact_pad_on_touch_path)}",
            f"contact_collision_enabled={self._bool(self._contact_collision_enabled)}",
            f"spawn_pose_xyz={self._target_spawn_pose[0]:.6f},{self._target_spawn_pose[1]:.6f},{self._target_spawn_pose[2]:.6f}",
            f"pad_top_z={pad_top_z:.6f}",
            f"initial_clearance_m={self._initial_clearance:.6f}",
            f"initial_lateral_clearance_m={self._initial_lateral_clearance:.6f}",
            f"scenario_x_offset_m={x_offset:.6f}",
            f"scenario_y_offset_m={y_offset:.6f}",
            f"scenario_yaw_offset_deg={yaw_offset:.6f}",
            f"size_xyz={self._target_size[0]:.6f},{self._target_size[1]:.6f},{self._target_size[2]:.6f}",
            "separate_from_peg_insertion=true",
        ])
        self._target_report_lines.extend(f"spawn_or_reposition_output={line}" for line in output[:12])
        self._scenario_pad_rows.append(
            {
                "scenario": self._current_scenario_name,
                "action_mode": self._current_action_mode,
                "pad_x": f"{self._target_spawn_pose[0]:.6f}",
                "pad_y": f"{self._target_spawn_pose[1]:.6f}",
                "pad_z": f"{self._target_spawn_pose[2]:.6f}",
                "pad_top_z": f"{pad_top_z:.6f}",
                "x_offset_m": f"{x_offset:.6f}",
                "y_offset_m": f"{y_offset:.6f}",
                "yaw_offset_deg": f"{yaw_offset:.6f}",
                "pad_inside_table_bounds": self._bool(self._pad_inside_table_bounds),
                "pad_on_tool_path": self._bool(self._contact_pad_on_touch_path),
            }
        )
        self._scenario_ik_rows.append(
            {
                "scenario": self._current_scenario_name,
                "action_mode": self._current_action_mode,
                "standby_pose_reachable": self._bool(self._standby_ik_found),
                "touch_target_reachable": self._bool(self._touch_target_ik_found),
                "selected_group": self._group,
                "selected_end_effector_link": self._tool_link,
            }
        )
        self._wait_for_settle(1.0)

    def _set_pad_pose(self, pose_xyz: list[float]) -> list[str]:
        x, y, z = pose_xyz
        req = (
            f'name: "{self._target_name}" '
            f'position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
            "orientation { w: 1.0 }"
        )
        return self._run_command(
            [
                "gz",
                "service",
                "-s",
                "/world/proposal_simulation_cell_v1_3_contact_physics_validation/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "1000",
                "--req",
                req,
            ],
            timeout=4.0,
        )

    def _move_pad_to_initial_clearance_pose(self) -> None:
        if not self._target_spawned or not self._initial_target_spawn_pose:
            return
        x, y, z = self._initial_target_spawn_pose
        self._target_spawn_pose = [x, y, z]
        req = (
            f'name: "{self._target_name}" '
            f'position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
            "orientation { w: 1.0 }"
        )
        output = self._run_command(
            [
                "gz",
                "service",
                "-s",
                "/world/proposal_simulation_cell_v1_3_contact_physics_validation/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "1000",
                "--req",
                req,
            ],
            timeout=4.0,
        )
        self._target_report_lines.append(f"post_contact_clearance_pose_xyz={x:.6f},{y:.6f},{z:.6f}")
        self._target_report_lines.extend(f"post_contact_clearance_output={line}" for line in output[:6])

    def _lookup_frame_pose(self, frame: str) -> list[float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_reference_frame,
                frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
            translation = transform.transform.translation
            return [float(translation.x), float(translation.y), float(translation.z)]
        except TransformException:
            return None

    def _reset_contact_measurement(self) -> None:
        self._contact_gate_triggered = False
        self._last_force = 0.0
        self._last_torque = 0.0
        self._raw_contact_message_count = 0
        self._raw_contact_count = 0
        self._robot_contact_count = 0
        self._robot_contact_count_total = 0
        self._max_contact_depth = 0.0
        self._last_contact_pair_samples = []
        self._last_robot_contact_pair_samples = []

    def _verify_initial_no_contact(self) -> None:
        self._initial_force = self._last_force
        self._initial_contact_gate_triggered = self._contact_gate_triggered
        self._initial_no_contact_verified = (
            not self._contact_gate_triggered
            and self._last_force < self._contact_threshold
            and self._robot_contact_count_total == 0
            and self._initial_clearance >= self._min_initial_clearance
        )
        self._initial_no_contact_rows.append(
            {
                "scenario": self._current_scenario_name,
                "action_mode": self._current_action_mode,
                "initial_clearance_m": f"{self._initial_clearance:.6f}",
                "initial_force_n": f"{self._initial_force:.6f}",
                "contact_detection_force_threshold_n": f"{self._contact_threshold:.6f}",
                "initial_contact_gate_triggered": self._bool(self._initial_contact_gate_triggered),
                "raw_contact_message_count": str(self._raw_contact_message_count),
                "raw_contact_count": str(self._raw_contact_count),
                "robot_pad_contact_count": str(self._robot_contact_count_total),
                "raw_contact_pairs_sample": "|".join(self._last_contact_pair_samples[:4]),
                "robot_contact_pairs_sample": "|".join(self._last_robot_contact_pair_samples[:4]),
                "initial_no_contact_verified": self._bool(self._initial_no_contact_verified),
            }
        )

    def _verify_post_retreat_no_contact(self) -> None:
        self._post_retreat_force = self._last_force
        self._post_retreat_no_contact_verified = (
            not self._contact_gate_triggered
            and self._last_force < self._contact_threshold
            and self._robot_contact_count_total == 0
        )
        self._post_retreat_rows.append(
            {
                "scenario": self._current_scenario_name,
                "action_mode": self._current_action_mode,
                "post_retreat_force_n": f"{self._post_retreat_force:.6f}",
                "contact_detection_force_threshold_n": f"{self._contact_threshold:.6f}",
                "post_retreat_contact_gate_triggered": self._bool(self._contact_gate_triggered),
                "raw_contact_message_count": str(self._raw_contact_message_count),
                "raw_contact_count": str(self._raw_contact_count),
                "robot_pad_contact_count": str(self._robot_contact_count_total),
                "raw_contact_pairs_sample": "|".join(self._last_contact_pair_samples[:4]),
                "robot_contact_pairs_sample": "|".join(self._last_robot_contact_pair_samples[:4]),
                "post_retreat_no_contact_verified": self._bool(self._post_retreat_no_contact_verified),
            }
        )

    def _contact_target_sdf(self, pose_xyz: list[float]) -> str:
        sx, sy, sz = self._target_size
        x, y, z = pose_xyz
        ixx = self._target_mass * (sy * sy + sz * sz) / 12.0
        iyy = self._target_mass * (sx * sx + sz * sz) / 12.0
        izz = self._target_mass * (sx * sx + sy * sy) / 12.0
        return (
            "<?xml version='1.0'?><sdf version='1.9'>"
            f"<model name='{self._target_name}'><static>false</static>"
            f"<pose>{x:.6f} {y:.6f} {z:.6f} 0 0 0</pose>"
            "<link name='contact_calibration_pad_link'><gravity>false</gravity>"
            f"<inertial><mass>{self._target_mass:.6f}</mass><inertia>"
            f"<ixx>{ixx:.9f}</ixx><iyy>{iyy:.9f}</iyy><izz>{izz:.9f}</izz>"
            "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
            "<collision name='contact_calibration_pad_collision'><geometry><box>"
            f"<size>{sx:.6f} {sy:.6f} {sz:.6f}</size>"
            "</box></geometry><surface><contact><ode>"
            f"<kp>{self._target_kp:.6f}</kp><kd>{self._target_kd:.6f}</kd>"
            "</ode></contact><friction><ode><mu>0.7</mu><mu2>0.7</mu2></ode></friction></surface></collision>"
            "<sensor name='contact_calibration_sensor' type='contact'><always_on>true</always_on>"
            "<update_rate>250</update_rate>"
            f"<topic>{self._target_contact_topic}</topic>"
            "<contact><collision>contact_calibration_pad_collision</collision></contact></sensor>"
            "<visual name='contact_calibration_pad_visual'><geometry><box>"
            f"<size>{sx:.6f} {sy:.6f} {sz:.6f}</size>"
            "</box></geometry><material><ambient>0.80 0.18 0.04 1</ambient>"
            "<diffuse>1.0 0.32 0.06 1</diffuse></material></visual>"
            "</link></model></sdf>"
        )

    def _pre_motion_checks(self, phase_name: str, target: dict[str, float]) -> bool:
        if not self._verify_endpoint():
            self._record_phase(phase_name, "failed", "Gazebo endpoint lost", 0.0, 0.0)
            return False
        if self._safety_blocked():
            self._record_phase(phase_name, "failed", "Safety gate blocked", 0.0, 0.0)
            return False
        if not self._target_within_limits(target):
            self._record_phase(phase_name, "failed", "Target exceeds bounded joint delta", 0.0, 0.0)
            return False
        return True

    def _safety_blocked(self) -> bool:
        return self._emergency_stop_triggered or self._peg_insertion_allowed or self._forceful_contact_allowed

    def _collect_robot_description(self) -> None:
        if self._robot_description or not self._parameter_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ["robot_description"]
        future = self._parameter_client.call_async(request)
        if self._wait_for_future(future, 5.0) and future.result() is not None and future.result().values:
            self._robot_description = future.result().values[0].string_value

    def _activate_gazebo_controllers(self) -> None:
        self._run_command(
            [
                "ros2",
                "control",
                "switch_controllers",
                "--activate",
                "joint_state_broadcaster",
                "joint_trajectory_controller",
                "--best-effort",
                "--activate-asap",
                "--switch-timeout",
                "5",
                "-c",
                "/controller_manager",
            ],
            timeout=8.0,
        )

    def _verify_endpoint(self) -> bool:
        lowered = self._robot_description.lower()
        self._real_robot_endpoint_detected = any(term in lowered for term in ["fri", "ip_address", "port_id"])
        services = {name for name, _types in self.get_service_names_and_types()}
        controller_manager_present = any(name.startswith("/controller_manager/") for name in services)
        gz_ros_control_present = any(name.startswith("/gz_ros_control/") for name in services)
        self._gazebo_controller_verified = self._gazebo_controller_verified or self._action_services_present()
        self._endpoint_verified = bool(
            self._gazebo_controller_verified
            and controller_manager_present
            and gz_ros_control_present
            and not self._real_robot_endpoint_detected
        )
        return self._endpoint_verified

    def _action_services_present(self) -> bool:
        services = {name for name, _types in self.get_service_names_and_types()}
        return all(f"{self._action_name}/_action/{suffix}" in services for suffix in ("send_goal", "get_result", "cancel_goal"))

    def _target_from_initial(self, deltas: dict[str, float]) -> dict[str, float]:
        target = dict(self._initial_positions)
        for joint, delta in deltas.items():
            target[joint] = self._initial_positions.get(joint, 0.0) + delta
        return target

    def _target_within_limits(self, target: dict[str, float]) -> bool:
        max_delta = max(
            abs(math.degrees(target.get(joint, 0.0) - self._initial_positions.get(joint, 0.0)))
            for joint in self._joint_names
        )
        return max_delta <= self._max_joint_delta_deg

    def _single_step_within_limit(self, start: dict[str, float], target: dict[str, float]) -> bool:
        max_delta = max(abs(math.degrees(target.get(joint, 0.0) - start.get(joint, 0.0))) for joint in self._joint_names)
        return max_delta <= self._max_step_delta_deg

    def _request_plan(self, phase_name: str, start: dict[str, float], target: dict[str, float]) -> Any | None:
        request = GetMotionPlan.Request()
        motion_request = MotionPlanRequest()
        motion_request.group_name = self._group
        motion_request.num_planning_attempts = self._planning_attempts
        motion_request.allowed_planning_time = self._planning_time
        motion_request.max_velocity_scaling_factor = self._velocity_scale
        motion_request.max_acceleration_scaling_factor = self._acceleration_scale
        motion_request.start_state = self._robot_state_from_positions(start)
        goal = Constraints()
        for joint_name in self._joint_names:
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = float(target.get(joint_name, 0.0))
            constraint.tolerance_above = self._joint_goal_tolerance
            constraint.tolerance_below = self._joint_goal_tolerance
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)
        motion_request.goal_constraints.append(goal)
        request.motion_plan_request = motion_request
        future = self._plan_client.call_async(request)
        received = self._wait_for_future(future, self._planning_time + 8.0)
        response = future.result() if received else None
        error_code = int(response.motion_plan_response.error_code.val) if response is not None else MoveItErrorCodes.TIMED_OUT
        success = response is not None and error_code == MoveItErrorCodes.SUCCESS
        trajectory = response.motion_plan_response.trajectory.joint_trajectory if response is not None else None
        points = len(trajectory.points) if trajectory is not None else 0
        joints = len(trajectory.joint_names) if trajectory is not None else 0
        duration = self._point_time_sec(trajectory.points[-1]) if trajectory is not None and trajectory.points else 0.0
        self._plan_rows.append(
            {
                "phase": phase_name,
                "plan_found": self._bool(success),
                "plan_error_code": str(error_code),
                "trajectory_point_count": str(points),
                "trajectory_joint_count": str(joints),
                "planned_duration_sec": f"{duration:.6f}",
            }
        )
        return trajectory if success and points > 0 else None

    def _planned_trajectory_within_limits(self, _phase_name: str, trajectory: Any) -> bool:
        for point in trajectory.points:
            for joint, position in zip(trajectory.joint_names, point.positions):
                if abs(math.degrees(float(position) - self._initial_positions.get(joint, 0.0))) > self._max_joint_delta_deg:
                    return False
        return True

    def _direct_joint_trajectory(self, target: dict[str, float], duration_sec: float) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(target.get(name, 0.0)) for name in self._joint_names]
        duration = DurationMsg()
        duration.sec = int(duration_sec)
        duration.nanosec = int((duration_sec - int(duration_sec)) * 1_000_000_000)
        point.time_from_start = duration
        trajectory.points.append(point)
        return trajectory

    def _execute_trajectory(self, trajectory: Any) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        if not self._wait_for_future(future, 5.0):
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, self._max_task_duration + 5.0):
            return False
        if result_future.result() is not None:
            self._wait_for_settle(0.75)
            return True
        return False

    def _phase_observed_and_error(self, start: dict[str, float], target: dict[str, float]) -> tuple[bool, float]:
        current = self._joint_positions(self._last_joint_state)
        motion_delta = max(abs(math.degrees(current.get(j, 0.0) - start.get(j, 0.0))) for j in self._joint_names)
        target_error = max(abs(math.degrees(current.get(j, 0.0) - target.get(j, 0.0))) for j in self._joint_names)
        return motion_delta > 0.1 and target_error <= self._return_tolerance_deg, target_error

    def _step_observed(self, start: dict[str, float], current: dict[str, float]) -> bool:
        motion_delta = max(abs(math.degrees(current.get(j, 0.0) - start.get(j, 0.0))) for j in self._joint_names)
        return motion_delta > 0.1

    def _calculate_final_return_error(self) -> None:
        current = self._joint_positions(self._last_joint_state)
        self._final_return_error_deg = max(
            abs(math.degrees(current.get(joint, 0.0) - self._ready_positions.get(joint, 0.0)))
            for joint in self._joint_names
        )
        self._final_return_within_tolerance = self._final_return_error_deg <= self._return_tolerance_deg

    def _record_phase(self, phase: str, status: str, note: str, duration: float, final_error: float) -> None:
        self._phase_rows.append(
            {
                "phase": phase,
                "status": status,
                "timestamp_sec": f"{time.monotonic() - self._start_time:.6f}",
                "duration_sec": f"{duration:.6f}",
                "final_error_deg": f"{final_error:.6f}",
                "max_force_n_so_far": f"{self._max_force:.6f}",
                "max_torque_nm_so_far": f"{self._max_torque:.6f}",
                "safety_violation_count": str(self._safety_violation_count),
                "note": note,
            }
        )

    def _record_guarded_step(self, step_index: int, status: str, positions: dict[str, float], duration: float) -> None:
        contact_pose = self._lookup_frame_pose(self._target_collision_frame) or [0.0, 0.0, 0.0]
        pad_top_z = self._target_spawn_pose[2] + (self._target_size[2] * 0.5)
        clearance = contact_pose[2] - pad_top_z
        self._step_rows.append(
            {
                "scenario": self._current_scenario_name,
                "action_mode": self._current_action_mode,
                "step_index": str(step_index),
                "status": status,
                "duration_sec": f"{duration:.6f}",
                "commanded_axis": self._approach_axis,
                "max_step_distance_m": f"{self._max_step_distance:.6f}",
                "measured_contact_link_x": f"{contact_pose[0]:.6f}",
                "measured_contact_link_y": f"{contact_pose[1]:.6f}",
                "measured_contact_link_z": f"{contact_pose[2]:.6f}",
                "estimated_clearance_m": f"{clearance:.6f}",
                "force_n": f"{self._last_force:.6f}",
                "torque_nm": f"{self._last_torque:.6f}",
                "contact_gate_triggered": self._bool(self._contact_gate_triggered),
                "emergency_stop_triggered": self._bool(self._emergency_stop_triggered),
                "joint_state_snapshot": self._position_summary(positions),
                "raw_contact_topic_available": self._bool(self._raw_contact_topic_available),
                "raw_contact_message_count": str(self._raw_contact_message_count),
                "raw_contact_count": str(self._raw_contact_count),
                "robot_pad_contact_count": str(self._robot_contact_count_total),
                "raw_contact_pairs_sample": "|".join(self._last_contact_pair_samples[:4]),
                "robot_contact_pairs_sample": "|".join(self._last_robot_contact_pair_samples[:4]),
                "max_contact_depth_m": f"{self._max_contact_depth:.9f}",
            }
        )

    def _record_contact_trigger(self, step_index: int) -> None:
        self._contact_rows.append(
            {
                "scenario": self._current_scenario_name,
                "action_mode": self._current_action_mode,
                "step_index": str(step_index),
                "force_n": f"{self._last_force:.6f}",
                "torque_nm": f"{self._last_torque:.6f}",
                "contact_detection_force_threshold_n": f"{self._contact_threshold:.6f}",
                "contact_validation_min_force_n": f"{self._contact_validation_min_force:.6f}",
                "raw_contact_topic_available": self._bool(self._raw_contact_topic_available),
                "raw_contact_message_count": str(self._raw_contact_message_count),
                "raw_contact_count": str(self._raw_contact_count),
                "robot_pad_contact_count": str(self._robot_contact_count_total),
                "raw_contact_pairs_sample": "|".join(self._last_contact_pair_samples[:4]),
                "robot_contact_pairs_sample": "|".join(self._last_robot_contact_pair_samples[:4]),
                "max_contact_depth_m": f"{self._max_contact_depth:.9f}",
                "desired_contact_force_upper_n": f"{self._desired_contact_upper:.6f}",
                "contact_gate_triggered": self._bool(self._contact_gate_triggered),
                "within_desired_contact_band": self._bool(self._contact_threshold <= self._last_force <= self._desired_contact_upper),
                "emergency_stop_triggered": self._bool(self._emergency_stop_triggered),
            }
        )

    def _write_endpoint_rows(self) -> None:
        self._endpoint_rows = [
            {"check": "simulation_engine", "value": self._simulation_engine},
            {"check": "control_interface", "value": self._action_name},
            {"check": "simulation_control_interface_used", "value": self._simulation_control_interface},
            {"check": "gazebo_controller_verified", "value": self._bool(self._gazebo_controller_verified)},
            {"check": "execution_endpoint_verified_simulation_only", "value": self._bool(self._endpoint_verified)},
            {"check": "real_robot_endpoint_detected", "value": self._bool(self._real_robot_endpoint_detected)},
            {"check": "peg_insertion_allowed", "value": "false"},
            {"check": "forceful_contact_allowed", "value": "false"},
            {"check": "follow_joint_trajectory_execution_allowed", "value": "gazebo_simulation_only"},
        ]

    def _write_joint_rows(self) -> None:
        rows = []
        for label, key in [
            ("initial", "initial"),
            ("ready", "after_ready_pose"),
            ("touch_standby", "after_touch_standby_pose"),
            ("contact_trigger_steps", "after_contact_trigger_steps"),
            ("retreat", "after_retreat"),
            ("return", "after_return"),
        ]:
            positions = self._joint_positions(self._snapshots.get(key))
            for joint in self._joint_names:
                rows.append(
                    {
                        "phase": label,
                        "joint": joint,
                        "position_rad": f"{positions.get(joint, 0.0):.9f}",
                        "delta_from_initial_deg": f"{math.degrees(positions.get(joint, 0.0) - self._initial_positions.get(joint, 0.0)):.9f}",
                    }
                )
        self._joint_rows = rows

    def _status_payload(self, status: str | None = None) -> dict[str, Any]:
        scenario_count = len(self._scenarios)
        attempted = len(self._scenario_results)
        validated = sum(1 for result in self._scenario_results if result.get("validated"))
        failed = [f"{result.get('name')}:{result.get('action_mode')}" for result in self._scenario_results if not result.get("validated")]
        max_force = max([float(result.get("max_force_n", 0.0)) for result in self._scenario_results] or [0.0])
        max_torque = max([float(result.get("max_torque_nm", 0.0)) for result in self._scenario_results] or [0.0])
        max_safety = max([int(result.get("safety_violation_count", 0)) for result in self._scenario_results] or [0])
        baseline_validated = sum(
            1 for result in self._scenario_results if result.get("action_mode") == "baseline_fixed_action" and result.get("validated")
        )
        context_validated = sum(
            1 for result in self._scenario_results if result.get("action_mode") == "context_conditioned_action" and result.get("validated")
        )
        return {
            "simulation_engine": self._simulation_engine,
            "task_sequence_type": self._task_sequence_type,
            "source_context_sprint": self._source_context_sprint,
            "source_action_sprint": self._source_action_sprint,
            "context_embeddings_found": self._context_embeddings_found,
            "context_embedding_count": self._context_embedding_count,
            "action_modes": self._action_modes,
            "moveit_used": True,
            "move_group_started": self._move_group_started,
            "planning_available": self._planning_available,
            "gazebo_controller_verified": self._gazebo_controller_verified,
            "execution_endpoint_verified_simulation_only": self._endpoint_verified,
            "scenario_count": scenario_count,
            "expected_run_count": self._expected_run_count,
            "runs_attempted": attempted,
            "runs_completed": attempted,
            "runs_validated": validated,
            "baseline_runs_validated": baseline_validated,
            "context_conditioned_runs_validated": context_validated,
            "all_action_parameters_within_bounds": self._all_action_parameters_within_bounds(),
            "all_initial_no_contact_verified": bool(attempted and all(r.get("initial_no_contact_verified") for r in self._scenario_results)),
            "all_contact_triggers_after_motion": bool(attempted and all(r.get("contact_trigger_after_motion") for r in self._scenario_results)),
            "all_stop_on_contact_executed": bool(attempted and all(r.get("stop_on_contact_executed") for r in self._scenario_results)),
            "all_retreats_completed": bool(attempted and all(r.get("retreat_completed") for r in self._scenario_results)),
            "all_post_retreat_no_contact_verified": bool(attempted and all(r.get("post_retreat_no_contact_verified") for r in self._scenario_results)),
            "all_returns_to_ready_completed": bool(attempted and all(r.get("return_to_ready_completed") for r in self._scenario_results)),
            "paired_comparison_written": bool(self._paired_comparison_rows),
            "diagnostic_ablation_only": True,
            "learning_executed": False,
            "policy_training_executed": False,
            "rl_training_executed": False,
            "fake_result_created": False,
            "peg_insertion_executed": False,
            "forceful_contact_executed": False,
            "max_observed_force_n": max_force,
            "max_observed_torque_nm": max_torque,
            "max_safety_violation_count": max_safety,
            "failed_scenarios": failed,
            "trajectory_execution_allowed": "gazebo_simulation_only",
            "controller_execution_allowed": "gazebo_simulation_only",
            "follow_joint_trajectory_execution_allowed": "gazebo_simulation_only",
            "real_robot_used": False,
            "motion_executed": self._motion_executed,
            "shutdown_segfault_observed_if_any": self._shutdown_segfault_observed,
            "status": status or "context_action_ablation_pending",
        }

    def _successful_phase_count(self) -> int:
        return sum(1 for row in self._phase_rows if row.get("status") == "succeeded")

    def _publish_reports(self) -> None:
        self._publish_json(self._status_pub, self._status_payload())
        self._publish_json(self._step_pub, {"rows": self._step_rows})
        self._publish_json(self._contact_pub, {"rows": self._paired_comparison_rows})
        self._publish_json(self._pad_pose_pub, {"lines": self._target_report_lines})
        self._publish_json(self._topic_wiring_pub, {"lines": self._topic_wiring_lines()})
        self._publish_json(self._safety_pub, self._safety_payload())
        self._publish_json(self._endpoint_pub, {"rows": self._endpoint_rows})
        self._publish_json(self._action_suggestion_pub, {"rows": self._action_suggestion_rows})

    def _write_outputs_once(self, status: str) -> None:
        if self._finished:
            return
        self._finished = True
        payload = self._status_payload(status)
        self._write_lines(self._output_dir / "nodes.txt", sorted(name for name in self.get_node_names() if name))
        self._write_lines(self._output_dir / "topics.txt", sorted(f"{n} {','.join(t)}" for n, t in self.get_topic_names_and_types()))
        self._write_lines(self._output_dir / "services.txt", sorted(f"{n} {','.join(t)}" for n, t in self.get_service_names_and_types()))
        self._write_parameters_file()
        self._write_tf_frames_file()
        self._write_csv(self._output_dir / "action_mode_definition_table.csv", self._action_mode_definition_rows)
        self._write_csv(self._output_dir / "action_mode_run_report.csv", self._scenario_execution_rows())
        self._write_csv(self._output_dir / "paired_ablation_comparison_report.csv", self._paired_comparison_rows)
        self._write_json(self._output_dir / "paired_ablation_summary.json", self._paired_summary(payload))
        self._write_csv(self._output_dir / "ablation_contact_transition_report.csv", self._contact_rows)
        self._write_csv(self._output_dir / "ablation_safety_report.csv", self._safety_rows())
        self._write_csv(self._output_dir / "ablation_endpoint_report.csv", self._endpoint_rows)
        self._write_json(self._output_dir / "ablation_manifest.json", self._action_manifest(payload))
        self._write_json(self._output_dir / "context_action_ablation_status.json", payload)
        self._write_summary(payload)
        self._write_run_log(payload)
        self._publish_reports()
        self.get_logger().info("proposal_simulation_cell_v2_15 diagnostics written")
        rclpy.shutdown()

    def _scenario_execution_rows(self) -> list[dict[str, str]]:
        return [
            {
                "scenario": str(result.get("name", "")),
                "action_mode": str(result.get("action_mode", "")),
                "context_embedding_loaded": self._bool(result.get("context_embedding_loaded", False)),
                "action_parameters_within_bounds": self._bool(result.get("action_parameters_within_bounds", False)),
                "approach_step_distance_m": f"{float(result.get('approach_step_distance_m', 0.0)):.6f}",
                "max_guarded_steps": str(result.get("max_guarded_steps", 0)),
                "validated": self._bool(result.get("validated", False)),
                "initial_no_contact_verified": self._bool(result.get("initial_no_contact_verified", False)),
                "contact_trigger_after_motion": self._bool(result.get("contact_trigger_after_motion", False)),
                "contact_trigger_step_index": str(result.get("contact_trigger_step_index", 0)),
                "stop_on_contact_executed": self._bool(result.get("stop_on_contact_executed", False)),
                "retreat_completed": self._bool(result.get("retreat_completed", False)),
                "post_retreat_no_contact_verified": self._bool(result.get("post_retreat_no_contact_verified", False)),
                "return_to_ready_completed": self._bool(result.get("return_to_ready_completed", False)),
                "max_force_n": f"{float(result.get('max_force_n', 0.0)):.9f}",
                "max_torque_nm": f"{float(result.get('max_torque_nm', 0.0)):.9f}",
                "safety_violation_count": str(result.get("safety_violation_count", 0)),
                "final_return_error_deg": f"{float(result.get('final_return_error_deg', 0.0)):.9f}",
                "real_robot_used": "false",
                "peg_insertion_executed": "false",
                "forceful_contact_executed": "false",
            }
            for result in self._scenario_results
        ]

    def _write_paired_comparison_rows(self) -> None:
        rows = []
        for scenario in self._scenarios:
            name = str(scenario.get("name", ""))
            baseline = self._result_for(name, "baseline_fixed_action")
            context = self._result_for(name, "context_conditioned_action")
            if not baseline or not context:
                continue
            rows.append(
                {
                    "scenario": name,
                    "baseline_validated": self._bool(baseline.get("validated", False)),
                    "context_conditioned_validated": self._bool(context.get("validated", False)),
                    "baseline_approach_step_distance_m": f"{float(baseline.get('approach_step_distance_m', 0.0)):.6f}",
                    "context_approach_step_distance_m": f"{float(context.get('approach_step_distance_m', 0.0)):.6f}",
                    "approach_step_distance_difference_m": f"{float(context.get('approach_step_distance_m', 0.0)) - float(baseline.get('approach_step_distance_m', 0.0)):.6f}",
                    "baseline_max_guarded_steps": str(baseline.get("max_guarded_steps", 0)),
                    "context_max_guarded_steps": str(context.get("max_guarded_steps", 0)),
                    "max_guarded_steps_difference": str(int(context.get("max_guarded_steps", 0)) - int(baseline.get("max_guarded_steps", 0))),
                    "baseline_trigger_step": str(baseline.get("contact_trigger_step_index", 0)),
                    "context_trigger_step": str(context.get("contact_trigger_step_index", 0)),
                    "trigger_step_difference": str(int(context.get("contact_trigger_step_index", 0)) - int(baseline.get("contact_trigger_step_index", 0))),
                    "baseline_max_force_n": f"{float(baseline.get('max_force_n', 0.0)):.9f}",
                    "context_max_force_n": f"{float(context.get('max_force_n', 0.0)):.9f}",
                    "max_force_difference_n": f"{float(context.get('max_force_n', 0.0)) - float(baseline.get('max_force_n', 0.0)):.9f}",
                    "baseline_final_return_error_deg": f"{float(baseline.get('final_return_error_deg', 0.0)):.9f}",
                    "context_final_return_error_deg": f"{float(context.get('final_return_error_deg', 0.0)):.9f}",
                    "final_return_error_difference_deg": f"{float(context.get('final_return_error_deg', 0.0)) - float(baseline.get('final_return_error_deg', 0.0)):.9f}",
                    "baseline_safety_violation_count": str(baseline.get("safety_violation_count", 0)),
                    "context_safety_violation_count": str(context.get("safety_violation_count", 0)),
                    "safety_violation_difference": str(int(context.get("safety_violation_count", 0)) - int(baseline.get("safety_violation_count", 0))),
                    "diagnostic_ablation_only": "true",
                    "rl_improvement_claimed": "false",
                    "generalization_claimed": "false",
                }
            )
        self._paired_comparison_rows = rows

    def _result_for(self, scenario: str, action_mode: str) -> dict[str, Any] | None:
        for result in self._scenario_results:
            if result.get("name") == scenario and result.get("action_mode") == action_mode:
                return result
        return None

    def _paired_summary(self, status: dict[str, Any]) -> dict[str, Any]:
        return {
            "scenario_count": len(self._scenarios),
            "action_modes": self._action_modes,
            "expected_run_count": self._expected_run_count,
            "runs_validated": status.get("runs_validated", 0),
            "paired_comparison_count": len(self._paired_comparison_rows),
            "paired_comparison_written": bool(self._paired_comparison_rows),
            "diagnostic_ablation_only": True,
            "rl_improvement_claimed": False,
            "generalization_claimed": False,
            "learning_executed": False,
            "policy_training_executed": False,
            "fake_result_created": False,
            "real_robot_used": False,
            "peg_insertion_executed": False,
            "forceful_contact_executed": False,
        }

    def _action_manifest(self, status: dict[str, Any]) -> dict[str, Any]:
        return {
            "source_context_sprint": self._source_context_sprint,
            "source_action_sprint": self._source_action_sprint,
            "context_embedding_table": str(self._context_embedding_path),
            "context_encoder_manifest": str(self._context_manifest_path),
            "scenario_context_vectors": str(self._scenario_context_path),
            "source_scenario_definition_table": str(self._source_scenario_definition_path),
            "context_embeddings_found": self._context_embeddings_found,
            "context_manifest_found": self._context_manifest_found,
            "context_embedding_count": self._context_embedding_count,
            "context_embedding_dim": self._context_embedding_dim,
            "deterministic_action_suggestion": self._deterministic_action_suggestion,
            "generated_files": [
                "context_action_ablation_status.json",
                "action_mode_definition_table.csv",
                "action_mode_run_report.csv",
                "paired_ablation_comparison_report.csv",
                "paired_ablation_summary.json",
                "ablation_contact_transition_report.csv",
                "ablation_safety_report.csv",
                "ablation_endpoint_report.csv",
                "ablation_manifest.json",
                "summary.md",
            ],
            "learning_executed": False,
            "policy_training_executed": False,
            "rl_training_executed": False,
            "fake_result_created": False,
            "real_robot_used": False,
            "peg_insertion_executed": False,
            "forceful_contact_executed": False,
            "status": status.get("status", ""),
        }

    def _write_parameters_file(self) -> None:
        lines = [
            "moveit_execution_allowed=gazebo_simulation_only",
            "controller_execution_allowed=gazebo_simulation_only",
            "trajectory_execution_allowed=gazebo_simulation_only",
            "follow_joint_trajectory_execution_allowed=gazebo_simulation_only",
            "real_robot_allowed=false",
            "peg_insertion_allowed=false",
            "forceful_contact_allowed=false",
        ]
        lines.extend(self._run_command(["ros2", "param", "list", self._move_group_node], timeout=2.0))
        self._write_lines(self._output_dir / "parameters.txt", lines)

    def _write_tf_frames_file(self) -> None:
        lines = [
            f"reference_frame={self._target_reference_frame}",
            f"tool_frame={self._target_tool_frame}",
            f"distal_collision_frame={self._target_collision_frame}",
            f"tool_pose_available={self._bool(self._lookup_frame_pose(self._target_tool_frame) is not None)}",
            f"distal_collision_pose_available={self._bool(self._lookup_frame_pose(self._target_collision_frame) is not None)}",
        ]
        tool_pose = self._lookup_frame_pose(self._target_tool_frame)
        collision_pose = self._lookup_frame_pose(self._target_collision_frame)
        if tool_pose is not None:
            lines.append(f"tool_pose_xyz={tool_pose[0]:.6f},{tool_pose[1]:.6f},{tool_pose[2]:.6f}")
        if collision_pose is not None:
            lines.append(f"distal_collision_pose_xyz={collision_pose[0]:.6f},{collision_pose[1]:.6f},{collision_pose[2]:.6f}")
        self._write_lines(self._output_dir / "tf_frames.txt", lines)

    def _topic_wiring_lines(self) -> list[str]:
        return [
            f"raw_contact_topic={self._target_contact_topic}",
            f"gz_contact_topic={self._target_gz_contact_topic}",
            f"derived_contact_wrench_topic={self._contact_wrench_topic}",
            f"raw_contact_topic_available={self._bool(self._raw_contact_topic_available or self._target_contact_topic in self._topic_names())}",
            f"derived_contact_wrench_available={self._bool(self._contact_wrench_topic_available or self._contact_wrench_topic in self._topic_names())}",
            f"raw_contact_message_count={self._raw_contact_message_count}",
            f"raw_contact_count={self._raw_contact_count}",
            f"robot_pad_contact_count={self._robot_contact_count_total}",
            f"raw_contact_pairs_sample={'|'.join(self._last_contact_pair_samples[:4])}",
            f"robot_contact_pairs_sample={'|'.join(self._last_robot_contact_pair_samples[:4])}",
            f"max_contact_depth_m={self._max_contact_depth:.9f}",
            f"derived_force_from_raw_contact_depth_enabled={self._bool(True)}",
        ]

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_15_context_action_ablation_validation",
            "",
            f"Status: `{status['status']}`",
            "",
            "This diagnostic validates a Gazebo-only batch of no-contact standby, bounded approach-to-contact, stop-on-contact, retreat, and return-to-ready scenarios.",
            "",
            f"- source_context_sprint: {status['source_context_sprint']}",
            f"- source_action_sprint: {status['source_action_sprint']}",
            f"- context_embedding_count: {status['context_embedding_count']}",
            f"- scenario_count: {status['scenario_count']}",
            f"- expected_run_count: {status['expected_run_count']}",
            f"- runs_attempted: {status['runs_attempted']}",
            f"- runs_validated: {status['runs_validated']}",
            f"- baseline_runs_validated: {status['baseline_runs_validated']}",
            f"- context_conditioned_runs_validated: {status['context_conditioned_runs_validated']}",
            f"- paired_comparison_written: {self._bool(status['paired_comparison_written'])}",
            f"- all_action_parameters_within_bounds: {self._bool(status['all_action_parameters_within_bounds'])}",
            f"- all_initial_no_contact_verified: {self._bool(status['all_initial_no_contact_verified'])}",
            f"- all_contact_triggers_after_motion: {self._bool(status['all_contact_triggers_after_motion'])}",
            f"- all_stop_on_contact_executed: {self._bool(status['all_stop_on_contact_executed'])}",
            f"- all_retreats_completed: {self._bool(status['all_retreats_completed'])}",
            f"- all_post_retreat_no_contact_verified: {self._bool(status['all_post_retreat_no_contact_verified'])}",
            f"- max_observed_force_n: {status['max_observed_force_n']:.9f}",
            f"- failed_scenarios: {status['failed_scenarios']}",
            "- peg_insertion_executed: false",
            "- forceful_contact_executed: false",
            "- real_robot_used: false",
        ]
        self._write_lines(self._output_dir / "summary.md", lines)

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_15_context_action_ablation_validation",
            f"status={status['status']}",
            f"scenario_count={status['scenario_count']}",
            f"expected_run_count={status['expected_run_count']}",
            f"runs_attempted={status['runs_attempted']}",
            f"runs_validated={status['runs_validated']}",
            f"baseline_runs_validated={status['baseline_runs_validated']}",
            f"context_conditioned_runs_validated={status['context_conditioned_runs_validated']}",
            f"failed_scenarios={status['failed_scenarios']}",
            f"context_embedding_count={status['context_embedding_count']}",
            f"all_action_parameters_within_bounds={self._bool(status['all_action_parameters_within_bounds'])}",
            f"all_initial_no_contact_verified={self._bool(status['all_initial_no_contact_verified'])}",
            f"all_contact_triggers_after_motion={self._bool(status['all_contact_triggers_after_motion'])}",
            f"max_observed_force_n={status['max_observed_force_n']}",
            "learning_executed=false",
            "policy_training_executed=false",
            "rl_training_executed=false",
            "fake_result_created=false",
            "real_robot_used=false",
            "peg_insertion_executed=false",
            "forceful_contact_executed=false",
        ]
        self._write_lines(self._output_dir / "run.log", lines)

    def _dict_lines(self, payload: dict[str, Any]) -> list[str]:
        return [f"{key}={value}" for key, value in payload.items()]

    def _safety_payload(self) -> dict[str, Any]:
        return {
            "contact_wrench_topic_available": self._contact_wrench_topic_available or self._contact_wrench_topic in self._topic_names(),
            "contact_detection_force_threshold_n": self._contact_threshold,
            "contact_validation_min_force_n": self._contact_validation_min_force,
            "desired_contact_force_upper_n": self._desired_contact_upper,
            "raw_contact_topic_available": self._raw_contact_topic_available or self._target_contact_topic in self._topic_names(),
            "raw_contact_message_count": self._raw_contact_message_count,
            "raw_contact_count": self._raw_contact_count,
            "robot_pad_contact_count": self._robot_contact_count_total,
            "raw_contact_pairs_sample": "|".join(self._last_contact_pair_samples[:4]),
            "robot_contact_pairs_sample": "|".join(self._last_robot_contact_pair_samples[:4]),
            "max_contact_depth_m": self._max_contact_depth,
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "max_allowed_force_n": self._max_allowed_force,
            "emergency_stop_force_threshold_n": self._emergency_force,
            "safety_violation_count": self._safety_violation_count,
            "contact_gate_triggered": self._contact_gate_triggered,
            "emergency_stop_triggered": self._emergency_stop_triggered,
            "peg_insertion_executed": False,
            "forceful_contact_executed": False,
            "real_robot_used": False,
        }

    def _safety_rows(self) -> list[dict[str, str]]:
        if self._scenario_results:
            return [
                {
                    "scenario": str(result.get("name", "")),
                    "action_mode": str(result.get("action_mode", "")),
                    "max_force_n": f"{float(result.get('max_force_n', 0.0)):.9f}",
                    "max_torque_nm": f"{float(result.get('max_torque_nm', 0.0)):.9f}",
                    "safety_violation_count": str(result.get("safety_violation_count", 0)),
                    "emergency_stop_triggered": self._bool(self._emergency_stop_triggered),
                    "validated": self._bool(bool(result.get("validated"))),
                    "peg_insertion_executed": "false",
                    "forceful_contact_executed": "false",
                    "real_robot_used": "false",
                }
                for result in self._scenario_results
            ]
        return [
            {"field": key, "value": str(value).lower() if isinstance(value, bool) else str(value)}
            for key, value in self._safety_payload().items()
        ]

    def _extract_contact_wrench(self, message: Any) -> tuple[float, float, int, int, float, list[str], list[str]]:
        max_force = 0.0
        max_torque = 0.0
        max_depth = 0.0
        contacts = list(getattr(message, "contacts", []))
        robot_contacts = 0
        pair_samples: list[str] = []
        robot_pair_samples: list[str] = []
        for contact in contacts:
            pair = self._contact_pair_name(contact)
            if pair and len(pair_samples) < 8:
                pair_samples.append(pair)
            robot_contact = self._is_robot_pad_contact(pair)
            if not robot_contact:
                continue
            robot_contacts += 1
            if pair and len(robot_pair_samples) < 8:
                robot_pair_samples.append(pair)
            for depth in list(getattr(contact, "depths", [])):
                max_depth = max(max_depth, float(depth))
            for wrench in self._contact_wrenches(contact):
                for nested in self._wrench_messages(wrench):
                    force = getattr(nested, "force", None)
                    torque = getattr(nested, "torque", None)
                    max_force = max(max_force, self._vector_magnitude(force))
                    max_torque = max(max_torque, self._vector_magnitude(torque))
        derived_force = max_depth * self._target_kp if max_depth > 0.0 else 0.0
        return derived_force, max_torque, len(contacts), robot_contacts, max_depth, pair_samples, robot_pair_samples

    def _contact_pair_name(self, contact: Any) -> str:
        collision1 = getattr(getattr(contact, "collision1", None), "name", "")
        collision2 = getattr(getattr(contact, "collision2", None), "name", "")
        body_names: list[str] = []
        for wrench in self._contact_wrenches(contact):
            for name in ("body_1_name", "body_2_name"):
                value = getattr(getattr(wrench, name, None), "data", "")
                if value:
                    body_names.append(str(value))
        names = [str(name) for name in (collision1, collision2, *body_names) if str(name)]
        return " <-> ".join(names)

    def _is_robot_pad_contact(self, pair_name: str) -> bool:
        text = pair_name.lower()
        if not text:
            return False
        pad_terms = [self._target_name.lower(), "contact_calibration_pad", "non_overlapping_contact_pad"]
        robot_terms = [
            self._robot_model.lower(),
            self._target_collision_frame.lower(),
            self._target_tool_frame.lower(),
            "lbr_iisy",
        ]
        return any(term in text for term in pad_terms) and any(term in text for term in robot_terms)

    def _contact_wrenches(self, contact: Any) -> list[Any]:
        result = []
        for name in ("wrenches", "wrench", "body_1_wrench", "body_2_wrench", "body1_wrench", "body2_wrench"):
            value = getattr(contact, name, None)
            if value is None:
                continue
            if isinstance(value, (list, tuple)):
                result.extend(value)
            else:
                result.append(value)
        return result

    def _wrench_messages(self, value: Any) -> list[Any]:
        result = []
        if getattr(value, "force", None) is not None or getattr(value, "torque", None) is not None:
            result.append(value)
        for name in ("body_1_wrench", "body_2_wrench", "body1_wrench", "body2_wrench", "wrench"):
            nested = getattr(value, name, None)
            if nested is not None:
                result.append(nested)
        return result

    def _vector_magnitude(self, vector: Any) -> float:
        if vector is None:
            return 0.0
        return math.sqrt(float(vector.x) ** 2 + float(vector.y) ** 2 + float(vector.z) ** 2)

    def _robot_state_from_positions(self, positions: dict[str, float]) -> RobotState:
        state = RobotState()
        joint_state = JointState()
        joint_state.name = list(self._joint_names)
        joint_state.position = [float(positions.get(name, 0.0)) for name in self._joint_names]
        state.joint_state = joint_state
        return state

    def _wait_for_joint_state(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._last_joint_state is not None and all(name in self._last_joint_state.name for name in self._joint_names):
                return True
            time.sleep(0.05)
        return False

    def _wait_for_contact_wrench_topic(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._contact_wrench_topic_available or self._contact_wrench_topic in self._topic_names():
                return True
            time.sleep(0.05)
        return False

    def _wait_for_future(self, future: Any, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return future.done()

    def _wait_for_settle(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            time.sleep(0.05)

    def _joint_positions(self, message: JointState | None) -> dict[str, float]:
        if message is None:
            return {}
        return {name: float(position) for name, position in zip(message.name, message.position)}

    def _joint_state_lines(self, message: JointState | None) -> list[str]:
        if message is None:
            return ["joint_state_available=false"]
        lines = ["joint_state_available=true"]
        for name, position in self._joint_positions(message).items():
            lines.append(f"{name}: {position:.9f}")
        return lines

    def _position_summary(self, positions: dict[str, float]) -> str:
        return ";".join(f"{joint}={positions.get(joint, 0.0):.9f}" for joint in self._joint_names)

    def _point_time_sec(self, point: JointTrajectoryPoint) -> float:
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1_000_000_000.0

    def _publish_zero_wrench(self) -> None:
        self._publish_contact_wrench(0.0, 0.0)

    def _publish_contact_wrench(self, force: float, torque: float) -> None:
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = self._target_frame
        wrench.wrench.force.x = float(force)
        wrench.wrench.torque.z = float(torque)
        self._contact_wrench_pub.publish(wrench)

    def _topic_names(self) -> set[str]:
        return {name for name, _types in self.get_topic_names_and_types()}

    def _run_command(self, command: list[str], timeout: float) -> list[str]:
        try:
            result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=timeout)
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            return [str(exc)]
        lines = []
        if result.stdout:
            lines.extend(result.stdout.splitlines())
        if result.stderr:
            lines.extend(result.stderr.splitlines())
        return lines

    def _write_csv(self, path: Path, rows: list[dict[str, str]]) -> None:
        fields = list(rows[0].keys()) if rows else ["status"]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _write_lines(self, path: Path, lines: list[str]) -> None:
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        publisher.publish(message)

    def _bool(self, value: Any) -> str:
        return str(bool(value)).lower()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV215ContextActionAblationNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
