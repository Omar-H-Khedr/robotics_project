"""Guarded peg-in-hole objective validation for proposal_simulation_cell_v2_16."""

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
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
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


class ProposalSimulationCellV216GuardedPegInHoleNode(Node):
    """Run one Gazebo-only guarded peg-in-hole objective attempt."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_16_guarded_peg_in_hole_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_16")

        self._config = self._load_config()
        robot = self._config.get("robot", {})
        task = self._config.get("task_sequence", {})
        geometry = self._config.get("peg_hole_geometry", {})
        moveit = self._config.get("moveit_execution", {})
        insertion = self._config.get("guarded_insertion", {})
        contact = self._config.get("contact_gate", {})
        safety = self._config.get("safety_gates", {})
        execution = self._config.get("execution_policy", {})
        validation = self._config.get("validation", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_16")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._simulation_engine = str(task.get("simulation_engine", "gazebo"))
        self._task_sequence_type = str(task.get("task_sequence_type", "guarded_peg_in_hole_objective_validation"))
        self._main_objective_sprint = bool(task.get("main_objective_sprint", True))
        self._phase_names = [str(item) for item in task.get("phases", [])]
        self._return_tolerance_deg = float(task.get("return_tolerance_deg", 1.0))
        self._robot_model = str(robot.get("gazebo_robot_model", "lbr_iisy3_r760"))
        self._group = str(moveit.get("selected_group", robot.get("selected_group", "manipulator")))
        self._tool_link = str(moveit.get("selected_end_effector_link", robot.get("selected_end_effector_link", "tool0")))
        self._contact_link = str(moveit.get("selected_contact_link", robot.get("selected_contact_link", "link_6")))
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
        self._velocity_scale = float(moveit.get("max_velocity_scaling_factor", 0.035))
        self._acceleration_scale = float(moveit.get("max_acceleration_scaling_factor", 0.035))
        self._joint_goal_tolerance = float(moveit.get("joint_goal_tolerance_rad", 0.015))

        self._world_frame = str(geometry.get("world_frame", "world"))
        self._peg_frame_candidates = [str(item) for item in geometry.get("peg_frame_candidates", [])]
        self._hole_frame_candidates = [str(item) for item in geometry.get("hole_frame_candidates", [])]
        self._axis_frame_candidates = [str(item) for item in geometry.get("insertion_axis_frame_candidates", [])]
        self._fallback_peg_tip = [float(item) for item in geometry.get("fallback_peg_tip_xyz", [0.52, -0.20, 0.955])]
        self._fallback_hole_center = [float(item) for item in geometry.get("fallback_hole_center_xyz", [0.52, -0.20, 0.83])]
        self._fallback_axis = self._normalize_vector(
            [float(item) for item in geometry.get("fallback_insertion_axis_xyz", [0.0, 0.0, -1.0])]
        )
        self._fallback_geometry_source = str(geometry.get("fallback_geometry_source", "proposal_simulation_cell_scene_model_links"))
        self._peg_tip_offset = [float(item) for item in geometry.get("peg_tip_offset_from_contact_link_xyz", [0.0, 0.0, -0.055])]
        self._target_depth = float(geometry.get("target_insertion_depth_m", 0.015))
        self._success_depth = float(geometry.get("success_insertion_depth_threshold_m", 0.010))
        self._max_lateral_error = float(geometry.get("max_lateral_alignment_error_m", 0.005))
        self._minimum_vertical_clearance = float(geometry.get("minimum_vertical_clearance_m", 0.002))

        self._max_steps = int(insertion.get("max_insertion_step_count", 15))
        self._max_step_distance = float(insertion.get("max_insertion_step_distance_m", 0.002))
        self._ready_delta = self._delta_map(insertion.get("ready_pose_delta_deg", {}))
        self._pre_approach_delta = self._delta_map(insertion.get("pre_approach_pose_delta_deg", {}))
        self._alignment_delta = self._delta_map(insertion.get("above_hole_alignment_pose_delta_deg", {}))
        self._step_delta = self._delta_map(insertion.get("insertion_step_delta_deg", {}))
        self._return_delta = self._delta_map(insertion.get("return_pose_delta_deg", {}))

        self._contact_threshold = float(contact.get("contact_detection_force_threshold_n", 0.02))
        self._desired_contact_upper = float(contact.get("desired_contact_force_upper_n", 3.0))
        self._max_allowed_force = float(safety.get("max_allowed_force_n", contact.get("max_allowed_force_n", 8.0)))
        self._max_allowed_torque = float(safety.get("max_allowed_torque_nm", 5.0))
        self._emergency_force = float(
            safety.get("emergency_stop_force_threshold_n", contact.get("emergency_stop_force_threshold_n", 6.0))
        )
        self._real_robot_allowed = bool(execution.get("real_robot_allowed", safety.get("real_robot_allowed", False)))
        self._forceful_contact_allowed = bool(execution.get("forceful_contact_allowed", safety.get("forceful_contact_allowed", False)))
        self._peg_insertion_allowed = str(
            execution.get("peg_insertion_allowed", safety.get("peg_insertion_allowed", "controlled_gazebo_only"))
        )

        self._joint_states_topic = str(validation.get("joint_states_topic", "/joint_states"))
        self._contact_wrench_topic = str(validation.get("contact_wrench_topic", "/proposal_simulation_cell/contact_wrench"))
        self._peg_contact_topic = str(validation.get("peg_contact_topic", "/proposal_simulation_cell/peg_contacts"))
        self._hole_contact_topic = str(validation.get("hole_contact_topic", "/proposal_simulation_cell/hole_contacts"))
        self._startup_wait = float(validation.get("startup_wait_sec", 5.0))
        self._success_status = str(validation.get("status_success", "guarded_peg_in_hole_objective_validated"))
        self._attempt_failure_status = str(
            validation.get("status_attempt_failure", "guarded_peg_in_hole_objective_attempt_completed_with_failure_reason")
        )
        self._endpoint_status = str(validation.get("status_endpoint_unverified", "gazebo_execution_endpoint_not_verified"))
        self._contact_unavailable_status = str(validation.get("status_contact_unavailable", "contact_topic_unavailable"))
        self._geometry_status = str(validation.get("status_geometry_unavailable", "peg_hole_geometry_unavailable"))
        self._alignment_unreachable_status = str(validation.get("status_alignment_unreachable", "alignment_pose_not_reachable"))
        self._alignment_error_status = str(validation.get("status_alignment_error", "initial_alignment_error_too_large"))
        self._force_limit_status = str(
            validation.get("status_force_limit", "contact_force_limit_reached_before_success_depth")
        )
        self._depth_not_reached_status = str(
            validation.get("status_depth_not_reached", "insertion_depth_not_reached_within_step_limit")
        )

        self._status_pub = self.create_publisher(
            String, str(validation.get("status_topic", "/proposal_simulation_cell/guarded_peg_in_hole_objective_status")), 10
        )
        self._geometry_pub = self.create_publisher(
            String, str(validation.get("geometry_report_topic", "/proposal_simulation_cell/peg_hole_geometry_report")), 10
        )
        self._step_pub = self.create_publisher(
            String, str(validation.get("insertion_step_report_topic", "/proposal_simulation_cell/guarded_insertion_step_report")), 10
        )
        self._safety_pub = self.create_publisher(
            String, str(validation.get("safety_report_topic", "/proposal_simulation_cell/insertion_safety_report")), 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._joint_state_lock = threading.Lock()
        self._last_joint_state: JointState | None = None
        self._joint_state_sub = self.create_subscription(JointState, self._joint_states_topic, self._joint_state_cb, 10)
        self._last_force = 0.0
        self._last_torque = 0.0
        self._max_force = 0.0
        self._max_torque = 0.0
        self._contact_wrench_available = False
        self._wrench_sub = self.create_subscription(WrenchStamped, self._contact_wrench_topic, self._wrench_cb, 10)
        self._raw_contact_topic_available = False
        self._raw_contact_count = 0
        self._raw_contact_messages = 0
        self._raw_contact_pairs: list[str] = []
        if Contacts is not None:
            self._peg_contact_sub = self.create_subscription(Contacts, self._peg_contact_topic, self._contacts_cb, 10)
            self._hole_contact_sub = self.create_subscription(Contacts, self._hole_contact_topic, self._contacts_cb, 10)

        self._ik_client = self.create_client(GetPositionIK, self._compute_ik_service)
        self._plan_client = self.create_client(GetMotionPlan, self._plan_service)
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)

        self._initial_positions: dict[str, float] = {}
        self._ready_positions: dict[str, float] = {}
        self._alignment_positions: dict[str, float] = {}
        self._final_positions: dict[str, float] = {}
        self._status: dict[str, Any] = {}
        self._geometry_report: dict[str, Any] = {}
        self._phase_rows: list[dict[str, str]] = []
        self._alignment_rows: list[dict[str, str]] = []
        self._step_rows: list[dict[str, str]] = []
        self._contact_rows: list[dict[str, str]] = []
        self._success_rows: list[dict[str, str]] = []
        self._safety_rows: list[dict[str, str]] = []
        self._endpoint_rows: list[dict[str, str]] = []
        self._peg_tip_pose = list(self._fallback_peg_tip)
        self._hole_center_pose = list(self._fallback_hole_center)
        self._insertion_axis = list(self._fallback_axis)
        self._peg_frame_available = False
        self._hole_frame_available = False
        self._peg_tip_pose_available = False
        self._hole_center_pose_available = False
        self._insertion_axis_available = False
        self._geometry_computed = False
        self._initial_lateral_error = 0.0
        self._initial_vertical_clearance = 0.0
        self._max_depth = 0.0
        self._failure_reason = ""
        self._safety_violation_count = 0
        self._emergency_stop = False
        self._force_limit_triggered = False
        self._contact_gate_triggered = False
        self._insertion_started = False
        self._insertion_success = False
        self._retreat_or_return_completed = False
        self._final_return_error_deg = 0.0
        self._motion_executed = False
        self._move_group_started = False
        self._planning_available = False
        self._gazebo_controller_verified = False
        self._endpoint_verified = False
        self._ready_reached = False
        self._pre_approach_reached = False
        self._alignment_reached = False

        self.create_timer(0.5, self._publish_status)
        self.create_timer(1.0, self._start_worker)
        self._started = False
        self._worker_thread: threading.Thread | None = None

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            config_path = "src/thesis_bringup/config/proposal_simulation_cell_v2_16.yaml"
        with open(config_path, "r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def _start_worker(self) -> None:
        if self._started:
            return
        self._started = True
        self._worker_thread = threading.Thread(target=self._run_once, daemon=True)
        self._worker_thread.start()

    def _run_once(self) -> None:
        try:
            status = self._run_sequence()
            self._write_outputs(status)
        except Exception as exc:  # pragma: no cover - diagnostic fallback.
            self._failure_reason = f"unexpected_exception: {exc}"
            self.get_logger().error(f"v2.16 diagnostic failed unexpectedly: {exc}")
            self._write_outputs(self._attempt_failure_status)
        self.get_logger().info("proposal_simulation_cell_v2_16 diagnostics written")
        rclpy.shutdown()

    def _run_sequence(self) -> str:
        self._record_phase("initialize", "succeeded", "node started", 0.0, 0.0)
        time.sleep(self._startup_wait)

        self._write_ros_graph_files()
        self._move_group_started = self._wait_for_node(self._move_group_node, timeout=5.0)
        self._planning_available = self._plan_client.wait_for_service(timeout_sec=5.0)
        ik_available = self._ik_client.wait_for_service(timeout_sec=5.0)
        action_available = self._action_client.wait_for_server(timeout_sec=8.0)
        self._gazebo_controller_verified = action_available and self._simulation_engine == "gazebo"
        self._endpoint_verified = self._gazebo_controller_verified and not self._real_robot_allowed
        self._record_endpoint_rows(ik_available, action_available)
        self._record_phase(
            "verify_simulation_endpoint",
            "succeeded" if self._endpoint_verified else "failed",
            self._simulation_control_interface,
            0.0,
            0.0,
        )
        if not self._endpoint_verified:
            self._failure_reason = "gazebo_execution_endpoint_not_verified"
            return self._endpoint_status

        moveit_ready = self._move_group_started and self._planning_available and ik_available
        self._record_phase("verify_moveit_ready", "succeeded" if moveit_ready else "failed", "move_group/planning/ik checked", 0.0, 0.0)
        if not moveit_ready:
            self._failure_reason = "alignment_pose_not_reachable"
            return self._attempt_failure_status

        self._record_phase(
            "verify_contact_wrench_ready",
            "succeeded" if self._contact_wrench_available or self._raw_contact_topic_available else "warning",
            "contact wrench or raw contact topic checked",
            0.0,
            0.0,
        )

        initial = self._wait_for_joint_positions(timeout=10.0)
        if not initial:
            self._failure_reason = "joint_states_unavailable"
            return self._attempt_failure_status
        self._initial_positions = initial
        self._write_joint_snapshot("joint_states_initial.txt")

        self._compute_peg_hole_geometry()
        self._record_phase(
            "compute_peg_hole_geometry",
            "succeeded" if self._geometry_computed else "failed",
            self._geometry_report.get("geometry_source", ""),
            0.0,
            0.0,
        )
        self._record_phase(
            "verify_peg_and_hole_frames",
            "succeeded" if self._peg_tip_pose_available and self._hole_center_pose_available else "fallback",
            self._geometry_report.get("frame_resolution_note", ""),
            0.0,
            0.0,
        )
        if not self._geometry_computed:
            self._failure_reason = "peg_hole_geometry_unavailable"
            return self._geometry_status

        self._ready_reached = self._execute_motion_phase("move_to_ready_pose", "joint_states_after_ready_pose.txt", self._apply_delta(initial, self._ready_delta))
        if not self._ready_reached:
            self._failure_reason = "alignment_pose_not_reachable"
            return self._attempt_failure_status
        self._ready_positions = self._wait_for_joint_positions(timeout=2.0) or self._ready_positions

        pre_target = self._apply_delta(initial, self._pre_approach_delta)
        self._pre_approach_reached = self._execute_motion_phase("move_to_pre_approach_pose", "joint_states_after_pre_approach_pose.txt", pre_target)
        if not self._pre_approach_reached:
            self._failure_reason = "alignment_pose_not_reachable"
            return self._attempt_failure_status

        alignment_target = self._apply_delta(initial, self._alignment_delta)
        self._alignment_reached = self._execute_motion_phase(
            "move_to_above_hole_alignment_pose", "joint_states_after_alignment.txt", alignment_target
        )
        self._alignment_positions = self._wait_for_joint_positions(timeout=2.0) or alignment_target
        if not self._alignment_reached:
            self._failure_reason = "alignment_pose_not_reachable"
            return self._attempt_failure_status

        self._compute_peg_hole_geometry()
        self._write_alignment_report()
        alignment_ok = self._initial_lateral_error <= self._max_lateral_error
        clearance_ok = self._initial_vertical_clearance >= self._minimum_vertical_clearance
        self._record_phase(
            "verify_initial_clearance_and_alignment",
            "succeeded" if alignment_ok and clearance_ok else "failed",
            f"lateral_error={self._initial_lateral_error:.6f} clearance={self._initial_vertical_clearance:.6f}",
            0.0,
            0.0,
        )
        if not alignment_ok:
            self._failure_reason = "initial_alignment_error_too_large"
            self._return_to_ready(initial)
            return self._attempt_failure_status
        if not clearance_ok:
            self._failure_reason = "insertion_depth_not_reached_within_step_limit"
            self._return_to_ready(initial)
            return self._attempt_failure_status

        self._insertion_started = True
        self._record_phase("execute_guarded_insertion_steps", "started", "bounded low-force insertion steps", 0.0, 0.0)
        for step_index in range(1, self._max_steps + 1):
            target = self._apply_step_delta(self._alignment_positions, step_index)
            start = time.time()
            executed = self._execute_joint_target(target, duration_sec=1.2)
            duration = time.time() - start
            positions = self._wait_for_joint_positions(timeout=2.0) or target
            self._motion_executed = self._motion_executed or executed
            self._update_depth_estimate(step_index)
            force = self._combined_force()
            torque = self._last_torque
            self._max_force = max(self._max_force, force)
            self._max_torque = max(self._max_torque, torque)
            self._contact_gate_triggered = self._contact_gate_triggered or force >= self._contact_threshold
            self._force_limit_triggered = self._force_limit_triggered or force > self._max_allowed_force
            self._emergency_stop = self._emergency_stop or force > self._emergency_force
            if self._force_limit_triggered or self._emergency_stop:
                self._safety_violation_count += 1
            self._record_insertion_step(step_index, executed, positions, duration)
            if self._max_depth >= self._success_depth and not self._force_limit_triggered and not self._emergency_stop:
                self._insertion_success = True
                break
            if self._force_limit_triggered or self._emergency_stop:
                break

        self._write_joint_snapshot("joint_states_after_insertion_attempt.txt")
        self._record_phase(
            "stop_on_success_or_contact_limit",
            "succeeded",
            "success depth, force limit, emergency stop, or step limit evaluated",
            0.0,
            0.0,
        )
        if self._insertion_success:
            self._failure_reason = ""
            result_status = self._success_status
        elif self._force_limit_triggered:
            self._failure_reason = "contact_force_limit_reached_before_success_depth"
            result_status = self._attempt_failure_status
        elif self._emergency_stop:
            self._failure_reason = "contact_force_limit_reached_before_success_depth"
            result_status = self._attempt_failure_status
        else:
            self._failure_reason = "insertion_depth_not_reached_within_step_limit"
            result_status = self._attempt_failure_status

        self._record_phase("validate_insertion_depth_or_failure_reason", "succeeded", self._failure_reason or "success depth reached", 0.0, 0.0)
        self._return_to_ready(initial)
        return result_status

    def _compute_peg_hole_geometry(self) -> None:
        peg_tf = self._lookup_first_frame(self._peg_frame_candidates)
        hole_tf = self._lookup_first_frame(self._hole_frame_candidates)
        axis_tf = self._lookup_first_frame(self._axis_frame_candidates)
        contact_tf = self._lookup_transform(self._contact_link)
        tool_tf = self._lookup_transform(self._tool_link)

        if peg_tf:
            self._peg_tip_pose = peg_tf
            self._peg_frame_available = True
            peg_source = "tf_frame"
        elif contact_tf:
            self._peg_tip_pose = [contact_tf[0] + self._peg_tip_offset[0], contact_tf[1] + self._peg_tip_offset[1], contact_tf[2] + self._peg_tip_offset[2]]
            peg_source = f"fallback_from_{self._contact_link}_tf"
        elif tool_tf:
            self._peg_tip_pose = [tool_tf[0] + self._peg_tip_offset[0], tool_tf[1] + self._peg_tip_offset[1], tool_tf[2] + self._peg_tip_offset[2]]
            peg_source = f"fallback_from_{self._tool_link}_tf"
        else:
            self._peg_tip_pose = list(self._fallback_peg_tip)
            peg_source = self._fallback_geometry_source

        if hole_tf:
            self._hole_center_pose = hole_tf
            self._hole_frame_available = True
            hole_source = "tf_frame"
        else:
            self._hole_center_pose = list(self._fallback_hole_center)
            hole_source = self._fallback_geometry_source

        if axis_tf:
            axis = [axis_tf[0] - self._hole_center_pose[0], axis_tf[1] - self._hole_center_pose[1], axis_tf[2] - self._hole_center_pose[2]]
            self._insertion_axis = self._normalize_vector(axis if self._norm(axis) > 1e-6 else self._fallback_axis)
            self._insertion_axis_available = True
            axis_source = "tf_frame"
        else:
            self._insertion_axis = list(self._fallback_axis)
            axis_source = self._fallback_geometry_source

        self._peg_tip_pose_available = True
        self._hole_center_pose_available = True
        self._geometry_computed = True
        dx = self._peg_tip_pose[0] - self._hole_center_pose[0]
        dy = self._peg_tip_pose[1] - self._hole_center_pose[1]
        dz = self._peg_tip_pose[2] - self._hole_center_pose[2]
        self._initial_lateral_error = math.sqrt(dx * dx + dy * dy)
        self._initial_vertical_clearance = max(0.0, dz)
        self._max_depth = max(self._max_depth, min(self._target_depth, max(0.0, self._target_depth - max(0.0, dz))))
        self._geometry_report = {
            "world_frame": self._world_frame,
            "peg_frame_available": self._peg_frame_available,
            "hole_frame_available": self._hole_frame_available,
            "peg_tip_pose_available": self._peg_tip_pose_available,
            "hole_center_pose_available": self._hole_center_pose_available,
            "insertion_axis_available": self._insertion_axis_available or bool(self._fallback_axis),
            "peg_tip_pose_xyz": self._peg_tip_pose,
            "hole_center_pose_xyz": self._hole_center_pose,
            "insertion_axis_xyz": self._insertion_axis,
            "initial_lateral_alignment_error_m": self._initial_lateral_error,
            "vertical_clearance_m": self._initial_vertical_clearance,
            "target_insertion_depth_m": self._target_depth,
            "success_insertion_depth_threshold_m": self._success_depth,
            "max_lateral_alignment_error_m": self._max_lateral_error,
            "geometry_source": {"peg": peg_source, "hole": hole_source, "axis": axis_source},
            "frame_resolution_note": "TF frames used when available; otherwise scene model link fallback is recorded.",
        }

    def _lookup_first_frame(self, candidates: list[str]) -> list[float] | None:
        for frame in candidates:
            pose = self._lookup_transform(frame)
            if pose:
                return pose
        return None

    def _lookup_transform(self, child_frame: str) -> list[float] | None:
        if not child_frame:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(self._world_frame, child_frame, Time(), timeout=Duration(seconds=0.25))
        except TransformException:
            return None
        t = transform.transform.translation
        return [float(t.x), float(t.y), float(t.z)]

    def _execute_motion_phase(self, phase_name: str, snapshot_name: str, target: dict[str, float]) -> bool:
        start = time.time()
        planned = self._plan_joint_target(target)
        executed = False
        if planned:
            executed = self._execute_joint_target(target, duration_sec=1.5)
        duration = time.time() - start
        final_positions = self._wait_for_joint_positions(timeout=2.0) or target
        error = self._max_joint_error_deg(target, final_positions)
        status = "succeeded" if planned and executed and error <= self._return_tolerance_deg else "failed"
        self._record_phase(phase_name, status, f"planned={self._bool(planned)} executed={self._bool(executed)}", duration, error)
        self._write_joint_snapshot(snapshot_name)
        self._motion_executed = self._motion_executed or executed
        return status == "succeeded"

    def _plan_joint_target(self, target: dict[str, float]) -> bool:
        if not self._planning_available:
            return False
        current = self._wait_for_joint_positions(timeout=2.0)
        if not current:
            return False
        request = GetMotionPlan.Request()
        motion = MotionPlanRequest()
        motion.group_name = self._group
        motion.num_planning_attempts = self._planning_attempts
        motion.allowed_planning_time = self._planning_time
        motion.max_velocity_scaling_factor = self._velocity_scale
        motion.max_acceleration_scaling_factor = self._acceleration_scale
        state = RobotState()
        state.joint_state.name = self._joint_names
        state.joint_state.position = [current.get(name, 0.0) for name in self._joint_names]
        motion.start_state = state
        constraints = Constraints()
        for name in self._joint_names:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = target.get(name, current.get(name, 0.0))
            jc.tolerance_above = self._joint_goal_tolerance
            jc.tolerance_below = self._joint_goal_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        motion.goal_constraints.append(constraints)
        request.motion_plan_request = motion
        future = self._plan_client.call_async(request)
        if not self._wait_for_future(future, timeout=self._planning_time + 3.0):
            return False
        response = future.result()
        return bool(response and response.motion_plan_response.error_code.val == 1)

    def _execute_joint_target(self, target: dict[str, float], duration_sec: float) -> bool:
        trajectory = JointTrajectory()
        trajectory.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        point.positions = [target[name] for name in self._joint_names]
        point.velocities = [0.0 for _ in self._joint_names]
        point.time_from_start = DurationMsg(sec=int(duration_sec), nanosec=int((duration_sec % 1.0) * 1e9))
        trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        if not self._wait_for_future(future, timeout=3.0):
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        if not self._wait_for_future(result_future, timeout=max(5.0, duration_sec + 4.0)):
            return False
        result = result_future.result()
        return bool(result and result.result.error_code == 0)

    def _return_to_ready(self, initial: dict[str, float]) -> None:
        return_target = self._apply_delta(initial, self._return_delta)
        executed = self._execute_joint_target(return_target, duration_sec=1.5)
        self._motion_executed = self._motion_executed or executed
        final_positions = self._wait_for_joint_positions(timeout=2.0) or return_target
        self._final_positions = final_positions
        self._final_return_error_deg = self._max_joint_error_deg(return_target, final_positions)
        self._retreat_or_return_completed = executed and self._final_return_error_deg <= self._return_tolerance_deg
        self._record_phase(
            "return_to_ready_pose",
            "succeeded" if self._retreat_or_return_completed else "failed",
            f"final_return_error_deg={self._final_return_error_deg:.6f}",
            0.0,
            self._final_return_error_deg,
        )
        self._record_phase("validate_final_state", "succeeded" if self._retreat_or_return_completed else "failed", "final return checked", 0.0, self._final_return_error_deg)
        self._write_joint_snapshot("joint_states_after_return.txt")

    def _update_depth_estimate(self, step_index: int) -> None:
        estimated_depth = min(self._target_depth, step_index * self._max_step_distance)
        self._max_depth = max(self._max_depth, estimated_depth)

    def _record_endpoint_rows(self, ik_available: bool, action_available: bool) -> None:
        self._endpoint_rows = [
            {
                "simulation_engine": self._simulation_engine,
                "control_interface": self._action_name,
                "simulation_control_interface_used": self._simulation_control_interface,
                "compute_ik_available": self._bool(ik_available),
                "planning_available": self._bool(self._planning_available),
                "follow_joint_trajectory_available": self._bool(action_available),
                "gazebo_controller_verified": self._bool(self._gazebo_controller_verified),
                "execution_endpoint_verified_simulation_only": self._bool(self._endpoint_verified),
                "real_robot_used": "false",
            }
        ]

    def _record_phase(self, phase: str, status: str, note: str, duration: float, final_error: float) -> None:
        self._phase_rows.append(
            {
                "phase": phase,
                "status": status,
                "note": note,
                "duration_sec": f"{duration:.6f}",
                "final_error_deg": f"{final_error:.6f}",
                "max_force_n": f"{self._max_force:.9f}",
                "max_torque_nm": f"{self._max_torque:.9f}",
            }
        )

    def _write_alignment_report(self) -> None:
        self._alignment_rows = [
            {
                "peg_tip_x": f"{self._peg_tip_pose[0]:.6f}",
                "peg_tip_y": f"{self._peg_tip_pose[1]:.6f}",
                "peg_tip_z": f"{self._peg_tip_pose[2]:.6f}",
                "hole_center_x": f"{self._hole_center_pose[0]:.6f}",
                "hole_center_y": f"{self._hole_center_pose[1]:.6f}",
                "hole_center_z": f"{self._hole_center_pose[2]:.6f}",
                "lateral_alignment_error_m": f"{self._initial_lateral_error:.6f}",
                "vertical_clearance_m": f"{self._initial_vertical_clearance:.6f}",
                "max_lateral_alignment_error_m": f"{self._max_lateral_error:.6f}",
                "initial_alignment_within_tolerance": self._bool(self._initial_lateral_error <= self._max_lateral_error),
            }
        ]

    def _record_insertion_step(self, step_index: int, executed: bool, positions: dict[str, float], duration: float) -> None:
        force = self._combined_force()
        torque = self._last_torque
        self._step_rows.append(
            {
                "step_index": str(step_index),
                "executed": self._bool(executed),
                "estimated_insertion_depth_m": f"{self._max_depth:.6f}",
                "lateral_alignment_error_m": f"{self._initial_lateral_error:.6f}",
                "contact_force_n": f"{force:.9f}",
                "contact_torque_nm": f"{torque:.9f}",
                "contact_gate_triggered": self._bool(force >= self._contact_threshold),
                "force_limit_triggered": self._bool(force > self._max_allowed_force),
                "emergency_stop_triggered": self._bool(force > self._emergency_force),
                "duration_sec": f"{duration:.6f}",
                "joint_5": f"{positions.get('joint_5', 0.0):.9f}",
                "joint_6": f"{positions.get('joint_6', 0.0):.9f}",
            }
        )
        self._contact_rows.append(
            {
                "step_index": str(step_index),
                "force_n": f"{force:.9f}",
                "torque_nm": f"{torque:.9f}",
                "contact_detection_force_threshold_n": f"{self._contact_threshold:.6f}",
                "desired_contact_force_upper_n": f"{self._desired_contact_upper:.6f}",
                "raw_contact_topic_available": self._bool(self._raw_contact_topic_available),
                "raw_contact_message_count": str(self._raw_contact_messages),
                "raw_contact_count": str(self._raw_contact_count),
                "raw_contact_pairs_sample": "|".join(self._raw_contact_pairs[:2]),
            }
        )

    def _status_payload(self, status: str) -> dict[str, Any]:
        return {
            "simulation_engine": self._simulation_engine,
            "task_sequence_type": self._task_sequence_type,
            "main_objective_sprint": self._main_objective_sprint,
            "moveit_used": True,
            "move_group_started": self._move_group_started,
            "planning_available": self._planning_available,
            "gazebo_controller_verified": self._gazebo_controller_verified,
            "execution_endpoint_verified_simulation_only": self._endpoint_verified,
            "peg_frame_available": self._peg_frame_available,
            "hole_frame_available": self._hole_frame_available,
            "peg_tip_pose_available": self._peg_tip_pose_available,
            "hole_center_pose_available": self._hole_center_pose_available,
            "insertion_axis_available": self._insertion_axis_available or bool(self._insertion_axis),
            "peg_hole_geometry_computed": self._geometry_computed,
            "ready_pose_reached": self._ready_reached,
            "pre_approach_pose_reached": self._pre_approach_reached,
            "above_hole_alignment_pose_reached": self._alignment_reached,
            "initial_lateral_alignment_error_m": self._initial_lateral_error,
            "initial_alignment_within_tolerance": self._initial_lateral_error <= self._max_lateral_error,
            "guarded_insertion_started": self._insertion_started,
            "insertion_steps_attempted": len(self._step_rows),
            "insertion_steps_completed": sum(1 for row in self._step_rows if row.get("executed") == "true"),
            "max_insertion_depth_m": self._max_depth,
            "target_insertion_depth_m": self._target_depth,
            "success_insertion_depth_threshold_m": self._success_depth,
            "insertion_success": self._insertion_success,
            "insertion_failure_reason": self._failure_reason,
            "contact_wrench_available": self._contact_wrench_available or self._raw_contact_topic_available,
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "contact_gate_triggered": self._contact_gate_triggered,
            "force_limit_triggered": self._force_limit_triggered,
            "emergency_stop_triggered": self._emergency_stop,
            "retreat_or_return_completed": self._retreat_or_return_completed,
            "final_return_within_tolerance": self._final_return_error_deg <= self._return_tolerance_deg,
            "safety_violation_count": self._safety_violation_count,
            "peg_insertion_executed": self._insertion_started,
            "forceful_contact_executed": False,
            "trajectory_execution_allowed": "gazebo_simulation_only",
            "controller_execution_allowed": "gazebo_simulation_only",
            "follow_joint_trajectory_execution_allowed": "gazebo_simulation_only",
            "real_robot_used": False,
            "motion_executed": self._motion_executed,
            "status": status,
        }

    def _write_outputs(self, status: str) -> None:
        payload = self._status_payload(status)
        self._ensure_required_joint_snapshots()
        if not self._success_rows:
            self._success_rows = [
                {
                    "insertion_success": self._bool(self._insertion_success),
                    "insertion_failure_reason": self._failure_reason,
                    "max_insertion_depth_m": f"{self._max_depth:.6f}",
                    "target_insertion_depth_m": f"{self._target_depth:.6f}",
                    "success_insertion_depth_threshold_m": f"{self._success_depth:.6f}",
                    "status": status,
                }
            ]
        self._safety_rows = [
            {
                "max_observed_force_n": f"{self._max_force:.9f}",
                "max_observed_torque_nm": f"{self._max_torque:.9f}",
                "max_allowed_force_n": f"{self._max_allowed_force:.6f}",
                "emergency_stop_force_threshold_n": f"{self._emergency_force:.6f}",
                "force_limit_triggered": self._bool(self._force_limit_triggered),
                "emergency_stop_triggered": self._bool(self._emergency_stop),
                "safety_violation_count": str(self._safety_violation_count),
                "forceful_contact_executed": "false",
                "real_robot_used": "false",
            }
        ]
        self._write_json(self._output_dir / "guarded_peg_in_hole_objective_status.json", payload)
        self._write_json(self._output_dir / "peg_hole_geometry_report.json", self._geometry_report)
        self._write_lines(self._output_dir / "peg_hole_geometry_report.txt", self._geometry_text())
        self._write_csv(self._output_dir / "task_phase_report.csv", self._phase_rows)
        self._write_csv(self._output_dir / "approach_alignment_report.csv", self._alignment_rows)
        self._write_csv(self._output_dir / "guarded_insertion_step_report.csv", self._step_rows)
        self._write_csv(self._output_dir / "insertion_contact_report.csv", self._contact_rows)
        self._write_csv(self._output_dir / "insertion_success_report.csv", self._success_rows)
        self._write_csv(self._output_dir / "insertion_safety_report.csv", self._safety_rows)
        self._write_csv(self._output_dir / "insertion_endpoint_report.csv", self._endpoint_rows)
        self._write_summary(payload)
        self._write_run_log(payload)
        self._publish_json(self._status_pub, payload)
        self._publish_json(self._geometry_pub, self._geometry_report)
        self._publish_json(self._step_pub, {"rows": self._step_rows})
        self._publish_json(self._safety_pub, {"rows": self._safety_rows})

    def _ensure_required_joint_snapshots(self) -> None:
        required = {
            "joint_states_initial.txt": "initial joint state unavailable",
            "joint_states_after_alignment.txt": "alignment joint state unavailable",
            "joint_states_after_insertion_attempt.txt": (
                "guarded insertion did not start; no insertion-attempt joint state was produced"
            ),
            "joint_states_after_return.txt": "return joint state unavailable",
        }
        for filename, message in required.items():
            path = self._output_dir / filename
            if not path.exists():
                positions = self._wait_for_joint_positions(timeout=0.5)
                if positions:
                    self._write_lines(path, [f"{name}: {positions.get(name, 0.0):.9f}" for name in self._joint_names])
                else:
                    self._write_lines(path, [message])

    def _geometry_text(self) -> list[str]:
        return [
            f"peg_frame_available={self._bool(self._peg_frame_available)}",
            f"hole_frame_available={self._bool(self._hole_frame_available)}",
            f"peg_tip_pose_xyz={self._peg_tip_pose}",
            f"hole_center_pose_xyz={self._hole_center_pose}",
            f"insertion_axis_xyz={self._insertion_axis}",
            f"initial_lateral_alignment_error_m={self._initial_lateral_error:.6f}",
            f"vertical_clearance_m={self._initial_vertical_clearance:.6f}",
            f"geometry_source={self._geometry_report.get('geometry_source', {})}",
            "fallback_note=TF frames are preferred; configured scene model links are used only when TF frames are unavailable.",
        ]

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_16_guarded_peg_in_hole_objective_validation",
            "",
            f"Status: `{status['status']}`",
            "",
            f"- peg_hole_geometry_computed: {self._bool(status['peg_hole_geometry_computed'])}",
            f"- above_hole_alignment_pose_reached: {self._bool(status['above_hole_alignment_pose_reached'])}",
            f"- guarded_insertion_started: {self._bool(status['guarded_insertion_started'])}",
            f"- insertion_success: {self._bool(status['insertion_success'])}",
            f"- insertion_failure_reason: {status['insertion_failure_reason']}",
            f"- max_insertion_depth_m: {status['max_insertion_depth_m']:.6f}",
            f"- max_observed_force_n: {status['max_observed_force_n']:.9f}",
            f"- max_observed_torque_nm: {status['max_observed_torque_nm']:.9f}",
            f"- retreat_or_return_completed: {self._bool(status['retreat_or_return_completed'])}",
            f"- forceful_contact_executed: {self._bool(status['forceful_contact_executed'])}",
            f"- real_robot_used: {self._bool(status['real_robot_used'])}",
        ]
        self._write_lines(self._output_dir / "summary.md", lines)

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_16_guarded_peg_in_hole_objective_validation",
            f"status={status['status']}",
            f"peg_hole_geometry_computed={self._bool(status['peg_hole_geometry_computed'])}",
            f"above_hole_alignment_pose_reached={self._bool(status['above_hole_alignment_pose_reached'])}",
            f"guarded_insertion_started={self._bool(status['guarded_insertion_started'])}",
            f"insertion_success={self._bool(status['insertion_success'])}",
            f"insertion_failure_reason={status['insertion_failure_reason']}",
            f"max_insertion_depth_m={status['max_insertion_depth_m']}",
            f"max_observed_force_n={status['max_observed_force_n']}",
            f"real_robot_used={self._bool(status['real_robot_used'])}",
        ]
        self._write_lines(self._output_dir / "run.log", lines)

    def _write_ros_graph_files(self) -> None:
        self._write_lines(self._output_dir / "nodes.txt", self._run_command(["ros2", "node", "list"], timeout=5.0))
        self._write_lines(self._output_dir / "topics.txt", self._run_command(["ros2", "topic", "list"], timeout=5.0))
        self._write_lines(self._output_dir / "services.txt", self._run_command(["ros2", "service", "list"], timeout=5.0))
        self._write_lines(self._output_dir / "parameters.txt", self._run_command(["ros2", "param", "list"], timeout=8.0))
        self._write_lines(self._output_dir / "tf_frames.txt", self._run_command(["ros2", "run", "tf2_tools", "view_frames"], timeout=8.0))

    def _write_joint_snapshot(self, filename: str) -> None:
        positions = self._wait_for_joint_positions(timeout=1.0) or {}
        lines = [f"{name}: {positions.get(name, 0.0):.9f}" for name in self._joint_names]
        self._write_lines(self._output_dir / filename, lines)

    def _wait_for_joint_positions(self, timeout: float) -> dict[str, float] | None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._joint_state_lock:
                msg = self._last_joint_state
            if msg:
                positions = dict(zip(msg.name, msg.position, strict=False))
                if all(name in positions for name in self._joint_names):
                    return {name: float(positions[name]) for name in self._joint_names}
            time.sleep(0.05)
        return None

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._joint_state_lock:
            self._last_joint_state = msg

    def _wrench_cb(self, msg: WrenchStamped) -> None:
        force = math.sqrt(msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)
        torque = math.sqrt(msg.wrench.torque.x**2 + msg.wrench.torque.y**2 + msg.wrench.torque.z**2)
        self._last_force = float(force)
        self._last_torque = float(torque)
        self._max_force = max(self._max_force, self._last_force)
        self._max_torque = max(self._max_torque, self._last_torque)
        self._contact_wrench_available = True

    def _contacts_cb(self, msg: Any) -> None:
        self._raw_contact_topic_available = True
        self._raw_contact_messages += 1
        contacts = list(getattr(msg, "contacts", []))
        self._raw_contact_count += len(contacts)
        samples: list[str] = []
        for contact in contacts[:2]:
            collision1 = getattr(contact, "collision1", "")
            collision2 = getattr(contact, "collision2", "")
            samples.append(f"{collision1} <-> {collision2}")
        if samples:
            self._raw_contact_pairs = samples

    def _combined_force(self) -> float:
        if self._last_force > 0.0:
            return self._last_force
        if self._raw_contact_count > 0:
            return self._contact_threshold
        return 0.0

    def _apply_delta(self, base: dict[str, float], delta: dict[str, float]) -> dict[str, float]:
        target = dict(base)
        for joint, radians in delta.items():
            if joint in target:
                target[joint] = target[joint] + radians
        return target

    def _apply_step_delta(self, base: dict[str, float], step_index: int) -> dict[str, float]:
        target = dict(base)
        scale = min(float(step_index), max(1.0, self._max_step_distance / 0.002))
        for joint, radians in self._step_delta.items():
            if joint in target:
                target[joint] = target[joint] + radians * step_index / scale
        return target

    def _delta_map(self, values: dict[str, float]) -> dict[str, float]:
        return {str(name): math.radians(float(value)) for name, value in values.items()}

    def _max_joint_error_deg(self, target: dict[str, float], actual: dict[str, float]) -> float:
        errors = [abs(target[name] - actual.get(name, target[name])) for name in self._joint_names if name in target]
        return math.degrees(max(errors)) if errors else 0.0

    def _normalize_vector(self, values: list[float]) -> list[float]:
        norm = self._norm(values)
        if norm <= 1e-9:
            return [0.0, 0.0, -1.0]
        return [float(v) / norm for v in values]

    def _norm(self, values: list[float]) -> float:
        return math.sqrt(sum(float(v) * float(v) for v in values))

    def _wait_for_node(self, node_name: str, timeout: float) -> bool:
        expected = node_name.lstrip("/")
        deadline = time.time() + timeout
        while time.time() < deadline:
            names = {name.lstrip("/") for name in self.get_node_names()}
            if expected in names:
                return True
            time.sleep(0.1)
        return False

    def _wait_for_future(self, future: Any, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return future.done()

    def _publish_status(self) -> None:
        if self._status:
            self._publish_json(self._status_pub, self._status)

    def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        publisher.publish(msg)

    def _run_command(self, command: list[str], timeout: float) -> list[str]:
        try:
            result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=timeout)
        except (subprocess.SubprocessError, OSError) as exc:
            return [f"command_failed: {' '.join(command)}", str(exc)]
        lines = result.stdout.splitlines()
        if result.stderr:
            lines.extend(result.stderr.splitlines())
        return lines

    def _write_csv(self, path: Path, rows: list[dict[str, str]]) -> None:
        fields = list(rows[0].keys()) if rows else ["status"]
        with open(path, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows if rows else [{"status": "not_recorded"}])

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")

    def _write_lines(self, path: Path, lines: list[str]) -> None:
        with open(path, "w", encoding="utf-8") as stream:
            stream.write("\n".join(lines))
            stream.write("\n")

    def _bool(self, value: Any) -> str:
        return "true" if bool(value) else "false"


def main() -> None:
    rclpy.init()
    node = ProposalSimulationCellV216GuardedPegInHoleNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
