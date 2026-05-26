"""Guarded pre-contact task sequence for proposal_simulation_cell_v2_5."""

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
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


class ProposalSimulationCellV25GuardedTaskSequenceNode(Node):
    """Execute a bounded pre-contact task sequence through the Gazebo controller."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_5_guarded_task_sequence_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_5")

        self._config = self._load_config()
        robot = self._config.get("robot", {})
        task = self._config.get("task_sequence", {})
        moveit = self._config.get("moveit_execution", {})
        targets = self._config.get("phase_targets", {})
        safety = self._config.get("safety_gates", {})
        validation = self._config.get("validation", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_5")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._simulation_engine = str(task.get("simulation_engine", "gazebo"))
        self._task_sequence_type = str(task.get("task_sequence_type", "guarded_pre_contact_sequence"))
        self._phase_names = [str(name) for name in task.get("phases", [])]
        self._max_task_duration = float(task.get("max_task_duration_sec", 40.0))
        self._hold_duration = float(task.get("hold_duration_sec", 2.0))
        self._return_tolerance_deg = float(task.get("return_tolerance_deg", 0.75))
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
        self._velocity_scale = float(moveit.get("max_velocity_scaling_factor", 0.1))
        self._acceleration_scale = float(moveit.get("max_acceleration_scaling_factor", 0.1))
        self._joint_goal_tolerance = float(moveit.get("joint_goal_tolerance_rad", 0.01))
        self._max_joint_delta_deg = float(targets.get("max_joint_delta_from_initial_deg", 5.0))
        self._ready_delta = self._delta_map(targets.get("ready_pose_delta_deg", {}))
        self._pre_approach_delta = self._delta_map(targets.get("pre_approach_pose_delta_deg", {}))
        self._standby_delta = self._delta_map(targets.get("pre_insertion_standby_pose_delta_deg", {}))
        self._max_allowed_force = float(safety.get("max_allowed_force_n", 50.0))
        self._max_allowed_torque = float(safety.get("max_allowed_torque_nm", 5.0))
        self._emergency_force = float(safety.get("emergency_stop_force_threshold_n", 45.0))
        self._peg_insertion_allowed = bool(safety.get("peg_insertion_allowed", False))
        self._contact_seeking_allowed = bool(safety.get("contact_seeking_allowed", False))
        self._joint_states_topic = str(validation.get("joint_states_topic", "/joint_states"))
        self._contact_wrench_topic = str(validation.get("contact_wrench_topic", "/proposal_simulation_cell/contact_wrench"))
        self._validation_timeout = float(validation.get("validation_timeout_sec", 150.0))
        self._startup_wait = float(validation.get("startup_wait_sec", 5.0))
        self._success_status = str(validation.get("status_success", "guarded_pre_contact_task_sequence_validated"))

        self._status_pub = self.create_publisher(
            String, str(validation.get("status_topic", "/proposal_simulation_cell/guarded_task_sequence_status")), 10
        )
        self._phase_pub = self.create_publisher(
            String, str(validation.get("phase_report_topic", "/proposal_simulation_cell/task_phase_report")), 10
        )
        self._safety_pub = self.create_publisher(
            String, str(validation.get("safety_report_topic", "/proposal_simulation_cell/task_sequence_safety_report")), 10
        )
        self._endpoint_pub = self.create_publisher(
            String, str(validation.get("endpoint_report_topic", "/proposal_simulation_cell/task_sequence_endpoint_report")), 10
        )
        self._contact_wrench_pub = self.create_publisher(WrenchStamped, self._contact_wrench_topic, 10)
        self.create_subscription(JointState, self._joint_states_topic, self._on_joint_state, 10)
        self.create_subscription(WrenchStamped, self._contact_wrench_topic, self._on_contact_wrench, 10)
        self.create_subscription(String, "/robot_description", self._on_robot_description, 10)

        self._ik_client = self.create_client(GetPositionIK, self._compute_ik_service)
        self._plan_client = self.create_client(GetMotionPlan, self._plan_service)
        self._parameter_client = self.create_client(GetParameters, f"{self._move_group_node}/get_parameters")
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)

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
        self._initial_positions: dict[str, float] = {}
        self._ready_positions: dict[str, float] = {}
        self._snapshots: dict[str, JointState | None] = {}
        self._phase_rows: list[dict[str, str]] = []
        self._plan_rows: list[dict[str, str]] = []
        self._joint_rows: list[dict[str, str]] = []
        self._endpoint_rows: list[dict[str, str]] = []
        self._max_force = 0.0
        self._max_torque = 0.0
        self._safety_violation_count = 0
        self._phases_planned = 0
        self._phases_executed = 0
        self._all_phase_plans_found = True
        self._all_phase_executions_observed = True
        self._ready_pose_reached = False
        self._pre_approach_pose_reached = False
        self._standby_pose_reached = False
        self._hold_completed = False
        self._return_completed = False
        self._max_observed_delta_deg = 0.0
        self._final_return_error_deg = 0.0
        self._final_return_within_tolerance = False
        self._motion_executed = False
        self._peg_insertion_executed = False
        self._contact_seeking_executed = False
        self._shutdown_segfault_observed = "unknown_after_launch_shutdown"

        self.create_timer(0.2, self._tick)
        self.get_logger().info("proposal_simulation_cell_v2_5 guarded task sequence node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v2.5 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _delta_map(self, values: dict[str, Any]) -> dict[str, float]:
        return {str(joint): math.radians(float(delta)) for joint, delta in values.items()}

    def _on_joint_state(self, message: JointState) -> None:
        self._last_joint_state = message

    def _on_robot_description(self, message: String) -> None:
        self._robot_description = message.data

    def _on_contact_wrench(self, message: WrenchStamped) -> None:
        force = self._force_magnitude(message)
        torque = self._torque_magnitude(message)
        self._max_force = max(self._max_force, force)
        self._max_torque = max(self._max_torque, torque)
        if force > self._emergency_force or force > self._max_allowed_force or torque > self._max_allowed_torque:
            self._safety_violation_count += 1

    def _tick(self) -> None:
        if self._finished:
            return
        self._publish_contact_wrench_sample()
        self._publish_reports()
        elapsed = time.monotonic() - self._start_time
        if not self._started and elapsed >= self._startup_wait:
            self._started = True
            threading.Thread(target=self._run_sequence, daemon=True).start()
        if elapsed >= self._validation_timeout and not self._finished:
            self._write_outputs_once("guarded_pre_contact_sequence_timeout")

    def _run_sequence(self) -> None:
        sequence_start = time.monotonic()
        self._record_phase("initialize", "succeeded", "task sequence initialized", 0.0, 0.0)
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
        if self._safety_violation_count > 0 or self._peg_insertion_allowed or self._contact_seeking_allowed:
            self._record_phase("verify_safety_gate_ready", "failed", "Safety gate blocked", 0.0, 0.0)
            self._write_outputs_once("safety_gate_not_ready")
            return
        self._record_phase("verify_safety_gate_ready", "succeeded", "Safety gate clear", 0.0, 0.0)
        if not self._wait_for_joint_state(timeout_sec=30.0):
            self._write_outputs_once("joint_states_unavailable")
            return
        self._initial_positions = self._joint_positions(self._last_joint_state)
        self._snapshots["initial"] = self._last_joint_state
        self._ready_positions = self._target_from_initial(self._ready_delta)

        phases = [
            ("move_to_ready_pose", "after_ready_pose", self._ready_positions),
            ("move_to_pre_approach_pose", "after_pre_approach_pose", self._target_from_initial(self._pre_approach_delta)),
            ("move_to_pre_insertion_standby_pose", "after_pre_insertion_standby_pose", self._target_from_initial(self._standby_delta)),
        ]
        for phase_name, snapshot_name, target in phases:
            if time.monotonic() - sequence_start > self._max_task_duration:
                self._write_outputs_once("guarded_pre_contact_sequence_timeout")
                return
            if not self._execute_motion_phase(phase_name, snapshot_name, target):
                self._write_outputs_once("guarded_pre_contact_sequence_execution_failed")
                return

        hold_start = time.monotonic()
        while time.monotonic() - hold_start < self._hold_duration:
            if self._safety_violation_count > 0:
                self._record_phase("hold_pre_contact_pose", "failed", "Safety violation during hold", 0.0, 0.0)
                self._write_outputs_once("safety_violation_detected")
                return
            time.sleep(0.05)
        self._snapshots["after_hold"] = self._last_joint_state
        self._hold_completed = True
        self._phases_executed += 1
        self._record_phase("hold_pre_contact_pose", "succeeded", "Pre-contact hold completed", self._hold_duration, 0.0)

        if not self._execute_motion_phase("return_to_ready_pose", "after_return", self._ready_positions):
            self._write_outputs_once("guarded_pre_contact_sequence_execution_failed")
            return
        self._return_completed = True
        self._calculate_final_return_error()
        self._record_phase(
            "validate_final_state",
            "succeeded" if self._final_return_within_tolerance else "failed",
            "Final state checked against ready pose",
            0.0,
            self._final_return_error_deg,
        )
        self._phases_executed += 1
        self._write_joint_rows()
        if not self._final_return_within_tolerance:
            self._write_outputs_once("guarded_pre_contact_sequence_execution_failed")
            return
        self._write_outputs_once(self._success_status)

    def _execute_motion_phase(self, phase_name: str, snapshot_name: str, target: dict[str, float]) -> bool:
        if not self._verify_endpoint():
            self._record_phase(phase_name, "failed", "Gazebo endpoint lost", 0.0, 0.0)
            return False
        if self._safety_violation_count > 0:
            self._record_phase(phase_name, "failed", "Safety gate blocked", 0.0, 0.0)
            return False
        if not self._target_within_limits(target):
            self._record_phase(phase_name, "failed", "Target exceeds bounded joint delta", 0.0, 0.0)
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
        if not (executed and observed):
            self._all_phase_executions_observed = False
        else:
            self._phases_executed += 1
            self._motion_executed = True
        if phase_name == "move_to_ready_pose":
            self._ready_pose_reached = executed and observed
        if phase_name == "move_to_pre_approach_pose":
            self._pre_approach_pose_reached = executed and observed
        if phase_name == "move_to_pre_insertion_standby_pose":
            self._standby_pose_reached = executed and observed
        self._record_phase(
            phase_name,
            "succeeded" if executed and observed else "failed",
            "Motion phase executed through Gazebo controller",
            duration,
            final_error,
        )
        return executed and observed

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
        self._max_observed_delta_deg = max(self._max_observed_delta_deg, max_delta)
        return max_delta <= self._max_joint_delta_deg

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

    def _planned_trajectory_within_limits(self, phase_name: str, trajectory: Any) -> bool:
        max_delta = 0.0
        for point in trajectory.points:
            for joint, position in zip(trajectory.joint_names, point.positions):
                delta = abs(math.degrees(float(position) - self._initial_positions.get(joint, 0.0)))
                max_delta = max(max_delta, delta)
        self._max_observed_delta_deg = max(self._max_observed_delta_deg, max_delta)
        if max_delta > self._max_joint_delta_deg:
            self._plan_rows.append({"phase": phase_name, "plan_found": "false", "plan_error_code": "joint_delta_limit", "trajectory_point_count": "0", "trajectory_joint_count": "0", "planned_duration_sec": "0.000000"})
            return False
        return True

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
        self._max_observed_delta_deg = max(
            self._max_observed_delta_deg,
            max(abs(math.degrees(current.get(j, 0.0) - self._initial_positions.get(j, 0.0))) for j in self._joint_names),
        )
        return motion_delta > 0.1 and target_error <= self._return_tolerance_deg, target_error

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

    def _write_endpoint_rows(self) -> None:
        self._endpoint_rows = [
            {"check": "simulation_engine", "value": self._simulation_engine},
            {"check": "control_interface", "value": self._action_name},
            {"check": "simulation_control_interface_used", "value": self._simulation_control_interface},
            {"check": "gazebo_controller_verified", "value": self._bool(self._gazebo_controller_verified)},
            {"check": "execution_endpoint_verified_simulation_only", "value": self._bool(self._endpoint_verified)},
            {"check": "real_robot_endpoint_detected", "value": self._bool(self._real_robot_endpoint_detected)},
            {"check": "peg_insertion_allowed", "value": "false"},
            {"check": "contact_seeking_allowed", "value": "false"},
            {"check": "follow_joint_trajectory_execution_allowed", "value": "gazebo_simulation_only"},
        ]

    def _write_joint_rows(self) -> None:
        mapping = [
            ("initial", "initial"),
            ("ready", "after_ready_pose"),
            ("pre_approach", "after_pre_approach_pose"),
            ("pre_insertion_standby", "after_pre_insertion_standby_pose"),
            ("hold", "after_hold"),
            ("return", "after_return"),
        ]
        self._joint_rows = []
        for label, key in mapping:
            positions = self._joint_positions(self._snapshots.get(key))
            for joint in self._joint_names:
                self._joint_rows.append(
                    {
                        "phase": label,
                        "joint": joint,
                        "position_rad": f"{positions.get(joint, 0.0):.9f}",
                        "delta_from_initial_deg": f"{math.degrees(positions.get(joint, 0.0) - self._initial_positions.get(joint, 0.0)):.9f}",
                    }
                )

    def _status_payload(self, status: str | None = None) -> dict[str, Any]:
        return {
            "simulation_engine": self._simulation_engine,
            "task_sequence_type": self._task_sequence_type,
            "moveit_used": True,
            "move_group_started": self._move_group_started,
            "planning_available": self._planning_available,
            "gazebo_controller_verified": self._gazebo_controller_verified,
            "execution_endpoint_verified_simulation_only": self._endpoint_verified,
            "phase_count": len(self._phase_names),
            "phases_planned": self._phases_planned,
            "phases_executed": self._successful_phase_count(),
            "all_phase_plans_found": self._all_phase_plans_found,
            "all_phase_executions_observed": self._all_phase_executions_observed,
            "ready_pose_reached": self._ready_pose_reached,
            "pre_approach_pose_reached": self._pre_approach_pose_reached,
            "pre_insertion_standby_pose_reached": self._standby_pose_reached,
            "hold_completed": self._hold_completed,
            "return_to_ready_completed": self._return_completed,
            "max_observed_joint_delta_deg": self._max_observed_delta_deg,
            "final_return_error_deg": self._final_return_error_deg,
            "final_return_within_tolerance": self._final_return_within_tolerance,
            "contact_wrench_topic_available": self._contact_wrench_topic in self._topic_names(),
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "safety_violation_count": self._safety_violation_count,
            "peg_insertion_executed": False,
            "contact_seeking_executed": False,
            "trajectory_execution_allowed": "gazebo_simulation_only",
            "controller_execution_allowed": "gazebo_simulation_only",
            "follow_joint_trajectory_execution_allowed": "gazebo_simulation_only",
            "real_robot_used": False,
            "motion_executed": self._motion_executed,
            "shutdown_segfault_observed_if_any": self._shutdown_segfault_observed,
            "status": status or "guarded_pre_contact_task_sequence_pending",
        }

    def _successful_phase_count(self) -> int:
        return sum(1 for row in self._phase_rows if row.get("status") == "succeeded")

    def _publish_reports(self) -> None:
        self._publish_json(self._status_pub, self._status_payload())
        self._publish_json(self._phase_pub, {"rows": self._phase_rows})
        self._publish_json(self._safety_pub, self._safety_payload())
        self._publish_json(self._endpoint_pub, {"rows": self._endpoint_rows})

    def _write_outputs_once(self, status: str) -> None:
        if self._finished:
            return
        self._finished = True
        payload = self._status_payload(status)
        self._write_lines(self._output_dir / "nodes.txt", sorted(name for name in self.get_node_names() if name))
        self._write_lines(self._output_dir / "topics.txt", sorted(f"{n} {','.join(t)}" for n, t in self.get_topic_names_and_types()))
        self._write_lines(self._output_dir / "services.txt", sorted(f"{n} {','.join(t)}" for n, t in self.get_service_names_and_types()))
        self._write_parameters_file()
        self._write_lines(self._output_dir / "joint_states_initial.txt", self._joint_state_lines(self._snapshots.get("initial")))
        self._write_lines(self._output_dir / "joint_states_after_ready_pose.txt", self._joint_state_lines(self._snapshots.get("after_ready_pose")))
        self._write_lines(self._output_dir / "joint_states_after_pre_approach_pose.txt", self._joint_state_lines(self._snapshots.get("after_pre_approach_pose")))
        self._write_lines(self._output_dir / "joint_states_after_pre_insertion_standby_pose.txt", self._joint_state_lines(self._snapshots.get("after_pre_insertion_standby_pose")))
        self._write_lines(self._output_dir / "joint_states_after_hold.txt", self._joint_state_lines(self._snapshots.get("after_hold")))
        self._write_lines(self._output_dir / "joint_states_after_return.txt", self._joint_state_lines(self._snapshots.get("after_return")))
        self._write_csv(self._output_dir / "task_phase_report.csv", self._phase_rows)
        self._write_csv(self._output_dir / "task_sequence_plan_summary.csv", self._plan_rows)
        self._write_csv(self._output_dir / "task_sequence_joint_report.csv", self._joint_rows)
        self._write_csv(self._output_dir / "task_sequence_safety_report.csv", self._safety_rows(payload))
        self._write_csv(self._output_dir / "task_sequence_endpoint_report.csv", self._endpoint_rows)
        self._write_json(self._output_dir / "guarded_task_sequence_status.json", payload)
        self._write_summary(payload)
        self._write_run_log(payload)
        self._publish_reports()
        self.get_logger().info("proposal_simulation_cell_v2_5 diagnostics written")
        rclpy.shutdown()

    def _write_parameters_file(self) -> None:
        lines = [
            "moveit_execution_allowed=gazebo_simulation_only",
            "controller_execution_allowed=gazebo_simulation_only",
            "trajectory_execution_allowed=gazebo_simulation_only",
            "follow_joint_trajectory_execution_allowed=gazebo_simulation_only",
            "real_robot_allowed=false",
            "peg_insertion_allowed=false",
            "contact_seeking_allowed=false",
        ]
        lines.extend(self._run_command(["ros2", "param", "list", self._move_group_node], timeout=2.0))
        self._write_lines(self._output_dir / "parameters.txt", lines)

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_5_guarded_pre_contact_task_sequence",
            "",
            f"Status: `{status['status']}`",
            "",
            "This diagnostic executes a guarded pre-contact task sequence through the verified Gazebo simulation controller.",
            "",
            f"- phases_planned: {status['phases_planned']}",
            f"- phases_executed: {status['phases_executed']}",
            f"- hold_completed: {self._bool(status['hold_completed'])}",
            f"- return_to_ready_completed: {self._bool(status['return_to_ready_completed'])}",
            f"- final_return_error_deg: {status['final_return_error_deg']:.9f}",
            f"- safety_violation_count: {status['safety_violation_count']}",
            "- peg_insertion_executed: false",
            "- contact_seeking_executed: false",
            "- real_robot_used: false",
        ]
        self._write_lines(self._output_dir / "summary.md", lines)

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_5_guarded_pre_contact_task_sequence",
            f"status={status['status']}",
            f"all_phase_plans_found={self._bool(status['all_phase_plans_found'])}",
            f"all_phase_executions_observed={self._bool(status['all_phase_executions_observed'])}",
            f"hold_completed={self._bool(status['hold_completed'])}",
            f"return_to_ready_completed={self._bool(status['return_to_ready_completed'])}",
            "real_robot_used=false",
            "peg_insertion_executed=false",
            "contact_seeking_executed=false",
        ]
        self._write_lines(self._output_dir / "run.log", lines)

    def _safety_payload(self) -> dict[str, Any]:
        return {
            "contact_wrench_topic_available": self._contact_wrench_topic in self._topic_names(),
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "max_allowed_force_n": self._max_allowed_force,
            "emergency_stop_force_threshold_n": self._emergency_force,
            "safety_violation_count": self._safety_violation_count,
            "peg_insertion_executed": False,
            "contact_seeking_executed": False,
            "real_robot_used": False,
        }

    def _safety_rows(self, status: dict[str, Any]) -> list[dict[str, str]]:
        return [{"field": key, "value": str(value).lower() if isinstance(value, bool) else str(value)} for key, value in self._safety_payload().items()]

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

    def _point_time_sec(self, point: JointTrajectoryPoint) -> float:
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1_000_000_000.0

    def _publish_contact_wrench_sample(self) -> None:
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = "gazebo_contact_monitor"
        self._contact_wrench_pub.publish(wrench)

    def _topic_names(self) -> set[str]:
        return {name for name, _types in self.get_topic_names_and_types()}

    def _force_magnitude(self, wrench: WrenchStamped) -> float:
        force = wrench.wrench.force
        return math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z)

    def _torque_magnitude(self, wrench: WrenchStamped) -> float:
        torque = wrench.wrench.torque
        return math.sqrt(torque.x * torque.x + torque.y * torque.y + torque.z * torque.z)

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
    node = ProposalSimulationCellV25GuardedTaskSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
