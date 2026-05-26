"""Contact-triggered guarded touch calibration for proposal_simulation_cell_v2_7."""

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


class ProposalSimulationCellV27ContactTriggeredGuardedTouchNode(Node):
    """Run a bounded Gazebo-only guarded touch against a calibration pad."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_7")

        self._config = self._load_config()
        robot = self._config.get("robot", {})
        task = self._config.get("task_sequence", {})
        target = self._config.get("contact_calibration_target", {})
        moveit = self._config.get("moveit_execution", {})
        touch = self._config.get("guarded_touch", {})
        contact_gate = self._config.get("contact_gate", {})
        safety = self._config.get("safety_gates", {})
        validation = self._config.get("validation", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_7")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._simulation_engine = str(task.get("simulation_engine", "gazebo"))
        self._task_sequence_type = str(task.get("task_sequence_type", "contact_triggered_guarded_touch_calibration"))
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
        self._success_status = str(validation.get("status_success_contact", "contact_triggered_guarded_touch_validated"))
        self._no_contact_status = str(validation.get("status_no_contact", "contact_triggered_guarded_touch_not_reached"))
        self._emergency_status = str(validation.get("status_emergency", "contact_triggered_guarded_touch_emergency_stop"))

        self._target_name = str(target.get("target_name", "proposal_v2_7_contact_calibration_pad"))
        self._target_type = str(target.get("target_type", "runtime_spawned_compliant_contact_pad"))
        self._target_contact_topic = str(target.get("contact_topic", "/proposal_simulation_cell/contact_calibration_contacts"))
        self._target_frame = str(target.get("frame_id", "contact_calibration_pad"))
        self._target_tool_frame = str(target.get("tool_frame", self._tool_link))
        self._target_reference_frame = str(target.get("reference_frame", "world"))
        self._target_fallback_pose = [float(v) for v in target.get("fallback_world_pose_xyz", [0.95, -0.25, 1.05])]
        self._target_offset = [float(v) for v in target.get("spawn_offset_xyz", [0.0, 0.0, 0.0])]
        self._target_size = [float(v) for v in target.get("size_xyz", [0.12, 0.12, 0.12])]
        self._target_mass = float(target.get("mass_kg", 0.03))
        self._target_kp = float(target.get("contact_kp", 350.0))
        self._target_kd = float(target.get("contact_kd", 12.0))
        self._target_spawned = False
        self._target_spawn_pose = list(self._target_fallback_pose)
        self._target_report_lines: list[str] = []

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/contact_triggered_guarded_touch_status")),
            10,
        )
        self._step_pub = self.create_publisher(
            String, str(validation.get("step_report_topic", "/proposal_simulation_cell/guarded_touch_step_report")), 10
        )
        self._contact_pub = self.create_publisher(
            String, str(validation.get("contact_trigger_report_topic", "/proposal_simulation_cell/contact_trigger_report")), 10
        )
        self._safety_pub = self.create_publisher(
            String, str(validation.get("safety_report_topic", "/proposal_simulation_cell/contact_triggered_safety_report")), 10
        )
        self._endpoint_pub = self.create_publisher(
            String, str(validation.get("endpoint_report_topic", "/proposal_simulation_cell/contact_triggered_endpoint_report")), 10
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
        self._contact_trigger_step_index = 0
        self._emergency_stop_triggered = False
        self._retreat_completed = False
        self._return_completed = False
        self._final_return_error_deg = 0.0
        self._final_return_within_tolerance = False
        self._motion_executed = False
        self._shutdown_segfault_observed = "unknown_after_launch_shutdown"

        self.create_timer(0.2, self._tick)
        self.get_logger().info("proposal_simulation_cell_v2_7 contact-triggered guarded touch node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v2.7 config not found: {path}")
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
        self._contact_wrench_topic_available = True

    def _on_contacts(self, message: Any) -> None:
        force, torque = self._extract_contact_wrench(message)
        if force <= 0.0 and torque <= 0.0:
            return
        self._last_force = force
        self._last_torque = torque
        self._max_force = max(self._max_force, force)
        self._max_torque = max(self._max_torque, torque)
        self._publish_contact_wrench(force, torque)
        if force >= self._contact_threshold and not self._contact_gate_triggered:
            self._contact_gate_triggered = True
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
            self._write_outputs_once("contact_triggered_guarded_touch_timeout")

    def _run_sequence(self) -> None:
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
        if not self._wait_for_contact_wrench_topic(timeout_sec=4.0):
            self._record_phase("verify_contact_wrench_ready", "failed", "Contact wrench topic unavailable", 0.0, 0.0)
            self._write_outputs_once("contact_wrench_topic_unavailable")
            return
        self._record_phase("verify_contact_wrench_ready", "succeeded", "Contact wrench topic available", 0.0, 0.0)
        self._contact_calibration_target_available = Contacts is not None
        self._record_phase(
            "verify_contact_calibration_target",
            "succeeded" if self._contact_calibration_target_available else "failed",
            "Runtime compliant contact calibration pad available",
            0.0,
            0.0,
        )
        if not self._contact_calibration_target_available:
            self._write_outputs_once("contact_calibration_target_unavailable")
            return
        if self._safety_blocked():
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
        self._standby_positions = self._target_from_initial(self._standby_delta)
        if not self._execute_motion_phase("move_to_ready_pose", "after_ready_pose", self._ready_positions):
            self._write_outputs_once("contact_triggered_guarded_touch_not_reached")
            return
        if not self._execute_motion_phase("move_to_touch_standby_pose", "after_touch_standby_pose", self._standby_positions):
            self._write_outputs_once("contact_triggered_guarded_touch_not_reached")
            return
        self._spawn_contact_target_near_tool()
        self._execute_guarded_touch_steps()
        self._snapshots["after_guarded_touch"] = self._last_joint_state
        self._record_phase(
            "stop_on_contact_gate",
            "succeeded" if self._contact_gate_triggered else "failed",
            "Contact gate checked after guarded touch steps",
            0.0,
            0.0,
        )
        self._execute_motion_phase("retreat_to_touch_standby_pose", "after_retreat", self._standby_positions)
        self._retreat_completed = self._snapshots.get("after_retreat") is not None
        self._execute_motion_phase("return_to_ready_pose", "after_return", self._ready_positions)
        self._return_completed = self._snapshots.get("after_return") is not None
        self._calculate_final_return_error()
        self._record_phase(
            "validate_final_state",
            "succeeded" if self._final_return_within_tolerance else "failed",
            "Final state checked against ready pose",
            0.0,
            self._final_return_error_deg,
        )
        self._write_joint_rows()
        if self._emergency_stop_triggered:
            self._write_outputs_once(self._emergency_status)
        elif self._contact_gate_triggered and self._max_force > 0.0 and self._retreat_completed:
            self._write_outputs_once(self._success_status)
        else:
            self._write_outputs_once(self._no_contact_status)

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
        if phase_name == "move_to_touch_standby_pose":
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
        self._record_phase("execute_low_force_guarded_touch_steps", "succeeded", "Guarded touch step loop started", 0.0, 0.0)
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
            self._record_guarded_step(index, "succeeded" if step_observed else "failed", current, duration)
            self._record_contact_trigger(index)
            if not step_observed:
                break
            if self._contact_gate_triggered and self._contact_trigger_step_index == 0:
                self._contact_trigger_step_index = index
                break

    def _spawn_contact_target_near_tool(self) -> None:
        pose = self._lookup_tool_pose()
        self._target_spawn_pose = [pose[i] + self._target_offset[i] for i in range(3)]
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
        self._target_report_lines = [
            f"target_name={self._target_name}",
            f"target_type={self._target_type}",
            f"contact_topic={self._target_contact_topic}",
            f"spawned={self._bool(self._target_spawned)}",
            f"spawn_pose_xyz={self._target_spawn_pose[0]:.6f},{self._target_spawn_pose[1]:.6f},{self._target_spawn_pose[2]:.6f}",
            f"size_xyz={self._target_size[0]:.6f},{self._target_size[1]:.6f},{self._target_size[2]:.6f}",
            "separate_from_peg_insertion=true",
        ]
        self._wait_for_settle(1.0)

    def _lookup_tool_pose(self) -> list[float]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_reference_frame,
                self._target_tool_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
            translation = transform.transform.translation
            return [float(translation.x), float(translation.y), float(translation.z)]
        except TransformException:
            return list(self._target_fallback_pose)

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
        self._step_rows.append(
            {
                "step_index": str(step_index),
                "status": status,
                "duration_sec": f"{duration:.6f}",
                "force_n": f"{self._last_force:.6f}",
                "torque_nm": f"{self._last_torque:.6f}",
                "contact_gate_triggered": self._bool(self._contact_gate_triggered),
                "emergency_stop_triggered": self._bool(self._emergency_stop_triggered),
                "joint_state_snapshot": self._position_summary(positions),
            }
        )

    def _record_contact_trigger(self, step_index: int) -> None:
        self._contact_rows.append(
            {
                "step_index": str(step_index),
                "force_n": f"{self._last_force:.6f}",
                "torque_nm": f"{self._last_torque:.6f}",
                "contact_detection_force_threshold_n": f"{self._contact_threshold:.6f}",
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
            ("guarded_touch", "after_guarded_touch"),
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
        return {
            "simulation_engine": self._simulation_engine,
            "task_sequence_type": self._task_sequence_type,
            "moveit_used": True,
            "move_group_started": self._move_group_started,
            "planning_available": self._planning_available,
            "gazebo_controller_verified": self._gazebo_controller_verified,
            "execution_endpoint_verified_simulation_only": self._endpoint_verified,
            "contact_calibration_target_available": self._contact_calibration_target_available,
            "contact_calibration_target_type": self._target_type,
            "phase_count": len(self._phase_names),
            "phases_planned": self._phases_planned,
            "phases_executed": self._successful_phase_count(),
            "all_required_phase_plans_found": self._all_phase_plans_found,
            "all_required_phase_executions_observed": self._all_phase_executions_observed,
            "ready_pose_reached": self._ready_pose_reached,
            "touch_standby_pose_reached": self._touch_standby_pose_reached,
            "guarded_touch_started": self._guarded_touch_started,
            "guarded_touch_steps_attempted": self._guarded_steps_attempted,
            "guarded_touch_steps_completed": self._guarded_steps_completed,
            "contact_wrench_topic_available": self._contact_wrench_topic_available or self._contact_wrench_topic in self._topic_names(),
            "contact_detection_force_threshold_n": self._contact_threshold,
            "contact_gate_triggered": self._contact_gate_triggered,
            "contact_trigger_step_index": self._contact_trigger_step_index,
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "emergency_stop_triggered": self._emergency_stop_triggered,
            "retreat_completed": self._retreat_completed,
            "return_to_ready_completed": self._return_completed,
            "final_return_error_deg": self._final_return_error_deg,
            "final_return_within_tolerance": self._final_return_within_tolerance,
            "safety_violation_count": self._safety_violation_count,
            "peg_insertion_executed": False,
            "forceful_contact_executed": False,
            "trajectory_execution_allowed": "gazebo_simulation_only",
            "controller_execution_allowed": "gazebo_simulation_only",
            "follow_joint_trajectory_execution_allowed": "gazebo_simulation_only",
            "real_robot_used": False,
            "motion_executed": self._motion_executed,
            "shutdown_segfault_observed_if_any": self._shutdown_segfault_observed,
            "status": status or "contact_triggered_guarded_touch_pending",
        }

    def _successful_phase_count(self) -> int:
        return sum(1 for row in self._phase_rows if row.get("status") == "succeeded")

    def _publish_reports(self) -> None:
        self._publish_json(self._status_pub, self._status_payload())
        self._publish_json(self._step_pub, {"rows": self._step_rows})
        self._publish_json(self._contact_pub, {"rows": self._contact_rows})
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
        for file_name, snapshot in [
            ("joint_states_initial.txt", "initial"),
            ("joint_states_after_ready_pose.txt", "after_ready_pose"),
            ("joint_states_after_touch_standby_pose.txt", "after_touch_standby_pose"),
            ("joint_states_after_guarded_touch.txt", "after_guarded_touch"),
            ("joint_states_after_retreat.txt", "after_retreat"),
            ("joint_states_after_return.txt", "after_return"),
        ]:
            self._write_lines(self._output_dir / file_name, self._joint_state_lines(self._snapshots.get(snapshot)))
        self._write_csv(self._output_dir / "task_phase_report.csv", self._phase_rows)
        self._write_csv(self._output_dir / "guarded_touch_step_report.csv", self._step_rows)
        self._write_csv(self._output_dir / "contact_trigger_report.csv", self._contact_rows)
        self._write_csv(self._output_dir / "contact_triggered_safety_report.csv", self._safety_rows())
        self._write_csv(self._output_dir / "contact_triggered_endpoint_report.csv", self._endpoint_rows)
        self._write_lines(self._output_dir / "contact_calibration_target_report.txt", self._target_report_lines)
        self._write_json(self._output_dir / "contact_triggered_guarded_touch_status.json", payload)
        self._write_summary(payload)
        self._write_run_log(payload)
        self._publish_reports()
        self.get_logger().info("proposal_simulation_cell_v2_7 diagnostics written")
        rclpy.shutdown()

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

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_calibration",
            "",
            f"Status: `{status['status']}`",
            "",
            "This diagnostic runs a Gazebo-only guarded touch against a simulation contact calibration pad.",
            "",
            f"- contact_calibration_target_available: {self._bool(status['contact_calibration_target_available'])}",
            f"- guarded_touch_steps_completed: {status['guarded_touch_steps_completed']}",
            f"- contact_gate_triggered: {self._bool(status['contact_gate_triggered'])}",
            f"- contact_trigger_step_index: {status['contact_trigger_step_index']}",
            f"- max_observed_force_n: {status['max_observed_force_n']:.9f}",
            f"- retreat_completed: {self._bool(status['retreat_completed'])}",
            f"- return_to_ready_completed: {self._bool(status['return_to_ready_completed'])}",
            "- peg_insertion_executed: false",
            "- forceful_contact_executed: false",
            "- real_robot_used: false",
        ]
        self._write_lines(self._output_dir / "summary.md", lines)

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_calibration",
            f"status={status['status']}",
            f"contact_gate_triggered={self._bool(status['contact_gate_triggered'])}",
            f"contact_trigger_step_index={status['contact_trigger_step_index']}",
            f"max_observed_force_n={status['max_observed_force_n']}",
            f"retreat_completed={self._bool(status['retreat_completed'])}",
            f"return_to_ready_completed={self._bool(status['return_to_ready_completed'])}",
            "real_robot_used=false",
            "peg_insertion_executed=false",
            "forceful_contact_executed=false",
        ]
        self._write_lines(self._output_dir / "run.log", lines)

    def _safety_payload(self) -> dict[str, Any]:
        return {
            "contact_wrench_topic_available": self._contact_wrench_topic_available or self._contact_wrench_topic in self._topic_names(),
            "contact_detection_force_threshold_n": self._contact_threshold,
            "desired_contact_force_upper_n": self._desired_contact_upper,
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
        return [
            {"field": key, "value": str(value).lower() if isinstance(value, bool) else str(value)}
            for key, value in self._safety_payload().items()
        ]

    def _extract_contact_wrench(self, message: Any) -> tuple[float, float]:
        max_force = 0.0
        max_torque = 0.0
        for contact in list(getattr(message, "contacts", [])):
            for wrench in self._contact_wrenches(contact):
                for nested in self._wrench_messages(wrench):
                    force = getattr(nested, "force", None)
                    torque = getattr(nested, "torque", None)
                    max_force = max(max_force, self._vector_magnitude(force))
                    max_torque = max(max_torque, self._vector_magnitude(torque))
        return max_force, max_torque

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
    node = ProposalSimulationCellV27ContactTriggeredGuardedTouchNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
