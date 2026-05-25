"""MoveIt-generated Gazebo-only execution validation for proposal_simulation_cell_v2_4."""

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


class ProposalSimulationCellV24MoveItGazeboExecutionNode(Node):
    """Plan with MoveIt and execute one bounded trajectory through Gazebo only."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_4_moveit_gazebo_execution_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_4")

        self._config = self._load_config()
        robot = self._config.get("robot", {})
        execution = self._config.get("moveit_gazebo_execution", {})
        motion = self._config.get("motion", {})
        safety = self._config.get("safety_limits", {})
        validation = self._config.get("validation", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_4")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._simulation_engine = str(execution.get("simulation_engine", "gazebo"))
        self._robot_model = str(robot.get("gazebo_robot_model", "lbr_iisy3_r760"))
        self._group = str(execution.get("selected_group", robot.get("selected_group", "manipulator")))
        self._tool_link = str(execution.get("selected_end_effector_link", robot.get("selected_end_effector_link", "tool0")))
        self._joint_names = [str(name) for name in robot.get("controller_joint_names", [])]
        self._selected_joint = str(motion.get("selected_joint", "joint_6"))
        self._target_delta_deg = float(motion.get("target_joint_delta_deg", 2.0))
        self._max_joint_delta_deg = float(motion.get("max_joint_delta_deg", 3.0))
        self._max_execution_duration = float(motion.get("max_execution_duration_sec", 15.0))
        self._return_to_initial = bool(motion.get("return_to_initial_after_execution", True))
        self._return_tolerance_deg = float(motion.get("return_tolerance_deg", 0.5))
        self._return_duration = float(motion.get("return_duration_sec", 3.0))
        self._planning_time = float(motion.get("plan_allowed_time_sec", 3.0))
        self._planning_attempts = int(motion.get("planning_attempts", 1))
        self._joint_goal_tolerance = float(motion.get("joint_goal_tolerance_rad", 0.01))
        self._velocity_scale = float(motion.get("max_velocity_scaling_factor", 0.1))
        self._acceleration_scale = float(motion.get("max_acceleration_scaling_factor", 0.1))
        self._compute_ik_service = str(execution.get("compute_ik_service", "/compute_ik"))
        self._plan_service = str(execution.get("plan_service", "/plan_kinematic_path"))
        self._move_group_node = str(execution.get("move_group_node_name", "/move_group"))
        self._action_name = str(execution.get("control_interface", "/joint_trajectory_controller/follow_joint_trajectory"))
        self._simulation_control_interface = str(
            execution.get(
                "simulation_control_interface_used",
                "gz_ros2_control/GazeboSimSystem via joint_trajectory_controller",
            )
        )
        self._joint_states_topic = str(validation.get("joint_states_topic", "/joint_states"))
        self._contact_wrench_topic = str(validation.get("contact_wrench_topic", "/proposal_simulation_cell/contact_wrench"))
        self._validation_timeout = float(validation.get("validation_timeout_sec", 120.0))
        self._startup_wait = float(validation.get("startup_wait_sec", 5.0))
        self._success_status = str(validation.get("status_success", "moveit_gazebo_execution_validated"))
        self._max_allowed_force = float(safety.get("max_allowed_force_n", 50.0))
        self._max_allowed_torque = float(safety.get("max_allowed_torque_nm", 5.0))
        self._emergency_force = float(safety.get("emergency_stop_force_threshold_n", 45.0))

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/moveit_gazebo_execution_status")),
            10,
        )
        self._joint_report_pub = self.create_publisher(
            String,
            str(validation.get("joint_report_topic", "/proposal_simulation_cell/moveit_gazebo_execution_joint_report")),
            10,
        )
        self._safety_report_pub = self.create_publisher(
            String,
            str(validation.get("safety_report_topic", "/proposal_simulation_cell/moveit_gazebo_execution_safety_report")),
            10,
        )
        self._endpoint_report_pub = self.create_publisher(
            String,
            str(validation.get("endpoint_report_topic", "/proposal_simulation_cell/moveit_gazebo_execution_endpoint_report")),
            10,
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
        self._compute_ik_available = False
        self._planning_available = False
        self._plan_request_sent = False
        self._plan_solution_found = False
        self._planned_point_count = 0
        self._planned_joint_count = 0
        self._planned_duration_sec = 0.0
        self._gazebo_controller_verified = False
        self._endpoint_verified = False
        self._trajectory_sent = False
        self._motion_executed = False
        self._motion_observed = False
        self._return_commanded = False
        self._final_return_error_deg = 0.0
        self._final_return_within_tolerance = False
        self._max_joint_delta_deg_observed = 0.0
        self._max_force = 0.0
        self._max_torque = 0.0
        self._safety_violation_count = 0
        self._real_robot_endpoint_detected = False
        self._snapshots: dict[str, JointState | None] = {}
        self._initial_positions: dict[str, float] = {}
        self._after_execution_positions: dict[str, float] = {}
        self._after_return_positions: dict[str, float] = {}
        self._plan_rows: list[dict[str, str]] = []
        self._joint_rows: list[dict[str, str]] = []
        self._endpoint_rows: list[dict[str, str]] = []
        self._shutdown_segfault_observed = "unknown_after_launch_shutdown"

        self.create_timer(0.2, self._tick)
        self.get_logger().info("proposal_simulation_cell_v2_4 MoveIt Gazebo execution node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v2.4 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

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
            threading.Thread(target=self._run_validation, daemon=True).start()
        if elapsed >= self._validation_timeout and not self._finished:
            self._write_outputs_once("moveit_gazebo_execution_timeout")

    def _run_validation(self) -> None:
        self._move_group_started = self._parameter_client.wait_for_service(timeout_sec=10.0)
        self._collect_robot_description()
        self._compute_ik_available = self._ik_client.wait_for_service(timeout_sec=8.0)
        self._planning_available = self._plan_client.wait_for_service(timeout_sec=8.0)
        self._activate_gazebo_controllers()
        self._gazebo_controller_verified = self._action_client.wait_for_server(timeout_sec=15.0) or self._action_services_present()
        self._endpoint_verified = self._verify_gazebo_execution_endpoint()
        self._write_endpoint_rows()
        if not self._endpoint_verified:
            self._write_outputs_once("gazebo_execution_endpoint_not_verified")
            return
        if not self._compute_ik_available or not self._planning_available or not self._move_group_started:
            self._write_outputs_once("moveit_planning_endpoint_unavailable")
            return
        if not self._wait_for_joint_state(timeout_sec=30.0):
            self._write_outputs_once("joint_states_unavailable")
            return
        self._snapshots["before_execution"] = self._last_joint_state
        self._initial_positions = self._joint_positions(self._last_joint_state)
        plan_trajectory = self._request_plan()
        if plan_trajectory is None:
            self._write_outputs_once("moveit_plan_failed")
            return
        if not self._planned_trajectory_within_limits(plan_trajectory):
            self._write_outputs_once("planned_trajectory_exceeds_limit")
            return
        if self._safety_violation_count > 0:
            self._write_outputs_once("safety_violation_before_execution")
            return
        execution_ok = self._execute_trajectory(plan_trajectory)
        self._snapshots["after_execution"] = self._last_joint_state
        self._after_execution_positions = self._joint_positions(self._last_joint_state)
        self._update_motion_observed()
        if not execution_ok:
            self._write_outputs_once("moveit_plan_validated_gazebo_execution_failed")
            return
        if self._return_to_initial and self._safety_violation_count == 0:
            self._return_commanded = self._send_return_to_initial()
        self._snapshots["after_return"] = self._last_joint_state
        self._after_return_positions = self._joint_positions(self._last_joint_state)
        self._calculate_return_error()
        self._write_joint_rows()
        if self._safety_violation_count > 0:
            self._write_outputs_once("safety_violation_detected")
            return
        if not self._motion_observed:
            self._write_outputs_once("moveit_plan_validated_gazebo_execution_failed")
            return
        if self._return_to_initial and not self._final_return_within_tolerance:
            self._write_outputs_once("gazebo_return_to_initial_failed")
            return
        self._write_outputs_once(self._success_status)

    def _collect_robot_description(self) -> None:
        if self._robot_description or not self._parameter_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ["robot_description"]
        future = self._parameter_client.call_async(request)
        if self._wait_for_future(future, 5.0) and future.result() is not None and future.result().values:
            self._robot_description = future.result().values[0].string_value

    def _verify_gazebo_execution_endpoint(self) -> bool:
        lowered = self._robot_description.lower()
        self._real_robot_endpoint_detected = any(term in lowered for term in ["fri", "ip_address", "port_id"])
        controller_services = {name for name, _types in self.get_service_names_and_types()}
        controller_manager_present = any(name.startswith("/controller_manager/") for name in controller_services)
        gz_ros_control_present = any(name.startswith("/gz_ros_control/") for name in controller_services)
        action_present = self._action_services_present()
        return bool(
            self._gazebo_controller_verified
            and action_present
            and controller_manager_present
            and gz_ros_control_present
            and not self._real_robot_endpoint_detected
        )

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

    def _action_services_present(self) -> bool:
        service_names = {name for name, _types in self.get_service_names_and_types()}
        return all(
            f"{self._action_name}/_action/{suffix}" in service_names
            for suffix in ("send_goal", "get_result", "cancel_goal")
        )

    def _request_plan(self) -> Any | None:
        current = self._joint_positions(self._last_joint_state)
        goal_positions = dict(current)
        goal_positions[self._selected_joint] = current.get(self._selected_joint, 0.0) + math.radians(self._target_delta_deg)
        request = GetMotionPlan.Request()
        motion_request = MotionPlanRequest()
        motion_request.group_name = self._group
        motion_request.num_planning_attempts = self._planning_attempts
        motion_request.allowed_planning_time = self._planning_time
        motion_request.max_velocity_scaling_factor = self._velocity_scale
        motion_request.max_acceleration_scaling_factor = self._acceleration_scale
        motion_request.start_state = self._robot_state_from_positions(current)
        goal = Constraints()
        for joint_name in self._joint_names:
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = float(goal_positions.get(joint_name, 0.0))
            constraint.tolerance_above = self._joint_goal_tolerance
            constraint.tolerance_below = self._joint_goal_tolerance
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)
        motion_request.goal_constraints.append(goal)
        request.motion_plan_request = motion_request
        self._plan_request_sent = True
        future = self._plan_client.call_async(request)
        if not self._wait_for_future(future, self._planning_time + 8.0):
            self._plan_rows.append({"field": "plan_response_received", "value": "false"})
            return None
        response = future.result()
        if response is None:
            return None
        error_code = int(response.motion_plan_response.error_code.val)
        self._plan_solution_found = error_code == MoveItErrorCodes.SUCCESS
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        self._planned_point_count = len(trajectory.points)
        self._planned_joint_count = len(trajectory.joint_names)
        if trajectory.points:
            self._planned_duration_sec = self._point_time_sec(trajectory.points[-1])
        self._plan_rows = [
            {"field": "plan_request_sent", "value": "true"},
            {"field": "plan_solution_found", "value": self._bool(self._plan_solution_found)},
            {"field": "plan_error_code", "value": str(error_code)},
            {"field": "planned_trajectory_point_count", "value": str(self._planned_point_count)},
            {"field": "planned_trajectory_joint_count", "value": str(self._planned_joint_count)},
            {"field": "planned_duration_sec", "value": f"{self._planned_duration_sec:.6f}"},
            {"field": "max_joint_delta_deg_limit", "value": f"{self._max_joint_delta_deg:.6f}"},
        ]
        return trajectory if self._plan_solution_found and self._planned_point_count > 0 else None

    def _planned_trajectory_within_limits(self, trajectory: Any) -> bool:
        if self._planned_duration_sec > self._max_execution_duration:
            return False
        initial = self._initial_positions
        max_delta = 0.0
        for point in trajectory.points:
            for joint_name, position in zip(trajectory.joint_names, point.positions):
                delta = abs(math.degrees(float(position) - initial.get(joint_name, 0.0)))
                max_delta = max(max_delta, delta)
        self._max_joint_delta_deg_observed = max(self._max_joint_delta_deg_observed, max_delta)
        return max_delta <= self._max_joint_delta_deg

    def _execute_trajectory(self, trajectory: Any) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._trajectory_sent = True
        future = self._action_client.send_goal_async(goal)
        if not self._wait_for_future(future, 5.0):
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, self._max_execution_duration + 5.0):
            return False
        result = result_future.result()
        self._motion_executed = result is not None
        return self._motion_executed

    def _send_return_to_initial(self) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = [self._initial_positions.get(name, 0.0) for name in self._joint_names]
        point.time_from_start.sec = int(self._return_duration)
        point.time_from_start.nanosec = int((self._return_duration - int(self._return_duration)) * 1_000_000_000)
        goal.trajectory.points.append(point)
        future = self._action_client.send_goal_async(goal)
        if not self._wait_for_future(future, 5.0):
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, self._return_duration + 5.0):
            return False
        self._wait_for_settle(1.0)
        return result_future.result() is not None

    def _update_motion_observed(self) -> None:
        deltas = []
        after = self._joint_positions(self._last_joint_state)
        for joint in self._joint_names:
            deltas.append(abs(math.degrees(after.get(joint, 0.0) - self._initial_positions.get(joint, 0.0))))
        self._max_joint_delta_deg_observed = max(self._max_joint_delta_deg_observed, max(deltas) if deltas else 0.0)
        self._motion_observed = self._max_joint_delta_deg_observed > 0.25

    def _calculate_return_error(self) -> None:
        after_return = self._joint_positions(self._last_joint_state)
        errors = [
            abs(math.degrees(after_return.get(joint, 0.0) - self._initial_positions.get(joint, 0.0)))
            for joint in self._joint_names
        ]
        self._final_return_error_deg = max(errors) if errors else 0.0
        self._final_return_within_tolerance = self._final_return_error_deg <= self._return_tolerance_deg

    def _write_endpoint_rows(self) -> None:
        self._endpoint_rows = [
            {"check": "simulation_engine", "value": self._simulation_engine},
            {"check": "control_interface", "value": self._action_name},
            {"check": "simulation_control_interface_used", "value": self._simulation_control_interface},
            {"check": "gazebo_controller_verified", "value": self._bool(self._gazebo_controller_verified)},
            {"check": "execution_endpoint_verified_simulation_only", "value": self._bool(self._endpoint_verified)},
            {"check": "real_robot_endpoint_detected", "value": self._bool(self._real_robot_endpoint_detected)},
            {"check": "follow_joint_trajectory_execution_allowed", "value": "gazebo_simulation_only"},
        ]

    def _write_joint_rows(self) -> None:
        self._joint_rows = []
        for joint in self._joint_names:
            before = self._initial_positions.get(joint, 0.0)
            after = self._after_execution_positions.get(joint, before)
            returned = self._after_return_positions.get(joint, after)
            self._joint_rows.append(
                {
                    "joint": joint,
                    "before_rad": f"{before:.9f}",
                    "after_execution_rad": f"{after:.9f}",
                    "after_return_rad": f"{returned:.9f}",
                    "execution_delta_deg": f"{math.degrees(after - before):.9f}",
                    "return_error_deg": f"{abs(math.degrees(returned - before)):.9f}",
                }
            )

    def _status_payload(self, status: str | None = None) -> dict[str, Any]:
        return {
            "simulation_engine": self._simulation_engine,
            "moveit_used": True,
            "move_group_started": self._move_group_started,
            "compute_ik_service_available": self._compute_ik_available,
            "planning_available": self._planning_available,
            "plan_request_sent": self._plan_request_sent,
            "plan_solution_found": self._plan_solution_found,
            "planned_trajectory_point_count": self._planned_point_count,
            "planned_trajectory_joint_count": self._planned_joint_count,
            "gazebo_controller_verified": self._gazebo_controller_verified,
            "execution_endpoint_verified_simulation_only": self._endpoint_verified,
            "trajectory_execution_allowed": "gazebo_simulation_only",
            "controller_execution_allowed": "gazebo_simulation_only",
            "follow_joint_trajectory_execution_allowed": "gazebo_simulation_only",
            "trajectory_sent": self._trajectory_sent,
            "motion_executed": self._motion_executed,
            "motion_observed_in_joint_states": self._motion_observed,
            "max_observed_joint_delta_deg": self._max_joint_delta_deg_observed,
            "return_to_initial_commanded": self._return_commanded,
            "final_return_error_deg": self._final_return_error_deg,
            "final_return_within_tolerance": self._final_return_within_tolerance,
            "contact_wrench_topic_available": self._contact_wrench_topic in self._topic_names(),
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "safety_violation_count": self._safety_violation_count,
            "real_robot_used": False,
            "shutdown_segfault_observed_if_any": self._shutdown_segfault_observed,
            "status": status or "moveit_gazebo_execution_pending",
        }

    def _publish_reports(self) -> None:
        self._publish_json(self._status_pub, self._status_payload())
        self._publish_json(self._joint_report_pub, {"rows": self._joint_rows})
        self._publish_json(self._safety_report_pub, self._safety_payload())
        self._publish_json(self._endpoint_report_pub, {"rows": self._endpoint_rows})

    def _write_outputs_once(self, status: str) -> None:
        if self._finished:
            return
        self._finished = True
        payload = self._status_payload(status)
        self._write_lines(self._output_dir / "nodes.txt", sorted(name for name in self.get_node_names() if name))
        self._write_lines(
            self._output_dir / "topics.txt",
            sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types()),
        )
        self._write_lines(
            self._output_dir / "services.txt",
            sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types()),
        )
        self._write_parameters_file()
        self._write_lines(self._output_dir / "joint_states_before_execution.txt", self._joint_state_lines(self._snapshots.get("before_execution")))
        self._write_lines(self._output_dir / "joint_states_after_execution.txt", self._joint_state_lines(self._snapshots.get("after_execution")))
        self._write_lines(self._output_dir / "joint_states_after_return.txt", self._joint_state_lines(self._snapshots.get("after_return")))
        self._write_csv(self._output_dir / "moveit_plan_summary.csv", self._plan_rows)
        self._write_csv(self._output_dir / "moveit_gazebo_execution_joint_report.csv", self._joint_rows)
        self._write_csv(self._output_dir / "moveit_gazebo_execution_safety_report.csv", self._safety_rows(payload))
        self._write_csv(self._output_dir / "moveit_gazebo_execution_endpoint_report.csv", self._endpoint_rows)
        self._write_json(self._output_dir / "moveit_gazebo_execution_status.json", payload)
        self._write_summary(payload)
        self._write_run_log(payload)
        self._publish_reports()
        self.get_logger().info("proposal_simulation_cell_v2_4 diagnostics written")
        rclpy.shutdown()

    def _write_parameters_file(self) -> None:
        lines = [
            "moveit_execution_allowed=gazebo_simulation_only",
            "controller_execution_allowed=gazebo_simulation_only",
            "trajectory_execution_allowed=gazebo_simulation_only",
            "follow_joint_trajectory_execution_allowed=gazebo_simulation_only",
            "real_robot_allowed=false",
        ]
        lines.extend(self._run_command(["ros2", "param", "list", self._move_group_node], timeout=2.0))
        self._write_lines(self._output_dir / "parameters.txt", lines)

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_4_moveit_gazebo_execution_validation",
            "",
            f"Status: `{status['status']}`",
            "",
            "This diagnostic executes one small MoveIt-generated trajectory only through the verified Gazebo simulation joint trajectory controller.",
            "",
            f"- plan_solution_found: {self._bool(status['plan_solution_found'])}",
            f"- gazebo_controller_verified: {self._bool(status['gazebo_controller_verified'])}",
            f"- execution_endpoint_verified_simulation_only: {self._bool(status['execution_endpoint_verified_simulation_only'])}",
            f"- trajectory_sent: {self._bool(status['trajectory_sent'])}",
            f"- motion_observed_in_joint_states: {self._bool(status['motion_observed_in_joint_states'])}",
            f"- max_observed_joint_delta_deg: {status['max_observed_joint_delta_deg']:.9f}",
            f"- final_return_error_deg: {status['final_return_error_deg']:.9f}",
            f"- safety_violation_count: {status['safety_violation_count']}",
            "- real_robot_used: false",
        ]
        self._write_lines(self._output_dir / "summary.md", lines)

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_4_moveit_gazebo_execution_validation",
            f"status={status['status']}",
            f"plan_solution_found={self._bool(status['plan_solution_found'])}",
            f"execution_endpoint_verified_simulation_only={self._bool(status['execution_endpoint_verified_simulation_only'])}",
            f"trajectory_sent={self._bool(status['trajectory_sent'])}",
            f"motion_observed_in_joint_states={self._bool(status['motion_observed_in_joint_states'])}",
            "real_robot_used=false",
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
        }

    def _safety_rows(self, status: dict[str, Any]) -> list[dict[str, str]]:
        return [
            {"field": "contact_wrench_topic_available", "value": self._bool(status["contact_wrench_topic_available"])},
            {"field": "max_observed_force_n", "value": f"{self._max_force:.6f}"},
            {"field": "max_observed_torque_nm", "value": f"{self._max_torque:.6f}"},
            {"field": "safety_violation_count", "value": str(self._safety_violation_count)},
            {"field": "real_robot_used", "value": "false"},
        ]

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

    def _wait_for_settle(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            time.sleep(0.05)

    def _wait_for_future(self, future: Any, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return future.done()

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
    node = ProposalSimulationCellV24MoveItGazeboExecutionNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
