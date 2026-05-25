"""MoveIt/Gazebo model alignment and plan-only validation for proposal_simulation_cell_v2_3."""

from __future__ import annotations

import csv
import json
import math
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class ProposalSimulationCellV23ModelAlignmentPlanOnlyNode(Node):
    """Validate model alignment, IK repeatability, and plan-only service response."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_3_model_alignment_plan_only_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_3")

        self._config = self._load_config()
        robot = self._config.get("robot", {})
        moveit = self._config.get("moveit_diagnostic", {})
        repeatability = self._config.get("ik_repeatability", {})
        plan_only = self._config.get("plan_only_validation", {})
        validation = self._config.get("validation", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_3")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._robot_model = str(robot.get("moveit_robot_model", "lbr_iisy3_r760"))
        self._gazebo_reference_model = str(robot.get("gazebo_reference_model", "lbr_iisy3_r760"))
        self._group = str(moveit.get("selected_group", robot.get("moveit_group", "manipulator")))
        self._tool_link = str(moveit.get("selected_end_effector_link", robot.get("tool_link", "tool0")))
        self._base_link = str(robot.get("base_link", repeatability.get("frame_id", "base_link")))
        self._gazebo_joints = [str(name) for name in robot.get("gazebo_controller_joints", [])]
        self._moveit_joints = [str(name) for name in robot.get("moveit_expected_joints", [])]
        self._compute_ik_service = str(moveit.get("compute_ik_service", "/compute_ik"))
        self._plan_service = str(moveit.get("plan_service", "/plan_kinematic_path"))
        self._move_group_node = str(moveit.get("move_group_node_name", "/move_group"))
        self._ik_timeout_sec = float(repeatability.get("ik_timeout_sec", 2.0))
        self._pose_offsets = list(repeatability.get("nearby_pose_offsets_m", []))
        self._base_pose = repeatability.get("base_target_pose", {})
        self._seed_joint_state = repeatability.get("seed_joint_state", {})
        self._joint_repeatability_tolerance = float(repeatability.get("repeatability_joint_tolerance_rad", 0.25))
        self._allowed_planning_time = float(plan_only.get("allowed_planning_time_sec", 3.0))
        self._planning_attempts = int(plan_only.get("planning_attempts", 1))
        self._joint_goal_tolerance = float(plan_only.get("joint_goal_tolerance_rad", 0.01))
        self._velocity_scale = float(plan_only.get("max_velocity_scaling_factor", 0.1))
        self._acceleration_scale = float(plan_only.get("max_acceleration_scaling_factor", 0.1))
        self._validation_timeout = float(validation.get("validation_timeout_sec", 60.0))
        self._startup_wait = float(validation.get("startup_wait_sec", 2.0))
        self._success_status = str(validation.get("status_success", "moveit_model_alignment_and_plan_only_validated"))

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/moveit_model_alignment_plan_only_status")),
            10,
        )
        self._alignment_pub = self.create_publisher(
            String,
            str(validation.get("alignment_report_topic", "/proposal_simulation_cell/moveit_gazebo_model_alignment_report")),
            10,
        )
        self._ik_pub = self.create_publisher(
            String,
            str(validation.get("ik_repeatability_report_topic", "/proposal_simulation_cell/moveit_ik_repeatability_report")),
            10,
        )
        self._plan_pub = self.create_publisher(
            String,
            str(validation.get("plan_only_report_topic", "/proposal_simulation_cell/moveit_plan_only_report")),
            10,
        )
        self._block_pub = self.create_publisher(
            String,
            str(validation.get("execution_block_report_topic", "/proposal_simulation_cell/moveit_plan_execution_block_report")),
            10,
        )

        self.create_subscription(String, "/robot_description", self._on_robot_description, 10)
        self.create_subscription(String, "/robot_description_semantic", self._on_robot_description_semantic, 10)
        self._ik_client = self.create_client(GetPositionIK, self._compute_ik_service)
        self._plan_client = self.create_client(GetMotionPlan, self._plan_service)
        self._parameter_client = self.create_client(GetParameters, f"{self._move_group_node}/get_parameters")

        self._start_time = time.monotonic()
        self._finished = False
        self._started = False
        self._robot_description = ""
        self._robot_description_semantic = ""
        self._move_group_started = False
        self._compute_ik_available = False
        self._plan_service_available = False
        self._alignment_report: dict[str, Any] = {}
        self._ik_rows: list[dict[str, str]] = []
        self._ik_solutions: list[dict[str, Any]] = []
        self._plan_report: dict[str, Any] = {}
        self._plan_solution_found = False
        self._plan_error_code = 0
        self._trajectory_point_count = 0
        self._max_repeatability_error_rad = 0.0
        self._real_robot_endpoint_detected = False

        self.create_timer(0.2, self._tick)
        self.get_logger().info("proposal_simulation_cell_v2_3 model alignment and plan-only diagnostic node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v2.3 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _on_robot_description(self, message: String) -> None:
        self._robot_description = message.data

    def _on_robot_description_semantic(self, message: String) -> None:
        self._robot_description_semantic = message.data

    def _tick(self) -> None:
        if self._finished:
            return
        self._publish_reports()
        elapsed = time.monotonic() - self._start_time
        if not self._started and elapsed >= self._startup_wait:
            self._started = True
            threading.Thread(target=self._run_validation, daemon=True).start()
        if elapsed >= self._validation_timeout and not self._finished:
            self._write_outputs_once("moveit_plan_only_validation_timeout")

    def _run_validation(self) -> None:
        self._move_group_started = self._parameter_client.wait_for_service(timeout_sec=8.0)
        self._collect_robot_descriptions()
        self._real_robot_endpoint_detected = self._detect_real_robot_endpoint()
        self._alignment_report = self._build_alignment_report()
        if not self._move_group_started:
            self._write_outputs_once("moveit_diagnostic_launch_failed")
            return
        if not self._alignment_report.get("model_alignment_validated", False):
            self._write_outputs_once("moveit_gazebo_model_alignment_failed")
            return
        if self._real_robot_endpoint_detected:
            self._write_outputs_once("real_robot_endpoint_detected")
            return
        self._compute_ik_available = self._ik_client.wait_for_service(timeout_sec=8.0)
        self._plan_service_available = self._plan_client.wait_for_service(timeout_sec=8.0)
        if not self._compute_ik_available:
            self._write_outputs_once("compute_ik_service_unavailable")
            return
        self._run_ik_repeatability()
        if not self._ik_solutions:
            self._write_outputs_once("ik_repeatability_failed")
            return
        if self._max_repeatability_error_rad > self._joint_repeatability_tolerance:
            self._write_outputs_once("ik_repeatability_failed")
            return
        if not self._plan_service_available:
            self._write_outputs_once("moveit_plan_service_unavailable")
            return
        self._run_plan_only_validation()
        if self._plan_solution_found:
            self._write_outputs_once(self._success_status)
        else:
            self._write_outputs_once("moveit_plan_only_failed")

    def _collect_robot_descriptions(self) -> None:
        if self._robot_description and self._robot_description_semantic:
            return
        if not self._parameter_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ["robot_description", "robot_description_semantic"]
        future = self._parameter_client.call_async(request)
        if not self._wait_for_future(future, 5.0):
            return
        response = future.result()
        if response is None:
            return
        if len(response.values) >= 2:
            self._robot_description = self._robot_description or response.values[0].string_value
            self._robot_description_semantic = self._robot_description_semantic or response.values[1].string_value

    def _build_alignment_report(self) -> dict[str, Any]:
        semantic_groups: list[str] = []
        group_links: list[str] = []
        semantic_error = ""
        if self._robot_description_semantic:
            try:
                root = ET.fromstring(self._robot_description_semantic)
                for group in root.findall("group"):
                    name = group.attrib.get("name", "")
                    if name:
                        semantic_groups.append(name)
                    if name == self._group:
                        for chain in group.findall("chain"):
                            group_links.extend(
                                [chain.attrib.get("base_link", ""), chain.attrib.get("tip_link", "")]
                            )
                        for link in group.findall("link"):
                            group_links.append(link.attrib.get("name", ""))
            except ET.ParseError as exc:
                semantic_error = str(exc)
        moveit_joints_found = all(joint in self._robot_description for joint in self._moveit_joints)
        joint_sets_match = self._gazebo_joints == self._moveit_joints
        group_valid = self._group in semantic_groups
        tool_link_valid = self._tool_link in group_links
        base_link_valid = self._base_link in self._robot_description
        return {
            "moveit_robot_model": self._robot_model,
            "gazebo_reference_model": self._gazebo_reference_model,
            "moveit_expected_joints": self._moveit_joints,
            "gazebo_controller_joints": self._gazebo_joints,
            "joint_sets_match": joint_sets_match,
            "moveit_joints_found": moveit_joints_found,
            "selected_group": self._group,
            "semantic_groups": semantic_groups,
            "group_valid": group_valid,
            "selected_tool_link": self._tool_link,
            "group_links": [link for link in group_links if link],
            "tool_link_valid": tool_link_valid,
            "base_link_valid": base_link_valid,
            "semantic_parse_error": semantic_error,
            "real_robot_endpoint_detected": self._real_robot_endpoint_detected,
            "model_alignment_validated": bool(
                joint_sets_match and moveit_joints_found and group_valid and tool_link_valid and base_link_valid
            ),
        }

    def _detect_real_robot_endpoint(self) -> bool:
        lowered = self._robot_description.lower()
        if "mock_components/genericsystem" in lowered or "mock" in lowered:
            return False
        return any(term in lowered for term in ["fri", "ip_address", "port_id"])

    def _run_ik_repeatability(self) -> None:
        first_solution: list[float] | None = None
        seed_state = self._seed_robot_state()
        for index, offset in enumerate(self._pose_offsets, start=1):
            request = GetPositionIK.Request()
            request.ik_request.group_name = self._group
            request.ik_request.ik_link_name = self._tool_link
            request.ik_request.avoid_collisions = False
            request.ik_request.timeout = self._duration_from_seconds(self._ik_timeout_sec)
            request.ik_request.pose_stamped = self._pose_with_offset(offset)
            request.ik_request.robot_state = seed_state
            future = self._ik_client.call_async(request)
            response_received = self._wait_for_future(future, self._ik_timeout_sec + 5.0)
            response = future.result() if response_received else None
            error_code = int(response.error_code.val) if response is not None else MoveItErrorCodes.TIMED_OUT
            solution_found = response is not None and error_code == MoveItErrorCodes.SUCCESS
            joint_names = list(response.solution.joint_state.name) if response is not None else []
            joint_positions = [float(value) for value in response.solution.joint_state.position] if response is not None else []
            if solution_found:
                if first_solution is None:
                    first_solution = joint_positions
                repeatability_error = self._max_joint_error(first_solution, joint_positions)
                self._max_repeatability_error_rad = max(self._max_repeatability_error_rad, repeatability_error)
                self._ik_solutions.append({"joint_names": joint_names, "joint_positions": joint_positions})
                seed_state = self._robot_state_from_solution(joint_names, joint_positions)
            else:
                repeatability_error = 0.0
            self._ik_rows.append(
                {
                    "pose_index": str(index),
                    "offset_x_m": f"{float(offset.get('x', 0.0)):.6f}",
                    "offset_y_m": f"{float(offset.get('y', 0.0)):.6f}",
                    "offset_z_m": f"{float(offset.get('z', 0.0)):.6f}",
                    "response_received": self._bool(response_received),
                    "ik_solution_found": self._bool(solution_found),
                    "ik_error_code": str(error_code),
                    "returned_joint_count": str(len(joint_names)),
                    "repeatability_error_rad": f"{repeatability_error:.9f}",
                }
            )

    def _run_plan_only_validation(self) -> None:
        solution = self._ik_solutions[0]
        request = GetMotionPlan.Request()
        motion_request = MotionPlanRequest()
        motion_request.group_name = self._group
        motion_request.num_planning_attempts = self._planning_attempts
        motion_request.allowed_planning_time = self._allowed_planning_time
        motion_request.max_velocity_scaling_factor = self._velocity_scale
        motion_request.max_acceleration_scaling_factor = self._acceleration_scale
        motion_request.start_state = self._seed_robot_state()
        goal = Constraints()
        for joint_name, joint_position in zip(solution["joint_names"], solution["joint_positions"]):
            if joint_name in self._moveit_joints:
                constraint = JointConstraint()
                constraint.joint_name = joint_name
                constraint.position = float(joint_position)
                constraint.tolerance_above = self._joint_goal_tolerance
                constraint.tolerance_below = self._joint_goal_tolerance
                constraint.weight = 1.0
                goal.joint_constraints.append(constraint)
        motion_request.goal_constraints.append(goal)
        request.motion_plan_request = motion_request
        future = self._plan_client.call_async(request)
        response_received = self._wait_for_future(future, self._allowed_planning_time + 8.0)
        response = future.result() if response_received else None
        self._plan_error_code = int(response.motion_plan_response.error_code.val) if response is not None else MoveItErrorCodes.TIMED_OUT
        self._plan_solution_found = response is not None and self._plan_error_code == MoveItErrorCodes.SUCCESS
        if response is not None:
            self._trajectory_point_count = len(response.motion_plan_response.trajectory.joint_trajectory.points)
        self._plan_report = {
            "plan_service": self._plan_service,
            "plan_service_available": self._plan_service_available,
            "plan_request_sent": True,
            "plan_response_received": response_received,
            "plan_solution_found": self._plan_solution_found,
            "plan_error_code": self._plan_error_code,
            "trajectory_point_count": self._trajectory_point_count,
            "goal_joint_count": len(goal.joint_constraints),
            "plan_only": True,
            "trajectory_executed": False,
            "trajectory_sent": False,
        }

    def _pose_with_offset(self, offset: dict[str, Any]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._base_link
        pose.header.stamp = self.get_clock().now().to_msg()
        position = self._base_pose.get("position", {})
        orientation = self._base_pose.get("orientation", {})
        pose.pose.position.x = float(position.get("x", 0.36)) + float(offset.get("x", 0.0))
        pose.pose.position.y = float(position.get("y", 0.0)) + float(offset.get("y", 0.0))
        pose.pose.position.z = float(position.get("z", 0.42)) + float(offset.get("z", 0.0))
        pose.pose.orientation.x = float(orientation.get("x", 0.0))
        pose.pose.orientation.y = float(orientation.get("y", 0.0))
        pose.pose.orientation.z = float(orientation.get("z", 0.0))
        pose.pose.orientation.w = float(orientation.get("w", 1.0))
        return pose

    def _seed_robot_state(self) -> RobotState:
        state = RobotState()
        joint_state = JointState()
        joint_state.name = [str(name) for name in self._seed_joint_state.keys()]
        joint_state.position = [float(position) for position in self._seed_joint_state.values()]
        state.joint_state = joint_state
        return state

    def _robot_state_from_solution(self, joint_names: list[str], joint_positions: list[float]) -> RobotState:
        state = RobotState()
        joint_state = JointState()
        joint_state.name = list(joint_names)
        joint_state.position = [float(position) for position in joint_positions]
        state.joint_state = joint_state
        return state

    def _status_payload(self, status: str | None = None) -> dict[str, Any]:
        ik_success_count = sum(1 for row in self._ik_rows if row.get("ik_solution_found") == "true")
        return {
            "robot_model": self._robot_model,
            "gazebo_reference_model": self._gazebo_reference_model,
            "move_group_started": self._move_group_started,
            "robot_description_available": bool(self._robot_description),
            "robot_description_semantic_available": bool(self._robot_description_semantic),
            "model_alignment_validated": bool(self._alignment_report.get("model_alignment_validated", False)),
            "compute_ik_service_available": self._compute_ik_available,
            "ik_pose_count": len(self._pose_offsets),
            "ik_success_count": ik_success_count,
            "ik_repeatability_within_tolerance": self._max_repeatability_error_rad <= self._joint_repeatability_tolerance,
            "max_repeatability_error_rad": self._max_repeatability_error_rad,
            "plan_service_available": self._plan_service_available,
            "plan_request_sent": bool(self._plan_report.get("plan_request_sent", False)),
            "plan_solution_found": self._plan_solution_found,
            "plan_error_code": self._plan_error_code,
            "trajectory_point_count": self._trajectory_point_count,
            "trajectory_execution_allowed": False,
            "controller_execution_allowed": False,
            "follow_joint_trajectory_execution_allowed": False,
            "planning_execution_allowed": False,
            "real_robot_used": False,
            "motion_executed": False,
            "trajectory_sent": False,
            "status": status or "moveit_model_alignment_plan_only_pending",
        }

    def _execution_block_report(self) -> dict[str, Any]:
        return {
            "trajectory_execution_allowed": False,
            "controller_execution_allowed": False,
            "follow_joint_trajectory_execution_allowed": False,
            "planning_execution_allowed": False,
            "real_robot_used": False,
            "motion_executed": False,
            "trajectory_sent": False,
            "real_robot_endpoint_detected": self._real_robot_endpoint_detected,
        }

    def _publish_reports(self) -> None:
        self._publish_json(self._status_pub, self._status_payload())
        self._publish_json(self._alignment_pub, self._alignment_report)
        self._publish_json(self._ik_pub, {"rows": self._ik_rows})
        self._publish_json(self._plan_pub, self._plan_report)
        self._publish_json(self._block_pub, self._execution_block_report())

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
        self._write_json(self._output_dir / "moveit_model_alignment_plan_only_status.json", payload)
        self._write_json(self._output_dir / "moveit_gazebo_model_alignment_report.json", self._alignment_report)
        self._write_csv(self._output_dir / "ik_repeatability_report.csv", self._ik_rows)
        self._write_yaml(self._output_dir / "ik_repeatability_solutions.yaml", {"solutions": self._ik_solutions})
        self._write_json(self._output_dir / "plan_only_report.json", self._plan_report)
        self._write_csv(
            self._output_dir / "moveit_plan_execution_block_report.csv",
            [{"check": key, "value": str(value).lower()} for key, value in self._execution_block_report().items()],
        )
        self._write_summary(payload)
        self._write_run_log(payload)
        self._publish_reports()
        self.get_logger().info("proposal_simulation_cell_v2_3 diagnostics written")
        rclpy.shutdown()

    def _write_parameters_file(self) -> None:
        lines = [
            "allow_trajectory_execution=false",
            "moveit_manage_controllers=false",
            "trajectory_execution_allowed=false",
            "planning_execution_allowed=false",
        ]
        lines.extend(self._run_command(["ros2", "param", "list", self._move_group_node], timeout=2.0))
        self._write_lines(self._output_dir / "parameters.txt", lines)

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_3_moveit_model_alignment_and_plan_only_validation",
            "",
            f"Status: `{status['status']}`",
            "",
            "This diagnostic audits MoveIt/Gazebo model alignment, checks IK repeatability over five nearby poses, and validates a MoveIt plan-only service response.",
            "",
            f"- model_alignment_validated: {self._bool(status['model_alignment_validated'])}",
            f"- ik_success_count: {status['ik_success_count']} / {status['ik_pose_count']}",
            f"- max_repeatability_error_rad: {status['max_repeatability_error_rad']:.9f}",
            f"- plan_solution_found: {self._bool(status['plan_solution_found'])}",
            f"- plan_error_code: {status['plan_error_code']}",
            f"- trajectory_point_count: {status['trajectory_point_count']}",
            "- trajectory_execution_allowed: false",
            "- controller_execution_allowed: false",
            "- follow_joint_trajectory_execution_allowed: false",
            "- real_robot_used: false",
            "- motion_executed: false",
            "- trajectory_sent: false",
        ]
        self._write_lines(self._output_dir / "summary.md", lines)

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_3_moveit_model_alignment_and_plan_only_validation",
            f"status={status['status']}",
            f"model_alignment_validated={self._bool(status['model_alignment_validated'])}",
            f"ik_success_count={status['ik_success_count']}",
            f"plan_solution_found={self._bool(status['plan_solution_found'])}",
            "trajectory_execution_allowed=false",
            "controller_execution_allowed=false",
            "follow_joint_trajectory_execution_allowed=false",
            "real_robot_used=false",
        ]
        self._write_lines(self._output_dir / "run.log", lines)

    def _duration_from_seconds(self, seconds: float) -> Duration:
        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
        return duration

    def _max_joint_error(self, reference: list[float], current: list[float]) -> float:
        if not reference or not current:
            return math.inf
        return max(abs(a - b) for a, b in zip(reference, current))

    def _wait_for_future(self, future: Any, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return future.done()

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

    def _write_yaml(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

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
    node = ProposalSimulationCellV23ModelAlignmentPlanOnlyNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
