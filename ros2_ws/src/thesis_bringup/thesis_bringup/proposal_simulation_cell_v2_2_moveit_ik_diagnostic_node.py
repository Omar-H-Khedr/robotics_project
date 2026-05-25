"""Diagnostic-only MoveIt IK validation for proposal_simulation_cell_v2_2."""

from __future__ import annotations

import csv
import json
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
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class ProposalSimulationCellV22MoveItIkDiagnosticNode(Node):
    """Call /compute_ik once with execution paths blocked."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_2_moveit_ik_diagnostic_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_2")

        self._config = self._load_config()
        robot = self._config.get("robot", {})
        moveit = self._config.get("moveit_diagnostic", {})
        request = self._config.get("ik_request", {})
        execution = self._config.get("execution_policy", {})
        validation = self._config.get("validation", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_2")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._robot_model = str(robot.get("moveit_robot_model", robot.get("robot_model", "lbr_iisy3_r760")))
        self._expected_joints = [str(name) for name in robot.get("expected_joint_names", [])]
        self._selected_group = str(moveit.get("selected_group", request.get("selected_group", "manipulator")))
        self._selected_link = str(
            moveit.get("selected_end_effector_link", request.get("selected_end_effector_link", "tool0"))
        )
        self._compute_ik_service = str(moveit.get("compute_ik_service", "/compute_ik"))
        self._move_group_node = str(moveit.get("move_group_node_name", "/move_group"))
        self._robot_description_topic = str(moveit.get("robot_description_topic", "/robot_description"))
        self._robot_description_semantic_topic = str(
            moveit.get("robot_description_semantic_topic", "/robot_description_semantic")
        )
        self._ik_timeout_sec = float(request.get("ik_timeout_sec", 2.0))
        self._ik_attempts = int(request.get("ik_attempts", 1))
        self._target_pose = request.get("diagnostic_target_pose", {})
        self._seed_joint_state = request.get("seed_joint_state", {})
        self._frame_id = str(request.get("frame_id", "base_link"))
        self._validation_timeout = float(validation.get("validation_timeout_sec", 45.0))
        self._startup_wait = float(validation.get("startup_wait_sec", 2.0))
        self._success_status = str(validation.get("status_success", "moveit_ik_diagnostic_validated"))

        self._execution_policy = {
            "trajectory_execution_allowed": bool(execution.get("trajectory_execution_allowed", False)),
            "controller_execution_allowed": bool(execution.get("controller_execution_allowed", False)),
            "follow_joint_trajectory_execution_allowed": bool(
                execution.get("follow_joint_trajectory_execution_allowed", False)
            ),
            "planning_execution_allowed": bool(execution.get("planning_execution_allowed", False)),
            "real_robot_allowed": bool(execution.get("real_robot_allowed", False)),
            "moveit_execution_allowed": bool(execution.get("moveit_execution_allowed", False)),
        }

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/moveit_ik_diagnostic_status")),
            10,
        )
        self._request_pub = self.create_publisher(
            String,
            str(validation.get("request_report_topic", "/proposal_simulation_cell/moveit_ik_request_report")),
            10,
        )
        self._response_pub = self.create_publisher(
            String,
            str(validation.get("response_report_topic", "/proposal_simulation_cell/moveit_ik_response_report")),
            10,
        )
        self._block_pub = self.create_publisher(
            String,
            str(validation.get("execution_block_report_topic", "/proposal_simulation_cell/moveit_execution_block_report")),
            10,
        )

        self.create_subscription(String, self._robot_description_topic, self._on_robot_description, 10)
        self.create_subscription(String, self._robot_description_semantic_topic, self._on_robot_description_semantic, 10)
        self._ik_client = self.create_client(GetPositionIK, self._compute_ik_service)
        self._parameter_client = self.create_client(GetParameters, f"{self._move_group_node}/get_parameters")

        self._start_time = time.monotonic()
        self._finished = False
        self._started = False
        self._compute_ik_called = False
        self._ik_request_sent = False
        self._ik_solution_found = False
        self._ik_error_code = 0
        self._returned_joint_names: list[str] = []
        self._returned_joint_positions: list[float] = []
        self._robot_description = ""
        self._robot_description_semantic = ""
        self._move_group_started = False
        self._compute_ik_available = False
        self._groups_report: dict[str, Any] = {}
        self._request_report: dict[str, Any] = {}
        self._response_report: dict[str, Any] = {}
        self._model_mismatch = False
        self._real_robot_endpoint_detected = False

        self.create_timer(0.2, self._tick)
        self.get_logger().info("proposal_simulation_cell_v2_2 MoveIt IK diagnostic node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v2.2 config not found: {path}")
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
            threading.Thread(target=self._run_diagnostic, daemon=True).start()
        if elapsed >= self._validation_timeout and not self._finished:
            self._write_outputs_once("compute_ik_service_unavailable")

    def _run_diagnostic(self) -> None:
        self._move_group_started = self._wait_for_move_group_parameters(timeout_sec=8.0)
        self._collect_robot_descriptions()
        self._groups_report = self._build_groups_report()
        self._model_mismatch = self._detect_model_mismatch()
        self._real_robot_endpoint_detected = self._detect_real_robot_endpoint()
        if not self._move_group_started:
            self._write_outputs_once("moveit_diagnostic_launch_failed")
            return
        if self._model_mismatch:
            self._write_outputs_once("moveit_model_mismatch_detected")
            return
        if not self._execution_blocked():
            self._write_outputs_once("moveit_diagnostic_launch_failed")
            return
        self._compute_ik_available = self._ik_client.wait_for_service(timeout_sec=8.0)
        if not self._compute_ik_available:
            self._write_outputs_once("compute_ik_service_unavailable")
            return
        self._call_compute_ik_once()
        if self._ik_solution_found:
            self._write_outputs_once(self._success_status)
        else:
            self._write_outputs_once("ik_solution_not_found")

    def _wait_for_move_group_parameters(self, timeout_sec: float) -> bool:
        return self._parameter_client.wait_for_service(timeout_sec=timeout_sec)

    def _collect_robot_descriptions(self) -> None:
        if self._robot_description and self._robot_description_semantic:
            return
        if not self._parameter_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = ["robot_description", "robot_description_semantic", "allow_trajectory_execution"]
        future = self._parameter_client.call_async(request)
        if not self._wait_for_future(future, timeout_sec=5.0):
            return
        response = future.result()
        if response is None:
            return
        if len(response.values) >= 2:
            if not self._robot_description:
                self._robot_description = response.values[0].string_value
            if not self._robot_description_semantic:
                self._robot_description_semantic = response.values[1].string_value

    def _build_groups_report(self) -> dict[str, Any]:
        report: dict[str, Any] = {
            "selected_group": self._selected_group,
            "selected_end_effector_link": self._selected_link,
            "available_groups": [],
            "group_links": [],
            "expected_joints": self._expected_joints,
            "semantic_parse_error": "",
            "robot_description_available": bool(self._robot_description),
            "robot_description_semantic_available": bool(self._robot_description_semantic),
            "model_choice": "consistent lbr_iisy3_r760 MoveIt model",
            "project_local_iisy6_candidate_used": False,
        }
        if not self._robot_description_semantic:
            return report
        try:
            root = ET.fromstring(self._robot_description_semantic)
        except ET.ParseError as exc:
            report["semantic_parse_error"] = str(exc)
            return report
        for group in root.findall("group"):
            name = group.attrib.get("name", "")
            if name:
                report["available_groups"].append(name)
            if name == self._selected_group:
                links = []
                for chain in group.findall("chain"):
                    links.extend([chain.attrib.get("base_link", ""), chain.attrib.get("tip_link", "")])
                for link in group.findall("link"):
                    links.append(link.attrib.get("name", ""))
                report["group_links"] = [link for link in links if link]
        return report

    def _detect_model_mismatch(self) -> bool:
        group_available = self._selected_group in self._groups_report.get("available_groups", [])
        link_available = self._selected_link in self._groups_report.get("group_links", [])
        joints_available = all(joint in self._robot_description for joint in self._expected_joints)
        return not (group_available and link_available and joints_available)

    def _detect_real_robot_endpoint(self) -> bool:
        lowered = self._robot_description.lower()
        real_endpoint_terms = ["fri", "ip_address", "port_id", "hardware"]
        mock_terms = ["mock_components/genericsystem", "mock"]
        if any(term in lowered for term in mock_terms):
            return False
        return any(term in lowered for term in real_endpoint_terms)

    def _execution_blocked(self) -> bool:
        return (
            not self._execution_policy["trajectory_execution_allowed"]
            and not self._execution_policy["controller_execution_allowed"]
            and not self._execution_policy["follow_joint_trajectory_execution_allowed"]
            and not self._execution_policy["planning_execution_allowed"]
            and not self._execution_policy["real_robot_allowed"]
            and not self._real_robot_endpoint_detected
        )

    def _call_compute_ik_once(self) -> None:
        service_request = GetPositionIK.Request()
        service_request.ik_request.group_name = self._selected_group
        service_request.ik_request.ik_link_name = self._selected_link
        service_request.ik_request.avoid_collisions = False
        service_request.ik_request.timeout = self._duration_from_seconds(self._ik_timeout_sec)
        service_request.ik_request.pose_stamped = self._pose_stamped()
        service_request.ik_request.robot_state = self._seed_robot_state()
        self._request_report = self._request_to_dict(service_request)
        self._ik_request_sent = True
        self._compute_ik_called = True
        future = self._ik_client.call_async(service_request)
        if not self._wait_for_future(future, timeout_sec=self._ik_timeout_sec + 5.0):
            self._ik_error_code = MoveItErrorCodes.TIMED_OUT
            self._response_report = {"service_response_received": False, "error": "timeout"}
            return
        response = future.result()
        if response is None:
            self._ik_error_code = MoveItErrorCodes.FAILURE
            self._response_report = {"service_response_received": False, "error": "empty_response"}
            return
        self._ik_error_code = int(response.error_code.val)
        self._ik_solution_found = self._ik_error_code == MoveItErrorCodes.SUCCESS
        self._returned_joint_names = list(response.solution.joint_state.name)
        self._returned_joint_positions = [float(position) for position in response.solution.joint_state.position]
        self._response_report = {
            "service_response_received": True,
            "ik_solution_found": self._ik_solution_found,
            "ik_error_code": self._ik_error_code,
            "returned_joint_names": self._returned_joint_names,
            "returned_joint_positions": self._returned_joint_positions,
        }

    def _pose_stamped(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        position = self._target_pose.get("position", {})
        orientation = self._target_pose.get("orientation", {})
        pose.pose.position.x = float(position.get("x", 0.36))
        pose.pose.position.y = float(position.get("y", 0.0))
        pose.pose.position.z = float(position.get("z", 0.42))
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

    def _duration_from_seconds(self, seconds: float) -> Duration:
        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
        return duration

    def _wait_for_future(self, future: Any, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.05)
        return future.done()

    def _status_payload(self, status: str | None = None) -> dict[str, Any]:
        resolved_status = status or self._current_status()
        return {
            "robot_model": self._robot_model,
            "moveit_used": True,
            "diagnostic_only": True,
            "robot_description_available": bool(self._robot_description),
            "robot_description_semantic_available": bool(self._robot_description_semantic),
            "move_group_started": self._move_group_started,
            "compute_ik_service_available": self._compute_ik_available,
            "compute_ik_called": self._compute_ik_called,
            "ik_request_sent": self._ik_request_sent,
            "ik_solution_found": self._ik_solution_found,
            "ik_error_code": self._ik_error_code,
            "selected_group": self._selected_group,
            "selected_end_effector_link": self._selected_link,
            "returned_joint_count": len(self._returned_joint_names),
            "trajectory_execution_allowed": False,
            "controller_execution_allowed": False,
            "follow_joint_trajectory_execution_allowed": False,
            "planning_execution_allowed": False,
            "real_robot_used": False,
            "motion_executed": False,
            "trajectory_sent": False,
            "status": resolved_status,
        }

    def _current_status(self) -> str:
        if self._model_mismatch:
            return "moveit_model_mismatch_detected"
        if not self._move_group_started and self._started:
            return "moveit_diagnostic_launch_failed"
        if not self._compute_ik_available and self._started:
            return "compute_ik_service_unavailable"
        if self._compute_ik_called and not self._ik_solution_found:
            return "ik_solution_not_found"
        if self._ik_solution_found:
            return self._success_status
        return "moveit_ik_diagnostic_pending"

    def _publish_reports(self) -> None:
        self._publish_json(self._status_pub, self._status_payload())
        self._publish_json(self._request_pub, self._request_report)
        self._publish_json(self._response_pub, self._response_report)
        self._publish_json(self._block_pub, self._execution_block_report())

    def _write_outputs_once(self, status: str) -> None:
        if self._finished:
            return
        self._finished = True
        self._compute_ik_available = self._compute_ik_available or self._service_name_available(self._compute_ik_service)
        payload = self._status_payload(status)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_parameters_file()
        self._write_lines(self._output_dir / "robot_description_status.txt", self._robot_description_status_lines())
        self._write_lines(
            self._output_dir / "robot_description_semantic_status.txt",
            self._robot_description_semantic_status_lines(),
        )
        self._write_lines(self._output_dir / "moveit_groups_report.txt", self._report_lines(self._groups_report))
        self._write_lines(self._output_dir / "compute_ik_service_status.txt", self._compute_ik_status_lines())
        self._write_yaml(self._output_dir / "ik_request.yaml", self._request_report)
        self._write_yaml(self._output_dir / "ik_response.yaml", self._response_report)
        self._write_json(self._output_dir / "moveit_ik_diagnostic_status.json", payload)
        self._write_execution_block_csv()
        self._write_summary(payload)
        self._write_run_log(payload)
        self._publish_reports()
        self.get_logger().info("proposal_simulation_cell_v2_2 MoveIt IK diagnostics written")
        rclpy.shutdown()

    def _write_parameters_file(self) -> None:
        lines = [
            f"move_group_started={self._move_group_started}",
            "allow_trajectory_execution=false",
            "moveit_manage_controllers=false",
            "trajectory_execution_allowed=false",
            "planning_execution_allowed=false",
        ]
        lines.extend(self._run_command(["ros2", "param", "list", self._move_group_node], timeout=2.0))
        self._write_lines(self._output_dir / "parameters.txt", lines)

    def _robot_description_status_lines(self) -> list[str]:
        return [
            f"robot_description_available={str(bool(self._robot_description)).lower()}",
            f"robot_model={self._robot_model}",
            f"expected_joint_names_found={str(all(j in self._robot_description for j in self._expected_joints)).lower()}",
            f"real_robot_endpoint_detected={str(self._real_robot_endpoint_detected).lower()}",
            "description_source=kuka_lbr_iisy_support lbr_iisy3_r760 mode mock",
        ]

    def _robot_description_semantic_status_lines(self) -> list[str]:
        return [
            f"robot_description_semantic_available={str(bool(self._robot_description_semantic)).lower()}",
            f"selected_group={self._selected_group}",
            f"selected_end_effector_link={self._selected_link}",
            f"group_available={str(self._selected_group in self._groups_report.get('available_groups', [])).lower()}",
            f"tool_link_available={str(self._selected_link in self._groups_report.get('group_links', [])).lower()}",
            "project_local_iisy6_candidate_used=false",
        ]

    def _compute_ik_status_lines(self) -> list[str]:
        return [
            f"compute_ik_service={self._compute_ik_service}",
            f"compute_ik_service_available={str(self._compute_ik_available).lower()}",
            f"compute_ik_called={str(self._compute_ik_called).lower()}",
            "compute_ik_call_count=1" if self._compute_ik_called else "compute_ik_call_count=0",
            "diagnostic_only=true",
        ]

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

    def _write_execution_block_csv(self) -> None:
        rows = [
            {"check": key, "value": str(value).lower()}
            for key, value in self._execution_block_report().items()
        ]
        self._write_csv(self._output_dir / "moveit_execution_block_report.csv", rows)

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation",
            "",
            f"Status: `{status['status']}`",
            "",
            "This diagnostic validates MoveIt IK availability without executing plans, trajectories, controllers, or real robot endpoints.",
            "",
            f"- move_group_started: {str(status['move_group_started']).lower()}",
            f"- compute_ik_service_available: {str(status['compute_ik_service_available']).lower()}",
            f"- compute_ik_called: {str(status['compute_ik_called']).lower()}",
            f"- ik_solution_found: {str(status['ik_solution_found']).lower()}",
            f"- ik_error_code: {status['ik_error_code']}",
            "- trajectory_execution_allowed: false",
            "- controller_execution_allowed: false",
            "- follow_joint_trajectory_execution_allowed: false",
            "- planning_execution_allowed: false",
            "- real_robot_used: false",
            "- motion_executed: false",
            "- trajectory_sent: false",
        ]
        self._write_lines(self._output_dir / "summary.md", lines)

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation",
            f"status={status['status']}",
            f"move_group_started={str(status['move_group_started']).lower()}",
            f"compute_ik_service_available={str(status['compute_ik_service_available']).lower()}",
            f"compute_ik_called={str(status['compute_ik_called']).lower()}",
            f"ik_solution_found={str(status['ik_solution_found']).lower()}",
            "trajectory_execution_allowed=false",
            "controller_execution_allowed=false",
            "follow_joint_trajectory_execution_allowed=false",
            "real_robot_used=false",
        ]
        self._write_lines(self._output_dir / "run.log", lines)

    def _request_to_dict(self, request: GetPositionIK.Request) -> dict[str, Any]:
        pose = request.ik_request.pose_stamped.pose
        return {
            "group_name": request.ik_request.group_name,
            "ik_link_name": request.ik_request.ik_link_name,
            "frame_id": request.ik_request.pose_stamped.header.frame_id,
            "avoid_collisions": request.ik_request.avoid_collisions,
            "timeout_sec": self._ik_timeout_sec,
            "configured_attempts": self._ik_attempts,
            "seed_joint_names": list(request.ik_request.robot_state.joint_state.name),
            "seed_joint_positions": [float(value) for value in request.ik_request.robot_state.joint_state.position],
            "target_pose": {
                "position": {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                },
                "orientation": {
                    "x": float(pose.orientation.x),
                    "y": float(pose.orientation.y),
                    "z": float(pose.orientation.z),
                    "w": float(pose.orientation.w),
                },
            },
            "diagnostic_only": True,
        }

    def _service_name_available(self, name: str) -> bool:
        return name in {service_name for service_name, _types in self.get_service_names_and_types()}

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

    def _report_lines(self, report: dict[str, Any]) -> list[str]:
        return json.dumps(report, indent=2, sort_keys=True).splitlines()

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


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV22MoveItIkDiagnosticNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
