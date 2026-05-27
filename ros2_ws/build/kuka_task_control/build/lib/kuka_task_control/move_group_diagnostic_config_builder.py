"""Build diagnostic-only move_group launch configuration metadata."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.node import Node
from std_msgs.msg import String

from kuka_task_control.diagnostic_robot_description import (
    REQUIRED_JOINTS,
    robot_description_content_report,
    robot_description_file_fallback,
)


class MoveGroupDiagnosticConfigBuilder(Node):
    """Publish the no-motion parameters expected by a diagnostic move_group."""

    TOPIC = "/move_group_diagnostic_config"
    ROBOT_DESCRIPTION_LIST_SERVICE_NAMES = (
        "/robot_state_publisher/list_parameters",
        "/move_group/list_parameters",
    )

    def __init__(self) -> None:
        super().__init__("move_group_diagnostic_config_builder")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("moveit_config_directory", "")
        self.declare_parameter("semantic_srdf_file_path", "")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("tool_link", "tool0")
        self.declare_parameter("move_group_launched", False)
        self.declare_parameter("robot_description_semantic", "")

        self._robot_description_semantic = str(
            self.get_parameter("robot_description_semantic").value or ""
        )
        self._robot_description = ""
        self._robot_description_source = "unavailable"
        self._robot_description_source_node: str | None = None
        self._robot_description_error: str | None = None
        self._parameter_list_clients: dict[str, Any] = {}
        self._parameter_list_futures: dict[str, Any] = {}
        self._parameter_get_clients: dict[str, Any] = {}
        self._parameter_get_futures: dict[str, Any] = {}
        self._parameter_get_names: dict[str, list[str]] = {}
        self._parameter_request_attempts: dict[str, int] = {}

        self._publisher = self.create_publisher(String, self.TOPIC, 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_config,
        )
        self.get_logger().info(
            "move_group diagnostic config builder started with motion disabled."
        )

    def _publish_config(self) -> None:
        self._request_description_parameters_if_visible()
        self._collect_description_parameter_results()
        self._apply_robot_description_file_fallback_if_needed()

        config_dir = self._moveit_config_directory()
        srdf_path = self._semantic_srdf_path(config_dir)
        semantic_text = self._read_text(srdf_path)
        if not self._robot_description_semantic and semantic_text:
            self._robot_description_semantic = semantic_text

        kinematics_yaml = config_dir / "kinematics.yaml"
        ompl_planning_yaml = config_dir / "ompl_planning.yaml"
        move_group_launched = self._as_bool(
            self.get_parameter("move_group_launched").value
        ) or self._move_group_node_detected()
        description_report = robot_description_content_report(
            self._robot_description,
            REQUIRED_JOINTS,
        )
        robot_description_error = (
            self._robot_description_error
            or description_report["robot_description_parse_error"]
        )

        payload: dict[str, Any] = {
            "status": "move_group_diagnostic_config_prepared",
            "robot_description_available": description_report[
                "robot_description_available"
            ],
            "robot_description_source": self._robot_description_source,
            "robot_description_source_node": self._robot_description_source_node,
            "robot_description_length": description_report[
                "robot_description_length"
            ],
            "robot_description_contains_required_joints": description_report[
                "robot_description_contains_required_joints"
            ],
            "robot_description_contains_tool0": description_report[
                "robot_description_contains_tool0"
            ],
            "robot_description_error": robot_description_error,
            "robot_description_semantic_available": bool(
                self._robot_description_semantic
            ),
            "robot_description_semantic_length": len(
                self._robot_description_semantic
            ),
            "kinematics_yaml_found": kinematics_yaml.is_file(),
            "kinematics_yaml_file": str(kinematics_yaml),
            "ompl_planning_yaml_found": ompl_planning_yaml.is_file(),
            "ompl_planning_yaml_file": str(ompl_planning_yaml),
            "planning_group": str(self.get_parameter("planning_group").value),
            "planning_frame": str(self.get_parameter("planning_frame").value),
            "tool_link": str(self.get_parameter("tool_link").value),
            "trajectory_execution_allowed": False,
            "controller_motion_allowed": False,
            "allow_trajectory_execution": False,
            "move_group_launch_allowed": False,
            "compute_ik_service_expected": move_group_launched,
            "approved_for_motion": False,
            "diagnostic_only": True,
            "recommended_next_step": (
                "launch_move_group_diagnostic_only_with_execution_disabled"
            ),
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _request_description_parameters_if_visible(self) -> None:
        if (
            self._robot_description
            and self._robot_description_source != "file_fallback"
            and self._robot_description_semantic
        ):
            return

        visible_services = {
            name
            for name, _types in self.get_service_names_and_types()
            if name.endswith("/list_parameters")
        }
        likely_services = list(self.ROBOT_DESCRIPTION_LIST_SERVICE_NAMES)
        likely_services.extend(
            service_name
            for service_name in visible_services
            if "robot_state_publisher" in service_name or "move_group" in service_name
        )
        likely_services.extend(
            service_name
            for service_name in visible_services
            if service_name not in likely_services
        )

        for service_name in sorted(set(likely_services)):
            if service_name in self._parameter_list_futures:
                continue
            attempts = self._parameter_request_attempts.get(service_name, 0)
            if attempts >= 5:
                continue
            client = self._parameter_list_clients.get(service_name)
            if client is None:
                client = self.create_client(ListParameters, service_name)
                self._parameter_list_clients[service_name] = client
            if not client.service_is_ready():
                self._parameter_request_attempts[service_name] = attempts + 1
                continue
            request = ListParameters.Request()
            request.prefixes = []
            request.depth = 0
            self._parameter_list_futures[service_name] = client.call_async(request)
            self._parameter_request_attempts[service_name] = attempts + 1

    def _collect_description_parameter_results(self) -> None:
        completed_lists = []
        for service_name, future in self._parameter_list_futures.items():
            if not future.done():
                continue
            completed_lists.append(service_name)
            try:
                response = future.result()
            except Exception as exc:  # pragma: no cover - defensive ROS callback path
                self.get_logger().debug(
                    f"Failed to list parameters from {service_name}: {exc}"
                )
                continue
            parameter_names = set(response.result.names)
            requested_names = []
            if "robot_description" in parameter_names:
                requested_names.append("robot_description")
            if "robot_description_semantic" in parameter_names:
                requested_names.append("robot_description_semantic")
            if requested_names:
                self._request_listed_description_parameters(
                    service_name,
                    requested_names,
                )

        for service_name in completed_lists:
            self._parameter_list_futures.pop(service_name, None)

        completed_gets = []
        for service_name, future in self._parameter_get_futures.items():
            if not future.done():
                continue
            completed_gets.append(service_name)
            try:
                response = future.result()
            except Exception as exc:  # pragma: no cover - defensive ROS callback path
                self._robot_description_error = (
                    f"Failed to read description parameters from {service_name}: {exc}"
                )
                self.get_logger().debug(self._robot_description_error)
                continue
            requested_names = self._parameter_get_names.get(service_name, [])
            values = list(response.values)
            for name, value in zip(requested_names, values):
                if name == "robot_description" and value.string_value:
                    self._robot_description = value.string_value
                    self._robot_description_source = "parameter_service"
                    self._robot_description_source_node = service_name.rsplit(
                        "/get_parameters",
                        1,
                    )[0]
                    self._robot_description_error = None
                if name == "robot_description_semantic" and value.string_value:
                    self._robot_description_semantic = value.string_value

        for service_name in completed_gets:
            self._parameter_get_futures.pop(service_name, None)
            self._parameter_get_names.pop(service_name, None)

    def _request_listed_description_parameters(
        self,
        list_service_name: str,
        requested_names: list[str],
    ) -> None:
        get_service_name = list_service_name.rsplit("/list_parameters", 1)[0]
        get_service_name += "/get_parameters"
        if get_service_name in self._parameter_get_futures:
            return
        client = self._parameter_get_clients.get(get_service_name)
        if client is None:
            client = self.create_client(GetParameters, get_service_name)
            self._parameter_get_clients[get_service_name] = client
        if not client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = requested_names
        self._parameter_get_futures[get_service_name] = client.call_async(request)
        self._parameter_get_names[get_service_name] = requested_names

    def _apply_robot_description_file_fallback_if_needed(self) -> None:
        if self._robot_description:
            return
        robot_description, error = robot_description_file_fallback()
        if robot_description:
            self._robot_description = robot_description
            self._robot_description_source = "file_fallback"
            self._robot_description_source_node = None
            self._robot_description_error = None
            return
        self._robot_description_source = "unavailable"
        self._robot_description_source_node = None
        self._robot_description_error = error

    def _moveit_config_directory(self) -> Path:
        configured = str(self.get_parameter("moveit_config_directory").value or "")
        if configured.strip():
            return Path(configured).expanduser()
        source_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "moveit_lbr_iisy6_r1300"
        )
        if source_path.is_dir():
            return source_path
        try:
            share_path = Path(get_package_share_directory("kuka_task_control"))
        except PackageNotFoundError:
            return source_path
        return share_path / "config" / "moveit_lbr_iisy6_r1300"

    def _semantic_srdf_path(self, config_dir: Path) -> Path:
        configured = str(self.get_parameter("semantic_srdf_file_path").value or "")
        if configured.strip():
            return Path(configured).expanduser()
        return config_dir / "lbr_iisy6_r1300.srdf"

    @staticmethod
    def _read_text(path: Path) -> str:
        try:
            return path.read_text(encoding="utf-8") if path.is_file() else ""
        except (OSError, UnicodeDecodeError):
            return ""

    def _move_group_node_detected(self) -> bool:
        return any(
            name.rsplit("/", 1)[-1] == "move_group"
            for name in self.get_node_names()
        )

    @staticmethod
    def _as_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MoveGroupDiagnosticConfigBuilder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
