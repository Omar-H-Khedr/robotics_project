"""Prepare diagnostic-only MoveIt launch inputs without launching move_group."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.node import Node
from std_msgs.msg import String

from kuka_task_control.diagnostic_robot_description import (
    REQUIRED_JOINTS as DIAGNOSTIC_REQUIRED_JOINTS,
    robot_description_content_report,
    robot_description_file_fallback,
)

try:
    import yaml
except ImportError:  # pragma: no cover - ROS environments normally provide PyYAML
    yaml = None


class MoveItDiagnosticInputBuilder(Node):
    """Publish a no-motion bundle of inputs for a future diagnostic move_group."""

    TOPIC = "/moveit_diagnostic_inputs"
    REQUIRED_JOINTS = DIAGNOSTIC_REQUIRED_JOINTS
    ROBOT_DESCRIPTION_LIST_SERVICE_NAMES = (
        "/robot_state_publisher/list_parameters",
        "/move_group/list_parameters",
    )

    def __init__(self) -> None:
        super().__init__("moveit_diagnostic_input_builder")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("semantic_srdf_file_path", "")
        self.declare_parameter("moveit_config_directory", "")
        self.declare_parameter("robot_description", "")

        self._robot_description_xml: str | None = (
            str(self.get_parameter("robot_description").value or "") or None
        )
        self._robot_description_source_node: str | None = (
            "/moveit_diagnostic_input_builder parameter"
            if self._robot_description_xml
            else None
        )
        self._robot_description_source = (
            "local_parameter" if self._robot_description_xml else "unavailable"
        )
        self._robot_description_error: str | None = None
        self._parameter_list_clients: dict[str, Any] = {}
        self._parameter_list_futures: dict[str, Any] = {}
        self._parameter_get_clients: dict[str, Any] = {}
        self._parameter_get_futures: dict[str, Any] = {}
        self._parameter_request_attempts: dict[str, int] = {}
        self._tool_link_validation_report: dict[str, Any] | None = None
        self._tool_link_validation_parse_error = False

        self._publisher = self.create_publisher(String, self.TOPIC, 10)
        self.create_subscription(
            String,
            "/tool_link_validation",
            self._on_tool_link_validation,
            10,
        )
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_inputs,
        )
        self.get_logger().info(
            "MoveIt diagnostic input builder started in no-motion mode."
        )

    def _publish_inputs(self) -> None:
        self._request_robot_description_if_visible()
        self._collect_robot_description_results()
        self._apply_robot_description_file_fallback_if_needed()

        srdf_path = self._semantic_srdf_path()
        config_dir = self._moveit_config_directory()
        robot_description = self._robot_description_report()
        semantic = self._semantic_report(srdf_path)
        config = self._moveit_config_report(config_dir)
        tool_status = self._tool_link_validation_status()
        tool_valid_for_diagnostics = (
            tool_status == "tool_link_candidate_valid_but_not_motion_approved"
        )
        moveit_diagnostic_inputs_ready = bool(
            robot_description["robot_description_available"]
            and robot_description["robot_description_contains_required_joints"]
            and robot_description["robot_description_contains_tool0"]
            and semantic["robot_description_semantic_available"]
            and semantic["semantic_srdf_parse_success"]
            and semantic["arm_group_valid"]
            and config["kinematics_group_config_found"]
            and tool_valid_for_diagnostics
        )

        payload: dict[str, Any] = {
            "status": "moveit_diagnostic_inputs_diagnostic_only_no_motion",
            "robot_description_available": robot_description[
                "robot_description_available"
            ],
            "robot_description_source_node": self._robot_description_source_node,
            "robot_description_source": self._robot_description_source,
            "robot_description_length": robot_description["robot_description_length"],
            "robot_description_contains_required_joints": robot_description[
                "robot_description_contains_required_joints"
            ],
            "robot_description_contains_tool0": robot_description[
                "robot_description_contains_tool0"
            ],
            "robot_description_error": robot_description["robot_description_error"],
            "semantic_srdf_file_path": str(srdf_path),
            "semantic_srdf_file_exists": semantic["semantic_srdf_file_exists"],
            "semantic_srdf_parse_success": semantic["semantic_srdf_parse_success"],
            "robot_description_semantic_available": semantic[
                "robot_description_semantic_available"
            ],
            "robot_description_semantic_length": semantic[
                "robot_description_semantic_length"
            ],
            "semantic_group_name": "arm",
            "semantic_group_joints": semantic["semantic_group_joints"],
            "semantic_group_valid": semantic["arm_group_valid"],
            "required_joints": list(self.REQUIRED_JOINTS),
            "missing_semantic_group_joints": semantic["missing_required_joints"],
            "kinematics_yaml_file": config["kinematics_yaml_file"],
            "kinematics_yaml_found": config["kinematics_yaml_found"],
            "kinematics_group_config_found": config[
                "kinematics_group_config_found"
            ],
            "ompl_planning_yaml_file": config["ompl_planning_yaml_file"],
            "ompl_planning_yaml_found": config["ompl_planning_yaml_found"],
            "joint_limits_yaml_file": config["joint_limits_yaml_file"],
            "joint_limits_yaml_found": config["joint_limits_yaml_found"],
            "joint_limits_fallback_source": config["joint_limits_fallback_source"],
            "planning_frame": "base_link",
            "tool_link": "tool0",
            "tool_link_validation_available": (
                self._tool_link_validation_report is not None
            ),
            "tool_link_validation_parse_error": (
                self._tool_link_validation_parse_error
            ),
            "tool_link_validation_status": tool_status,
            "selected_tool_axis_candidate": self._tool_link_field(
                "selected_tool_axis_candidate",
                "tool0_+Z",
            ),
            "trajectory_execution_allowed": False,
            "controller_motion_allowed": False,
            "move_group_launch_allowed": False,
            "compute_ik_test_allowed": False,
            "moveit_diagnostic_inputs_ready": moveit_diagnostic_inputs_ready,
            "move_group_launch_inputs_ready": moveit_diagnostic_inputs_ready,
            "approved_for_motion": False,
            "recommended_next_step": self._recommended_next_step(
                robot_description=robot_description,
                semantic=semantic,
                config=config,
                tool_valid_for_diagnostics=tool_valid_for_diagnostics,
                ready=moveit_diagnostic_inputs_ready,
            ),
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _on_tool_link_validation(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self._tool_link_validation_parse_error = True
            return
        if not isinstance(payload, dict):
            self._tool_link_validation_parse_error = True
            return
        self._tool_link_validation_parse_error = False
        self._tool_link_validation_report = payload

    def _request_robot_description_if_visible(self) -> None:
        if (
            self._robot_description_xml
            and self._robot_description_source != "file_fallback"
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

    def _collect_robot_description_results(self) -> None:
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
            if "robot_description" in set(response.result.names):
                self._request_listed_robot_description(service_name)

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
                    f"Failed to read robot_description from {service_name}: {exc}"
                )
                self.get_logger().debug(self._robot_description_error)
                continue
            if response.values and response.values[0].string_value:
                self._robot_description_xml = response.values[0].string_value
                self._robot_description_source = "parameter_service"
                self._robot_description_source_node = service_name.rsplit(
                    "/get_parameters",
                    1,
                )[0]
                self._robot_description_error = None

        for service_name in completed_gets:
            self._parameter_get_futures.pop(service_name, None)

    def _request_listed_robot_description(self, list_service_name: str) -> None:
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
        request.names = ["robot_description"]
        self._parameter_get_futures[get_service_name] = client.call_async(request)

    def _apply_robot_description_file_fallback_if_needed(self) -> None:
        if self._robot_description_xml:
            return
        robot_description, error = robot_description_file_fallback()
        if robot_description:
            self._robot_description_xml = robot_description
            self._robot_description_source = "file_fallback"
            self._robot_description_source_node = None
            self._robot_description_error = None
            return
        self._robot_description_source = "unavailable"
        self._robot_description_source_node = None
        self._robot_description_error = error

    def _robot_description_report(self) -> dict[str, Any]:
        robot_description = self._robot_description_xml or ""
        report = robot_description_content_report(
            robot_description,
            self.REQUIRED_JOINTS,
        )
        return {
            "robot_description_available": report["robot_description_available"],
            "robot_description_length": report["robot_description_length"],
            "robot_description_contains_required_joints": report[
                "robot_description_contains_required_joints"
            ],
            "robot_description_contains_tool0": report[
                "robot_description_contains_tool0"
            ],
            "robot_description_error": (
                self._robot_description_error
                or report["robot_description_parse_error"]
            ),
        }

    def _semantic_report(self, srdf_path: Path) -> dict[str, Any]:
        semantic_text = ""
        parse_success = False
        arm_group_joints: list[str] = []
        if srdf_path.is_file():
            try:
                semantic_text = srdf_path.read_text(encoding="utf-8")
                root = ElementTree.fromstring(semantic_text)
                parse_success = True
            except (ElementTree.ParseError, OSError, UnicodeDecodeError):
                root = None
            if root is not None:
                for group in root.findall("group"):
                    if group.attrib.get("name") != "arm":
                        continue
                    arm_group_joints = [
                        joint.attrib["name"]
                        for joint in group.findall("joint")
                        if joint.attrib.get("name")
                    ]
                    break

        missing_required_joints = [
            joint for joint in self.REQUIRED_JOINTS if joint not in set(arm_group_joints)
        ]
        return {
            "semantic_srdf_file_exists": srdf_path.is_file(),
            "semantic_srdf_parse_success": parse_success,
            "robot_description_semantic_available": bool(semantic_text and parse_success),
            "robot_description_semantic_length": len(semantic_text),
            "semantic_group_joints": arm_group_joints,
            "arm_group_valid": not missing_required_joints,
            "missing_required_joints": missing_required_joints,
        }

    def _moveit_config_report(self, config_dir: Path) -> dict[str, Any]:
        kinematics_yaml = config_dir / "kinematics.yaml"
        ompl_planning_yaml = config_dir / "ompl_planning.yaml"
        joint_limits_yaml = config_dir / "joint_limits.yaml"
        kinematics_data = self._yaml_mapping(kinematics_yaml)
        arm_config = kinematics_data.get("arm") if isinstance(kinematics_data, dict) else None
        joint_limits_found = joint_limits_yaml.is_file()
        return {
            "kinematics_yaml_file": str(kinematics_yaml),
            "kinematics_yaml_found": kinematics_yaml.is_file(),
            "kinematics_group_config_found": isinstance(arm_config, dict)
            and bool(arm_config),
            "ompl_planning_yaml_file": str(ompl_planning_yaml),
            "ompl_planning_yaml_found": ompl_planning_yaml.is_file(),
            "joint_limits_yaml_file": str(joint_limits_yaml),
            "joint_limits_yaml_found": joint_limits_found,
            "joint_limits_fallback_source": (
                None
                if joint_limits_found
                else "robot_description_joint_limits_required_at_move_group_launch"
            ),
        }

    @staticmethod
    def _yaml_mapping(path: Path) -> dict[str, Any]:
        if not path.is_file():
            return {}
        if yaml is None:
            return {}
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            return {}
        return data if isinstance(data, dict) else {}

    def _semantic_srdf_path(self) -> Path:
        configured = str(self.get_parameter("semantic_srdf_file_path").value or "")
        if configured.strip():
            return Path(configured).expanduser()
        return self._default_config_directory() / "lbr_iisy6_r1300.srdf"

    def _moveit_config_directory(self) -> Path:
        configured = str(self.get_parameter("moveit_config_directory").value or "")
        if configured.strip():
            return Path(configured).expanduser()
        return self._default_config_directory()

    def _default_config_directory(self) -> Path:
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

    def _tool_link_validation_status(self) -> str:
        if self._tool_link_validation_report is None:
            return "not_observed"
        status = self._tool_link_validation_report.get("tool_link_validation_status")
        return str(status) if status else "tool_link_candidate_incomplete"

    def _tool_link_field(self, key: str, default: Any = None) -> Any:
        if self._tool_link_validation_report is None:
            return default
        return self._tool_link_validation_report.get(key, default)

    @staticmethod
    def _recommended_next_step(
        *,
        robot_description: dict[str, Any],
        semantic: dict[str, Any],
        config: dict[str, Any],
        tool_valid_for_diagnostics: bool,
        ready: bool,
    ) -> str:
        if ready:
            return "create_move_group_diagnostic_launch_with_trajectory_execution_disabled"
        if not robot_description["robot_description_available"]:
            return "missing_robot_description"
        if not robot_description["robot_description_contains_required_joints"]:
            return "robot_description_missing_required_joints"
        if not robot_description["robot_description_contains_tool0"]:
            return "robot_description_missing_tool0"
        if not semantic["semantic_srdf_file_exists"]:
            return "missing_semantic_srdf_file"
        if not semantic["semantic_srdf_parse_success"]:
            return "semantic_srdf_parse_failed"
        if not semantic["robot_description_semantic_available"]:
            return "robot_description_semantic_unavailable"
        if not semantic["arm_group_valid"]:
            return "semantic_arm_group_missing_required_joints"
        if not config["kinematics_yaml_found"]:
            return "missing_kinematics_yaml"
        if not config["kinematics_group_config_found"]:
            return "kinematics_yaml_missing_arm_group_config"
        if not tool_valid_for_diagnostics:
            return "missing_valid_tool_link_diagnostic_validation"
        return "complete_missing_moveit_diagnostic_input"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MoveItDiagnosticInputBuilder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
