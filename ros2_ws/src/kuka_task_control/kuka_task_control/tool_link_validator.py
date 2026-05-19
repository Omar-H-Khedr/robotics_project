"""Validate the diagnostic tool link candidate without enabling robot motion."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class ToolLinkValidator(Node):
    """Publish diagnostic-only validation for the intended MoveIt tool link."""

    TOPIC = "/tool_link_validation"
    REQUIRED_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
    ROBOT_DESCRIPTION_SERVICE_NAMES = (
        "/robot_state_publisher/get_parameters",
        "/tool_link_validator/get_parameters",
    )

    def __init__(self) -> None:
        super().__init__("tool_link_validator")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_link_candidate", "tool0")
        self.declare_parameter("selected_tool_axis_candidate", "tool0_+Z")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("srdf_path", "")

        self._world_frame = str(self.get_parameter("world_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tool_link_candidate = str(
            self.get_parameter("tool_link_candidate").value
        )
        self._selected_tool_axis_candidate = str(
            self.get_parameter("selected_tool_axis_candidate").value
        )
        self._robot_description_xml: str | None = (
            str(self.get_parameter("robot_description").value or "") or None
        )
        self._robot_description_source: str | None = (
            "/tool_link_validator parameter" if self._robot_description_xml else None
        )
        self._robot_description_check_reason = (
            "robot_description parameter forwarded to tool_link_validator"
            if self._robot_description_xml
            else "robot_description not checked yet"
        )
        self._tool_axis_payload: dict[str, Any] | None = None
        self._orientation_targets_payload: dict[str, Any] | None = None
        self._parameter_clients: dict[str, Any] = {}
        self._parameter_futures: dict[str, Any] = {}
        self._parameter_request_attempts: dict[str, int] = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(String, self.TOPIC, 10)
        self.create_subscription(String, "/tool_axis_audit", self._on_tool_axis, 10)
        self.create_subscription(
            String,
            "/cartesian_orientation_targets",
            self._on_orientation_targets,
            10,
        )
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_validation,
        )
        self.get_logger().info(
            "Tool link validator started in diagnostic-only no-motion mode."
        )

    def _publish_validation(self) -> None:
        self._request_robot_description_if_visible()
        self._collect_robot_description_results()

        world_to_tool = self._lookup_transform(
            self._world_frame,
            self._tool_link_candidate,
        )
        base_to_tool = self._lookup_transform(
            self._base_frame,
            self._tool_link_candidate,
        )
        world_to_base = self._lookup_transform(self._world_frame, self._base_frame)
        urdf_report = self._urdf_report()
        srdf_report = self._srdf_report(self._srdf_path())
        tool_axis_candidate_available = self._tool_axis_candidate_available()
        orientation_targets_available = self._orientation_targets_available()

        candidate_valid = bool(
            urdf_report["robot_description_available"]
            and urdf_report["tool_link_exists_in_urdf"]
            and world_to_tool is not None
            and base_to_tool is not None
            and world_to_base is not None
            and srdf_report["arm_group_found"]
            and srdf_report["required_joints_present"]
        )

        payload: dict[str, Any] = {
            "status": "tool_link_validation_diagnostic_only_no_motion",
            "tool_link_candidate": self._tool_link_candidate,
            "tool_link_exists_in_urdf": urdf_report["tool_link_exists_in_urdf"],
            "robot_description_available": urdf_report["robot_description_available"],
            "robot_description_source": self._robot_description_source,
            "robot_description_check_reason": self._robot_description_check_reason,
            "urdf_parse_success": urdf_report["urdf_parse_success"],
            "urdf_links": urdf_report["urdf_links"],
            "tf_world_to_tool_available": world_to_tool is not None,
            "tf_base_to_tool_available": base_to_tool is not None,
            "tf_world_to_base_available": world_to_base is not None,
            "current_tool_pose_world": self._pose_from_transform(world_to_tool),
            "current_tool_pose_base": self._pose_from_transform(base_to_tool),
            "srdf_file_path": str(self._srdf_path()),
            "srdf_file_exists": srdf_report["file_exists"],
            "srdf_parse_success": srdf_report["parse_success"],
            "arm_group_found": srdf_report["arm_group_found"],
            "arm_group_joints": srdf_report["arm_group_joints"],
            "required_joints": list(self.REQUIRED_JOINTS),
            "required_joints_present": srdf_report["required_joints_present"],
            "missing_required_joints": srdf_report["missing_required_joints"],
            "selected_tool_axis_candidate": self._selected_tool_axis_candidate,
            "expected_aligned_insertion_axis": [0.0, 0.0, -1.0],
            "tool_axis_candidate_available": tool_axis_candidate_available,
            "orientation_targets_available": orientation_targets_available,
            "tool_link_validation_status": (
                "tool_link_candidate_valid_but_not_motion_approved"
                if candidate_valid
                else "tool_link_candidate_incomplete"
            ),
            "approved_for_motion": False,
            "controller_motion_allowed": False,
            "trajectory_execution_allowed": False,
            "motion_execution_enabled": False,
            "trajectory_execution_requested": False,
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _on_tool_axis(self, message: String) -> None:
        self._tool_axis_payload = self._parse_json(message.data)

    def _on_orientation_targets(self, message: String) -> None:
        self._orientation_targets_payload = self._parse_json(message.data)

    def _lookup_transform(self, target_frame: str, source_frame: str) -> Any | None:
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as exc:
            self.get_logger().debug(
                f"TF lookup unavailable for {target_frame} -> {source_frame}: {exc}"
            )
            return None

    def _request_robot_description_if_visible(self) -> None:
        if self._robot_description_xml:
            return

        visible_services = {
            name
            for name, _types in self.get_service_names_and_types()
            if name.endswith("/get_parameters")
        }
        likely_services = list(self.ROBOT_DESCRIPTION_SERVICE_NAMES)
        likely_services.extend(
            service_name
            for service_name in visible_services
            if "robot_state_publisher" in service_name
        )

        for service_name in sorted(set(likely_services)):
            if service_name in self._parameter_futures:
                continue
            attempts = self._parameter_request_attempts.get(service_name, 0)
            if attempts >= 5:
                continue
            client = self._parameter_clients.get(service_name)
            if client is None:
                client = self.create_client(GetParameters, service_name)
                self._parameter_clients[service_name] = client
            if not client.service_is_ready():
                self._parameter_request_attempts[service_name] = attempts + 1
                self._robot_description_check_reason = (
                    "waiting for robot_description parameter service"
                )
                continue
            request = GetParameters.Request()
            request.names = ["robot_description"]
            self._parameter_futures[service_name] = client.call_async(request)
            self._parameter_request_attempts[service_name] = attempts + 1
            self._robot_description_check_reason = (
                f"robot_description parameter query pending from {service_name}"
            )

    def _collect_robot_description_results(self) -> None:
        completed = []
        for service_name, future in self._parameter_futures.items():
            if not future.done():
                continue
            completed.append(service_name)
            try:
                response = future.result()
            except Exception as exc:  # pragma: no cover - defensive ROS callback path
                self.get_logger().debug(
                    f"Failed to read robot_description from {service_name}: {exc}"
                )
                continue
            if response.values and bool(response.values[0].string_value):
                self._robot_description_xml = response.values[0].string_value
                self._robot_description_source = service_name.rsplit(
                    "/get_parameters",
                    1,
                )[0]
                self._robot_description_check_reason = (
                    f"robot_description parameter returned by {service_name}"
                )

        for service_name in completed:
            self._parameter_futures.pop(service_name, None)
        if not self._robot_description_xml and not self._parameter_futures:
            attempted = sorted(self._parameter_request_attempts)
            if attempted:
                self._robot_description_check_reason = (
                    "robot_description parameter not populated by checked service(s): "
                    + ", ".join(attempted)
                )

    def _urdf_report(self) -> dict[str, Any]:
        if not self._robot_description_xml:
            return {
                "robot_description_available": False,
                "urdf_parse_success": False,
                "tool_link_exists_in_urdf": False,
                "urdf_links": [],
            }
        try:
            root = ElementTree.fromstring(self._robot_description_xml)
        except ElementTree.ParseError:
            return {
                "robot_description_available": True,
                "urdf_parse_success": False,
                "tool_link_exists_in_urdf": False,
                "urdf_links": [],
            }
        links = sorted(
            link.attrib["name"]
            for link in root.findall("link")
            if link.attrib.get("name")
        )
        return {
            "robot_description_available": True,
            "urdf_parse_success": True,
            "tool_link_exists_in_urdf": self._tool_link_candidate in set(links),
            "urdf_links": links,
        }

    def _srdf_path(self) -> Path:
        configured_path = str(self.get_parameter("srdf_path").value or "").strip()
        if configured_path:
            return Path(configured_path).expanduser()
        source_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "moveit_lbr_iisy6_r1300"
            / "lbr_iisy6_r1300.srdf"
        )
        if source_path.is_file():
            return source_path
        share_path = Path(get_package_share_directory("kuka_task_control"))
        return (
            share_path
            / "config"
            / "moveit_lbr_iisy6_r1300"
            / "lbr_iisy6_r1300.srdf"
        )

    def _srdf_report(self, srdf_path: Path) -> dict[str, Any]:
        if not srdf_path.is_file():
            return {
                "file_exists": False,
                "parse_success": False,
                "arm_group_found": False,
                "arm_group_joints": [],
                "required_joints_present": False,
                "missing_required_joints": list(self.REQUIRED_JOINTS),
            }
        try:
            root = ElementTree.parse(srdf_path).getroot()
        except (ElementTree.ParseError, OSError):
            return {
                "file_exists": True,
                "parse_success": False,
                "arm_group_found": False,
                "arm_group_joints": [],
                "required_joints_present": False,
                "missing_required_joints": list(self.REQUIRED_JOINTS),
            }
        arm_group = None
        for group in root.findall("group"):
            if group.attrib.get("name") == "arm":
                arm_group = group
                break
        arm_group_joints = []
        if arm_group is not None:
            arm_group_joints = [
                joint.attrib["name"]
                for joint in arm_group.findall("joint")
                if joint.attrib.get("name")
            ]
        arm_group_joint_set = set(arm_group_joints)
        missing_required_joints = [
            joint for joint in self.REQUIRED_JOINTS if joint not in arm_group_joint_set
        ]
        return {
            "file_exists": True,
            "parse_success": True,
            "arm_group_found": arm_group is not None,
            "arm_group_joints": arm_group_joints,
            "required_joints_present": not missing_required_joints,
            "missing_required_joints": missing_required_joints,
        }

    def _tool_axis_candidate_available(self) -> bool:
        if self._tool_axis_payload is None:
            return False
        if (
            self._tool_axis_payload.get("recommended_selected_tool_axis_candidate")
            == self._selected_tool_axis_candidate
        ):
            return True
        scores = self._tool_axis_payload.get("candidate_alignment_scores", {})
        return isinstance(scores, dict) and self._selected_tool_axis_candidate in scores

    def _orientation_targets_available(self) -> bool:
        if self._orientation_targets_payload is None:
            return False
        if (
            self._orientation_targets_payload.get("selected_tool_axis_candidate")
            != self._selected_tool_axis_candidate
        ):
            return False
        return bool(
            self._orientation_targets_payload.get("orientation_targets_available")
        )

    @staticmethod
    def _pose_from_transform(transform: Any | None) -> dict[str, Any] | None:
        if transform is None:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "frame_id": transform.header.frame_id,
            "child_frame_id": transform.child_frame_id,
            "translation": {
                "x": translation.x,
                "y": translation.y,
                "z": translation.z,
            },
            "rotation_xyzw": {
                "x": rotation.x,
                "y": rotation.y,
                "z": rotation.z,
                "w": rotation.w,
            },
        }

    @staticmethod
    def _parse_json(data: str) -> dict[str, Any] | None:
        try:
            payload = json.loads(data)
        except json.JSONDecodeError:
            return None
        return payload if isinstance(payload, dict) else None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ToolLinkValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
