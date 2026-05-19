"""Publish diagnostic-only robot_description_semantic readiness from the SRDF."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String


class RobotDescriptionSemanticDiagnostics(Node):
    """Publish file-backed SRDF diagnostics without launching MoveIt or motion."""

    TOPIC = "/robot_description_semantic_diagnostics"
    REQUIRED_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")

    def __init__(self) -> None:
        super().__init__("robot_description_semantic_diagnostics")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("srdf_path", "")

        self._publisher = self.create_publisher(String, self.TOPIC, 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_diagnostics,
        )
        self.get_logger().info(
            "robot_description_semantic diagnostics started in no-motion mode."
        )

    def _publish_diagnostics(self) -> None:
        srdf_path = self._srdf_path()
        srdf_report = self._srdf_report(srdf_path)
        semantic_available = bool(
            srdf_report["file_exists"]
            and srdf_report["parse_success"]
            and srdf_report["semantic_text"]
        )
        semantic_status = (
            "semantic_candidate_valid_but_not_motion_approved"
            if semantic_available
            and srdf_report["contains_group_arm"]
            and srdf_report["references_required_joints"]
            else "semantic_candidate_incomplete"
        )

        payload: dict[str, Any] = {
            "status": "robot_description_semantic_diagnostics_no_motion",
            "tool_link_candidate": "tool0",
            "tool_link_validation_required": True,
            "tool_link_validation_topic": "/tool_link_validation",
            "srdf_file_path": str(srdf_path),
            "srdf_file_exists": srdf_report["file_exists"],
            "srdf_parse_success": srdf_report["parse_success"],
            "robot_description_semantic_available": semantic_available,
            "robot_description_semantic_length": len(srdf_report["semantic_text"]),
            "arm_group_found": srdf_report["contains_group_arm"],
            "arm_group_joints": srdf_report["joint_names"],
            "required_joints": list(self.REQUIRED_JOINTS),
            "required_joints_present": srdf_report["references_required_joints"],
            "missing_required_joints": srdf_report["missing_joints"],
            "semantic_model_validation_status": semantic_status,
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
        semantic_text = ""
        if not srdf_path.is_file():
            return {
                "file_exists": False,
                "parse_success": False,
                "semantic_text": semantic_text,
                "contains_group_arm": False,
                "joint_names": [],
                "references_required_joints": False,
                "missing_joints": list(self.REQUIRED_JOINTS),
            }

        try:
            semantic_text = srdf_path.read_text(encoding="utf-8")
            root = ElementTree.fromstring(semantic_text)
        except (ElementTree.ParseError, OSError, UnicodeDecodeError):
            return {
                "file_exists": True,
                "parse_success": False,
                "semantic_text": semantic_text,
                "contains_group_arm": False,
                "joint_names": [],
                "references_required_joints": False,
                "missing_joints": list(self.REQUIRED_JOINTS),
            }

        arm_group = None
        for group in root.findall("group"):
            if group.attrib.get("name") == "arm":
                arm_group = group
                break

        joint_names = []
        if arm_group is not None:
            joint_names = [
                joint.attrib["name"]
                for joint in arm_group.findall("joint")
                if joint.attrib.get("name")
            ]
        joint_set = set(joint_names)
        missing_joints = [
            joint for joint in self.REQUIRED_JOINTS if joint not in joint_set
        ]
        return {
            "file_exists": True,
            "parse_success": True,
            "semantic_text": semantic_text,
            "contains_group_arm": arm_group is not None,
            "joint_names": joint_names,
            "references_required_joints": not missing_joints,
            "missing_joints": missing_joints,
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotDescriptionSemanticDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
