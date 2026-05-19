"""Diagnostic-only semantic model validator for LBR iisy 6 R1300."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class SemanticModelValidator(Node):
    """Publish conservative validation status for the project-local SRDF candidate."""

    TOPIC = "/semantic_model_validation"
    TARGET_ROBOT_MODEL = "lbr_iisy6_r1300"
    REQUIRED_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")

    def __init__(self) -> None:
        super().__init__("semantic_model_validator")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("srdf_path", "")
        self.declare_parameter("tool_link_requires_validation", True)

        self._joint_state_names: list[str] = []
        self._joint_states_observed = False
        self._publisher = self.create_publisher(String, self.TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_validation,
        )
        self.get_logger().info(
            "Semantic model validator started in diagnostic-only no-motion mode."
        )

    def _on_joint_states(self, message: JointState) -> None:
        self._joint_states_observed = True
        self._joint_state_names = list(message.name)

    def _publish_validation(self) -> None:
        srdf_path = self._srdf_path()
        srdf_report = self._srdf_report(srdf_path)
        joint_state_set = set(self._joint_state_names)
        joint_states_match_srdf = bool(
            self._joint_states_observed
            and srdf_report["parse_success"]
            and srdf_report["joint_names"]
            and all(joint in joint_state_set for joint in srdf_report["joint_names"])
        )
        tool_link_requires_validation = bool(
            self.get_parameter("tool_link_requires_validation").value
        )
        semantic_model_exact_candidate = bool(
            srdf_report["robot_name"] == self.TARGET_ROBOT_MODEL
            and srdf_report["parse_success"]
            and srdf_report["contains_group_arm"]
            and srdf_report["references_required_joints"]
        )
        candidate_complete = bool(
            semantic_model_exact_candidate and joint_states_match_srdf
        )

        payload: dict[str, Any] = {
            "status": "semantic_model_validation_diagnostic_only_no_motion",
            "target_robot_model": self.TARGET_ROBOT_MODEL,
            "selected_moveit_config_package": "project_local_lbr_iisy6_r1300_overlay",
            "selected_srdf": str(srdf_path),
            "semantic_model_exact_candidate": semantic_model_exact_candidate,
            "srdf_file_path": str(srdf_path),
            "srdf_file_exists": srdf_report["file_exists"],
            "srdf_parse_success": srdf_report["parse_success"],
            "arm_group_found": srdf_report["contains_group_arm"],
            "arm_group_joints": srdf_report["joint_names"],
            "required_joints": list(self.REQUIRED_JOINTS),
            "required_joints_present": srdf_report["references_required_joints"],
            "missing_required_joints": srdf_report["missing_joints"],
            "joint_state_names": list(self._joint_state_names),
            "joint_states_available": self._joint_states_observed,
            "joint_states_match_srdf": joint_states_match_srdf,
            "tool_link_requires_validation": tool_link_requires_validation,
            "approved_for_motion": False,
            "controller_motion_allowed": False,
            "trajectory_execution_allowed": False,
            "validation_status": (
                "semantic_candidate_valid_but_not_motion_approved"
                if candidate_complete
                else "semantic_candidate_incomplete"
            ),
            "semantic_model_validation_status": (
                "semantic_candidate_valid_but_not_motion_approved"
                if candidate_complete
                else "semantic_candidate_incomplete"
            ),
            # Legacy field names retained for consumers that have not migrated.
            "srdf_exists": srdf_path.is_file(),
            "srdf_robot_name": srdf_report["robot_name"],
            "srdf_contains_group_arm": srdf_report["contains_group_arm"],
            "srdf_joint_names": srdf_report["joint_names"],
            "srdf_references_required_joints": srdf_report[
                "references_required_joints"
            ],
            "missing_srdf_joints": srdf_report["missing_joints"],
            "joint_states_contain_required_joints": bool(
                self._joint_states_observed
                and all(joint in joint_state_set for joint in self.REQUIRED_JOINTS)
            ),
            "missing_joint_state_joints": [
                joint for joint in self.REQUIRED_JOINTS if joint not in joint_state_set
            ],
            "tool_link_validation_status": (
                "requires_validation"
                if tool_link_requires_validation
                else "validated"
            ),
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
        if not srdf_path.is_file():
            return {
                "file_exists": False,
                "parse_success": False,
                "robot_name": None,
                "contains_group_arm": False,
                "joint_names": [],
                "references_required_joints": False,
                "missing_joints": list(self.REQUIRED_JOINTS),
            }
        try:
            root = ElementTree.parse(srdf_path).getroot()
        except (ElementTree.ParseError, OSError):
            return {
                "file_exists": True,
                "parse_success": False,
                "robot_name": None,
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
            "robot_name": (root.attrib.get("name") or "").strip().lower() or None,
            "contains_group_arm": arm_group is not None,
            "joint_names": joint_names,
            "references_required_joints": not missing_joints,
            "missing_joints": missing_joints,
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticModelValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
