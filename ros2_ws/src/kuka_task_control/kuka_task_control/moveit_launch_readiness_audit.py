"""Diagnostic-only MoveIt launch readiness audit for IK preparation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import rclpy
from ament_index_python.packages import get_packages_with_prefixes
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class MoveItLaunchReadinessAudit(Node):
    """Publish whether a no-motion move_group diagnostic launch is safe to prepare."""

    AUDIT_TOPIC = "/moveit_launch_readiness_audit"
    TARGET_ROBOT_MODEL = "lbr_iisy6_r1300"
    PROJECT_LOCAL_OVERLAY_NAME = "project_local_lbr_iisy6_r1300_overlay"
    REQUIRED_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
    PROJECT_LOCAL_OVERLAY_RELATIVE_PATH = Path(
        "kuka_task_control/config/moveit_lbr_iisy6_r1300"
    )
    EXPLICIT_CONFIG_PACKAGES = (
        "kuka_lbr_iisy_moveit_config",
        "kuka_kr_moveit_config",
        "kuka_lbr_iiwa_moveit_config",
    )
    ROBOT_DESCRIPTION_SERVICE_NAMES = (
        "/robot_state_publisher/get_parameters",
        "/moveit_launch_readiness_audit/get_parameters",
    )

    def __init__(self) -> None:
        super().__init__("moveit_launch_readiness_audit")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("source_search_roots", "")
        self.declare_parameter("startup_grace_period_sec", 1.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("fallback_tool_link_candidate", "tool0")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("robot_description_semantic", "")

        self._started_at = self.get_clock().now()
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._fallback_tool_link_candidate = str(
            self.get_parameter("fallback_tool_link_candidate").value
        )
        self._joint_names_from_joint_states: list[str] = []
        local_robot_description = str(
            self.get_parameter("robot_description").value or ""
        )
        local_robot_description_semantic = str(
            self.get_parameter("robot_description_semantic").value or ""
        )
        self._robot_description_available = bool(local_robot_description)
        self._robot_description_semantic_available = bool(
            local_robot_description_semantic
        )
        self._robot_description_source_node: str | None = (
            "/moveit_launch_readiness_audit"
            if self._robot_description_available
            else None
        )
        self._robot_description_check_reason = (
            "robot_description parameter forwarded to audit node"
            if self._robot_description_available
            else "robot_description not checked yet"
        )
        self._robot_description_xml: str | None = (
            local_robot_description if local_robot_description else None
        )
        self._robot_description_semantic_source: str | None = (
            "/moveit_launch_readiness_audit parameter"
            if self._robot_description_semantic_available
            else None
        )
        self._semantic_diagnostics_report: dict[str, Any] | None = None
        self._tool_link_validation_report: dict[str, Any] | None = None
        self._tool_link_validation_parse_error = False
        self._parameter_clients: dict[str, Any] = {}
        self._parameter_futures: dict[str, Any] = {}
        self._parameter_request_attempts: dict[str, int] = {}
        self._config_report_cache = self._config_report()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(String, self.AUDIT_TOPIC, 10)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
        )
        self.create_subscription(
            String,
            "/robot_description_semantic_diagnostics",
            self._on_semantic_diagnostics,
            10,
        )
        self.create_subscription(
            String,
            "/tool_link_validation",
            self._on_tool_link_validation,
            10,
        )
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_audit,
        )
        self.get_logger().info(
            "MoveIt launch readiness audit started in diagnostic-only no-motion mode."
        )

    def _publish_audit(self) -> None:
        services = self._service_report()
        self._request_description_parameters_if_visible(services["all_services"])
        self._collect_description_parameter_results()
        if not self._startup_grace_period_elapsed():
            return

        report = self._config_report_cache
        semantic_validation = self._semantic_validation_report(report["selected_srdf"])
        exact_robot_semantic_match = bool(report["selected_srdf"])
        semantic_candidate_complete = bool(
            semantic_validation["semantic_model_exact_candidate"]
            and semantic_validation["srdf_file_exists"]
            and semantic_validation["srdf_parse_success"]
            and semantic_validation["arm_group_found"]
            and semantic_validation["required_joints_present"]
            and semantic_validation["joint_states_match_srdf"]
        )
        semantic_candidate_structurally_valid = bool(
            semantic_validation["semantic_model_exact_candidate"]
            and semantic_validation["srdf_file_exists"]
            and semantic_validation["srdf_parse_success"]
            and semantic_validation["arm_group_found"]
            and semantic_validation["required_joints_present"]
        )
        tool_link_requires_validation = bool(
            semantic_validation["tool_link_requires_validation"]
        )
        tool_link_report = self._tool_link_report_for_readiness()
        tool_link_validation_status = self._tool_link_validation_status(tool_link_report)
        tool_link_candidate_valid_for_diagnostics = bool(
            tool_link_validation_status
            == "tool_link_candidate_valid_but_not_motion_approved"
        )
        move_group_launch_found = bool(report["move_group_launch_files"])
        kinematics_yaml_found = bool(report["kinematics_yaml_file"])
        ompl_planning_yaml_found = bool(report["ompl_planning_yaml_file"])
        joint_limits_yaml_found = bool(report["joint_limits_yaml_file"])
        compute_ik_service_available = bool(services["compute_ik_service_available"])
        moveit_launch_ready = False
        compute_ik_expected_after_launch = False
        semantic_diagnostics_available = self._semantic_diagnostics_report is not None
        semantic_diagnostics_status = (
            self._semantic_diagnostics_report.get("semantic_model_validation_status")
            or self._semantic_diagnostics_report.get("status")
            if self._semantic_diagnostics_report
            else "not_observed"
        )
        robot_description_semantic_candidate_available = bool(
            semantic_candidate_structurally_valid
            or (
                self._semantic_diagnostics_report
                and self._semantic_diagnostics_report.get(
                    "robot_description_semantic_available"
                )
            )
        )
        robot_description_semantic_source = self._robot_description_semantic_source
        if self._semantic_diagnostics_report and self._semantic_diagnostics_report.get(
            "srdf_file_path"
        ):
            robot_description_semantic_source = str(
                self._semantic_diagnostics_report["srdf_file_path"]
            )
        elif not robot_description_semantic_source and report["selected_srdf"]:
            robot_description_semantic_source = report["selected_srdf"]

        payload = {
            "status": "moveit_launch_readiness_audit_diagnostic_only_no_motion",
            "moveit_launch_ready": moveit_launch_ready,
            "compute_ik_expected_after_launch": compute_ik_expected_after_launch,
            "exact_robot_semantic_match": exact_robot_semantic_match,
            "same_family_srdf_available": report["same_family_srdf_available"],
            "project_local_moveit_config_found": report[
                "project_local_moveit_config_found"
            ],
            "selected_moveit_config_package": report["selected_moveit_config_package"],
            "selected_moveit_config_package_path": report[
                "selected_moveit_config_package_path"
            ],
            "selected_srdf": report["selected_srdf"],
            "semantic_model_validation_status": semantic_validation[
                "validation_status"
            ],
            "semantic_model_validation": semantic_validation,
            "available_srdf_variants": report["available_srdf_variants"],
            "kinematics_yaml_found": kinematics_yaml_found,
            "kinematics_yaml_file": report["kinematics_yaml_file"],
            "ompl_planning_yaml_found": ompl_planning_yaml_found,
            "ompl_planning_yaml_file": report["ompl_planning_yaml_file"],
            "joint_limits_yaml_found": joint_limits_yaml_found,
            "joint_limits_yaml_file": report["joint_limits_yaml_file"],
            "robot_description_available": self._robot_description_available,
            "robot_description_semantic_available": (
                self._robot_description_semantic_available
            ),
            "robot_description_semantic_candidate_available": (
                robot_description_semantic_candidate_available
            ),
            "robot_description_semantic_source": robot_description_semantic_source,
            "semantic_diagnostics_available": semantic_diagnostics_available,
            "semantic_diagnostics_status": semantic_diagnostics_status,
            "tool_link_validation_available": (
                self._tool_link_validation_report is not None
            ),
            "tool_link_validation_source": tool_link_report["source"],
            "tool_link_candidate": self._tool_link_field(
                tool_link_report,
                "tool_link_candidate",
            ),
            "tool_link_exists_in_urdf": self._tool_link_field(
                tool_link_report,
                "tool_link_exists_in_urdf",
                False,
            ),
            "tf_world_to_tool_available": self._tool_link_field(
                tool_link_report,
                "tf_world_to_tool_available",
                False,
            ),
            "tf_base_to_tool_available": self._tool_link_field(
                tool_link_report,
                "tf_base_to_tool_available",
                False,
            ),
            "selected_tool_axis_candidate": self._tool_link_field(
                tool_link_report,
                "selected_tool_axis_candidate",
            ),
            "tool_axis_candidate_available": self._tool_link_field(
                tool_link_report,
                "tool_axis_candidate_available",
                False,
            ),
            "orientation_targets_available": self._tool_link_field(
                tool_link_report,
                "orientation_targets_available",
                False,
            ),
            "tool_link_validation_status": tool_link_validation_status,
            "tool_link_approved_for_motion": self._tool_link_field(
                tool_link_report,
                "approved_for_motion",
                False,
            ),
            "tool_link_validation_parse_error": (
                self._tool_link_validation_parse_error
            ),
            "robot_description_source_node": self._robot_description_source_node,
            "robot_description_check_reason": self._robot_description_check_reason,
            "robot_joint_names_from_joint_states": list(
                self._joint_names_from_joint_states
            ),
            "robot_joint_names_from_urdf": self._joint_names_from_urdf(),
            "move_group_launch_found": move_group_launch_found,
            "move_group_launch_files": report["move_group_launch_files"],
            "compute_ik_service_available": compute_ik_service_available,
            "controller_motion_allowed": False,
            "trajectory_execution_allowed": False,
            "motion_execution_enabled": False,
            "trajectory_execution_requested": False,
            "recommended_next_step": self._recommended_next_step(
                exact_robot_semantic_match=exact_robot_semantic_match,
                same_family_srdf_available=report["same_family_srdf_available"],
                semantic_candidate_complete=semantic_candidate_complete,
                semantic_candidate_structurally_valid=(
                    semantic_candidate_structurally_valid
                ),
                tool_link_requires_validation=tool_link_requires_validation,
                tool_link_candidate_valid_for_diagnostics=(
                    tool_link_candidate_valid_for_diagnostics
                ),
                move_group_launch_found=move_group_launch_found,
                compute_ik_service_available=compute_ik_service_available,
            ),
            "decision_reason": self._decision_reason(
                exact_robot_semantic_match=exact_robot_semantic_match,
                semantic_candidate_complete=semantic_candidate_complete,
                semantic_candidate_structurally_valid=(
                    semantic_candidate_structurally_valid
                ),
                tool_link_requires_validation=tool_link_requires_validation,
                tool_link_candidate_valid_for_diagnostics=(
                    tool_link_candidate_valid_for_diagnostics
                ),
                move_group_launch_found=move_group_launch_found,
                moveit_launch_ready=moveit_launch_ready,
                compute_ik_service_available=compute_ik_service_available,
            ),
            "target_robot_model": self.TARGET_ROBOT_MODEL,
            "config_package_candidates": report["config_package_candidates"],
            "search_roots": report["search_roots"],
            "services": services,
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _on_joint_states(self, message: JointState) -> None:
        self._joint_names_from_joint_states = list(message.name)

    def _on_semantic_diagnostics(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self._semantic_diagnostics_report = {
                "status": "invalid_robot_description_semantic_diagnostics_json"
            }
            return
        self._semantic_diagnostics_report = payload
        if payload.get("robot_description_semantic_available"):
            self._robot_description_semantic_available = True
            self._robot_description_semantic_source = str(
                payload.get("srdf_file_path")
                or "/robot_description_semantic_diagnostics"
            )

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

    def _startup_grace_period_elapsed(self) -> bool:
        grace_period = float(self.get_parameter("startup_grace_period_sec").value)
        if grace_period <= 0.0:
            return True
        elapsed = (self.get_clock().now() - self._started_at).nanoseconds / 1.0e9
        return elapsed >= grace_period

    def _tool_link_report_for_readiness(self) -> dict[str, Any]:
        if self._tool_link_validation_report is not None:
            return {
                **self._tool_link_validation_report,
                "source": "/tool_link_validation",
            }
        return self._direct_tool_link_fallback_report()

    def _direct_tool_link_fallback_report(self) -> dict[str, Any]:
        tool_link_candidate = self._fallback_tool_link_candidate
        links = self._link_names_from_urdf()
        return {
            "source": "direct_fallback",
            "tool_link_candidate": tool_link_candidate,
            "tool_link_exists_in_urdf": tool_link_candidate in set(links),
            "tf_world_to_tool_available": self._lookup_transform_available(
                self._world_frame,
                tool_link_candidate,
            ),
            "tf_base_to_tool_available": self._lookup_transform_available(
                self._base_frame,
                tool_link_candidate,
            ),
            "selected_tool_axis_candidate": None,
            "tool_axis_candidate_available": False,
            "orientation_targets_available": False,
            "tool_link_validation_status": "not_observed",
            "approved_for_motion": False,
        }

    def _lookup_transform_available(self, target_frame: str, source_frame: str) -> bool:
        try:
            self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as exc:
            self.get_logger().debug(
                f"TF lookup unavailable for {target_frame} -> {source_frame}: {exc}"
            )
            return False
        return True

    def _tool_link_validation_status(self, report: dict[str, Any]) -> str:
        if self._tool_link_validation_report is None:
            return "not_observed"
        status = report.get("tool_link_validation_status")
        return str(status) if status else "tool_link_candidate_incomplete"

    @staticmethod
    def _tool_link_field(
        report: dict[str, Any],
        key: str,
        default: Any = None,
    ) -> Any:
        return report.get(key, default)

    def _service_report(self) -> dict[str, Any]:
        all_services = []
        compute_ik_services = []
        compute_ik_service_available = False

        for service_name, service_types in self.get_service_names_and_types():
            service_types_list = list(service_types)
            entry = {"name": service_name, "types": service_types_list}
            all_services.append(entry)
            type_text = " ".join(service_types_list).lower()
            name_text = service_name.lower()
            if service_name == "/compute_ik":
                compute_ik_service_available = True
            if "compute_ik" in name_text or "getpositionik" in type_text:
                compute_ik_services.append(entry)

        return {
            "compute_ik_service_available": compute_ik_service_available,
            "compute_ik_services": sorted(
                compute_ik_services,
                key=lambda entry: entry["name"],
            ),
            "all_services": sorted(all_services, key=lambda entry: entry["name"]),
        }

    def _config_report(self) -> dict[str, Any]:
        candidates = self._package_candidates()
        candidate_reports = []

        for candidate in candidates:
            root = Path(candidate["share_path"])
            files = self._candidate_files(root)
            candidate_report = {
                "package_name": candidate["package"],
                "package_path": candidate["share_path"],
                "source": candidate["source"],
                **files,
            }
            candidate_report["semantic_model_match_level"] = (
                self._semantic_model_match_level(files["srdf_files"])
            )
            candidate_report["internally_consistent"] = self._internally_consistent(
                candidate_report
            )
            candidate_reports.append(candidate_report)

        selected_package = self._select_moveit_config_package(candidate_reports)
        selected_srdf = (
            self._select_exact_srdf(selected_package["srdf_files"])
            if selected_package
            else None
        )
        available_srdf_variants = sorted(
            {
                srdf_file
                for candidate in candidate_reports
                for srdf_file in candidate["srdf_files"]
            }
        )
        same_family_srdf_available = any(
            candidate["semantic_model_match_level"] == "same_family_not_exact"
            for candidate in candidate_reports
        )
        semantic_model_validation_status = (
            "candidate_requires_validation"
            if selected_package
            and selected_package["package_name"] == self.PROJECT_LOCAL_OVERLAY_NAME
            else ("verified" if selected_srdf else "missing")
        )

        return {
            "config_package_candidates": sorted(
                candidate_reports,
                key=lambda entry: entry["package_name"],
            ),
            "selected_moveit_config_package": (
                selected_package["package_name"] if selected_package else None
            ),
            "selected_moveit_config_package_path": (
                selected_package["package_path"] if selected_package else None
            ),
            "selected_srdf": selected_srdf,
            "semantic_model_validation_status": semantic_model_validation_status,
            "project_local_moveit_config_found": any(
                candidate["package_name"] == self.PROJECT_LOCAL_OVERLAY_NAME
                for candidate in candidate_reports
            ),
            "same_family_srdf_available": same_family_srdf_available,
            "available_srdf_variants": available_srdf_variants,
            "kinematics_yaml_file": self._first_or_none(
                selected_package["kinematics_yaml_files"] if selected_package else []
            ),
            "ompl_planning_yaml_file": self._first_or_none(
                selected_package["ompl_planning_yaml_files"] if selected_package else []
            ),
            "joint_limits_yaml_file": self._first_or_none(
                selected_package["joint_limits_yaml_files"] if selected_package else []
            ),
            "move_group_launch_files": (
                selected_package["move_group_launch_files"] if selected_package else []
            ),
            "search_roots": [str(path) for path in self._search_roots()],
        }

    def _package_candidates(self) -> list[dict[str, str]]:
        candidates: dict[str, dict[str, str]] = {}

        for package_name, prefix in get_packages_with_prefixes().items():
            if not self._is_likely_package_name(package_name):
                continue
            share_path = Path(prefix) / "share" / package_name
            if share_path.exists():
                candidates[str(share_path)] = {
                    "package": package_name,
                    "share_path": str(share_path),
                    "source": "ament_index",
                }

        for root in self._search_roots():
            overlay = root / self.PROJECT_LOCAL_OVERLAY_RELATIVE_PATH
            if overlay.exists():
                candidates[str(overlay)] = {
                    "package": self.PROJECT_LOCAL_OVERLAY_NAME,
                    "share_path": str(overlay),
                    "source": "project_local_config_overlay",
                }
            for package_xml in root.glob("**/package.xml"):
                package_name = self._package_name_from_xml(package_xml)
                if not package_name or not self._is_likely_package_name(package_name):
                    continue
                candidates.setdefault(
                    str(package_xml.parent),
                    {
                        "package": package_name,
                        "share_path": str(package_xml.parent),
                        "source": "source_tree",
                    },
                )

        return sorted(candidates.values(), key=lambda entry: entry["package"])

    def _semantic_validation_report(self, selected_srdf: str | None) -> dict[str, Any]:
        srdf_path = Path(selected_srdf).expanduser() if selected_srdf else None
        srdf_file_exists = bool(srdf_path and srdf_path.is_file())
        srdf_parse_success = False
        arm_group_found = False
        arm_group_joints: list[str] = []
        robot_name: str | None = None

        if srdf_path and srdf_file_exists:
            try:
                root = ElementTree.parse(srdf_path).getroot()
            except (ElementTree.ParseError, OSError):
                root = None
            if root is not None:
                srdf_parse_success = True
                robot_name = (root.attrib.get("name") or "").strip().lower() or None
                arm_group = None
                for group in root.findall("group"):
                    if group.attrib.get("name") == "arm":
                        arm_group = group
                        break
                arm_group_found = arm_group is not None
                if arm_group is not None:
                    arm_group_joints = [
                        joint.attrib["name"]
                        for joint in arm_group.findall("joint")
                        if joint.attrib.get("name")
                    ]

        arm_group_joint_set = set(arm_group_joints)
        joint_state_set = set(self._joint_names_from_joint_states)
        missing_required_joints = [
            joint for joint in self.REQUIRED_JOINTS if joint not in arm_group_joint_set
        ]
        required_joints_present = not missing_required_joints
        joint_states_available = bool(self._joint_names_from_joint_states)
        joint_states_match_srdf = bool(
            joint_states_available
            and arm_group_joints
            and all(joint in joint_state_set for joint in arm_group_joints)
        )
        semantic_model_exact_candidate = bool(
            robot_name == self.TARGET_ROBOT_MODEL
            and srdf_parse_success
            and arm_group_found
            and required_joints_present
        )
        validation_status = (
            "semantic_candidate_valid_but_not_motion_approved"
            if semantic_model_exact_candidate and joint_states_match_srdf
            else "semantic_candidate_incomplete"
        )

        return {
            "semantic_model_exact_candidate": semantic_model_exact_candidate,
            "srdf_file_path": str(srdf_path) if srdf_path else None,
            "srdf_file_exists": srdf_file_exists,
            "srdf_parse_success": srdf_parse_success,
            "arm_group_found": arm_group_found,
            "arm_group_joints": arm_group_joints,
            "required_joints": list(self.REQUIRED_JOINTS),
            "required_joints_present": required_joints_present,
            "missing_required_joints": missing_required_joints,
            "joint_state_names": list(self._joint_names_from_joint_states),
            "joint_states_available": joint_states_available,
            "joint_states_match_srdf": joint_states_match_srdf,
            "tool_link_requires_validation": True,
            "approved_for_motion": False,
            "controller_motion_allowed": False,
            "trajectory_execution_allowed": False,
            "validation_status": validation_status,
        }

    def _candidate_files(self, root: Path) -> dict[str, list[str]]:
        return {
            "srdf_files": self._srdf_files(root),
            "kinematics_yaml_files": self._glob_named_files(root, "kinematics.yaml"),
            "ompl_planning_yaml_files": self._glob_named_files(
                root,
                "ompl_planning.yaml",
            ),
            "joint_limits_yaml_files": self._glob_named_files(root, "joint_limits.yaml"),
            "move_group_launch_files": self._move_group_launch_files(root),
        }

    def _search_roots(self) -> list[Path]:
        roots: list[Path] = []
        configured_roots = self.get_parameter("source_search_roots").value
        if isinstance(configured_roots, str):
            roots.extend(
                Path(path.strip()).expanduser()
                for path in configured_roots.split(":")
                if path.strip()
            )

        cwd = Path.cwd()
        roots.extend(
            [
                cwd / "ros2_ws" / "src",
                cwd / "src",
                cwd.parent / "src",
                Path(__file__).resolve().parents[3] / "src",
            ]
        )
        return self._existing_unique_paths(roots)

    @staticmethod
    def _existing_unique_paths(paths: list[Path]) -> list[Path]:
        unique_paths = []
        seen = set()
        for path in paths:
            try:
                resolved = path.resolve()
            except OSError:
                continue
            if resolved in seen or not resolved.exists():
                continue
            seen.add(resolved)
            unique_paths.append(resolved)
        return unique_paths

    @staticmethod
    def _srdf_files(root: Path) -> list[str]:
        return sorted(
            str(path)
            for pattern in ("**/*.srdf", "**/*.srdf.xacro")
            for path in root.glob(pattern)
            if path.is_file()
        )

    @staticmethod
    def _glob_named_files(root: Path, file_name: str) -> list[str]:
        return sorted(
            str(path) for path in root.glob(f"**/{file_name}") if path.is_file()
        )

    def _move_group_launch_files(self, root: Path) -> list[str]:
        return sorted(
            str(path)
            for path in root.glob("**/*.launch.py")
            if path.is_file() and self._launch_starts_move_group(path)
        )

    @staticmethod
    def _launch_starts_move_group(path: Path) -> bool:
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            return False
        return "moveit_ros_move_group" in text and "move_group" in text

    def _select_exact_srdf(self, srdf_files: list[str]) -> str | None:
        for srdf_file in srdf_files:
            path = Path(srdf_file)
            if self.TARGET_ROBOT_MODEL in path.name.lower():
                return srdf_file
            if self._srdf_robot_name(path) == self.TARGET_ROBOT_MODEL:
                return srdf_file
        return None

    def _semantic_model_match_level(self, srdf_files: list[str]) -> str:
        if self._select_exact_srdf(srdf_files):
            return "exact_match"

        names = [
            self._srdf_model_identifier(Path(srdf_file))
            for srdf_file in srdf_files
        ]
        names = [name for name in names if name]
        if any("lbr_iisy" in name for name in names):
            return "same_family_not_exact"
        if names:
            return "wrong_family"
        return "unknown"

    def _srdf_model_identifier(self, path: Path) -> str | None:
        robot_name = self._srdf_robot_name(path)
        if robot_name:
            return robot_name
        return path.name.lower()

    @staticmethod
    def _internally_consistent(candidate_report: dict[str, Any]) -> bool:
        return bool(
            candidate_report["srdf_files"]
            and candidate_report["kinematics_yaml_files"]
            and candidate_report["ompl_planning_yaml_files"]
            and candidate_report["move_group_launch_files"]
        )

    def _select_moveit_config_package(
        self,
        candidate_reports: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        exact_ready = [
            candidate
            for candidate in candidate_reports
            if candidate["semantic_model_match_level"] == "exact_match"
            and (
                candidate["internally_consistent"]
                or candidate["package_name"] == self.PROJECT_LOCAL_OVERLAY_NAME
            )
        ]
        if not exact_ready:
            return None

        def score(candidate: dict[str, Any]) -> tuple[int, int, str]:
            package_name = candidate["package_name"].lower()
            return (
                1 if package_name == self.PROJECT_LOCAL_OVERLAY_NAME else 0,
                1 if package_name == "kuka_lbr_iisy_moveit_config" else 0,
                1 if candidate["source"] == "ament_index" else 0,
                package_name,
            )

        return sorted(exact_ready, key=score, reverse=True)[0]

    @staticmethod
    def _srdf_robot_name(path: Path) -> str | None:
        try:
            root = ElementTree.parse(path).getroot()
        except (ElementTree.ParseError, OSError):
            return None
        name = root.attrib.get("name")
        return name.strip().lower() if name else None

    def _request_description_parameters_if_visible(
        self,
        service_entries: list[dict[str, Any]],
    ) -> None:
        if (
            self._robot_description_available
            and self._robot_description_semantic_available
        ):
            return

        visible_services = {
            str(entry.get("name", ""))
            for entry in service_entries
            if str(entry.get("name", "")).endswith("/get_parameters")
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
            request.names = ["robot_description", "robot_description_semantic"]
            self._parameter_futures[service_name] = client.call_async(request)
            self._parameter_request_attempts[service_name] = attempts + 1
            self._robot_description_check_reason = (
                f"robot_description parameter query pending from {service_name}"
            )

    def _collect_description_parameter_results(self) -> None:
        completed = []
        for service_name, future in self._parameter_futures.items():
            if not future.done():
                continue
            completed.append(service_name)
            try:
                response = future.result()
            except Exception as exc:  # pragma: no cover - defensive ROS callback path
                self.get_logger().debug(
                    f"Failed to read description parameters from {service_name}: {exc}"
                )
                continue
            for name, value in zip(
                ["robot_description", "robot_description_semantic"],
                response.values,
            ):
                if name == "robot_description" and bool(value.string_value):
                    self._robot_description_available = True
                    self._robot_description_xml = value.string_value
                    self._robot_description_source_node = service_name.rsplit(
                        "/get_parameters", 1
                    )[0]
                    self._robot_description_check_reason = (
                        f"robot_description parameter returned by {service_name}"
                    )
                if name == "robot_description_semantic" and bool(value.string_value):
                    self._robot_description_semantic_available = True
                    self._robot_description_semantic_source = service_name.rsplit(
                        "/get_parameters", 1
                    )[0]

        for service_name in completed:
            self._parameter_futures.pop(service_name, None)
        if not self._robot_description_available and not self._parameter_futures:
            attempted = sorted(self._parameter_request_attempts)
            if attempted:
                self._robot_description_check_reason = (
                    "robot_description parameter not populated by checked service(s): "
                    + ", ".join(attempted)
                )

    def _joint_names_from_urdf(self) -> list[str]:
        if not self._robot_description_xml:
            return []
        try:
            root = ElementTree.fromstring(self._robot_description_xml)
        except ElementTree.ParseError:
            return []
        joint_names = []
        for joint in root.findall("joint"):
            if joint.attrib.get("type") == "fixed":
                continue
            name = joint.attrib.get("name")
            if name:
                joint_names.append(name)
        return sorted(joint_names)

    def _link_names_from_urdf(self) -> list[str]:
        if not self._robot_description_xml:
            return []
        try:
            root = ElementTree.fromstring(self._robot_description_xml)
        except ElementTree.ParseError:
            return []
        return sorted(
            link.attrib["name"]
            for link in root.findall("link")
            if link.attrib.get("name")
        )

    @classmethod
    def _is_likely_package_name(cls, package_name: str) -> bool:
        name = package_name.lower()
        return name in cls.EXPLICIT_CONFIG_PACKAGES or "moveit_config" in name

    @staticmethod
    def _package_name_from_xml(package_xml: Path) -> str | None:
        try:
            root = ElementTree.parse(package_xml).getroot()
        except (ElementTree.ParseError, OSError):
            return None
        name = root.findtext("name")
        return name.strip() if name else None

    @staticmethod
    def _first_or_none(paths: list[str]) -> str | None:
        unique_paths = sorted(set(paths))
        return unique_paths[0] if unique_paths else None

    @staticmethod
    def _recommended_next_step(
        *,
        exact_robot_semantic_match: bool,
        same_family_srdf_available: bool,
        semantic_candidate_complete: bool,
        semantic_candidate_structurally_valid: bool,
        tool_link_requires_validation: bool,
        tool_link_candidate_valid_for_diagnostics: bool,
        move_group_launch_found: bool,
        compute_ik_service_available: bool,
    ) -> str:
        if not exact_robot_semantic_match:
            if same_family_srdf_available:
                return "create_lbr_iisy6_r1300_srdf_from_same_family_template"
            return "create_or_select_matching_srdf_for_lbr_iisy6_r1300"
        if (
            semantic_candidate_structurally_valid
            and tool_link_requires_validation
            and tool_link_candidate_valid_for_diagnostics
        ):
            return "prepare_move_group_diagnostic_launch_inputs"
        if semantic_candidate_structurally_valid and tool_link_requires_validation:
            return "validate_tool_link_and_prepare_move_group_diagnostic_launch"
        if not semantic_candidate_complete:
            return "complete_semantic_model_validation"
        if not move_group_launch_found:
            return "create_move_group_diagnostic_launch"
        if not compute_ik_service_available:
            return "launch_move_group_diagnostic_only"
        return "test_compute_ik_service_no_motion"

    @staticmethod
    def _decision_reason(
        *,
        exact_robot_semantic_match: bool,
        semantic_candidate_complete: bool,
        semantic_candidate_structurally_valid: bool,
        tool_link_requires_validation: bool,
        tool_link_candidate_valid_for_diagnostics: bool,
        move_group_launch_found: bool,
        moveit_launch_ready: bool,
        compute_ik_service_available: bool,
    ) -> str:
        if not exact_robot_semantic_match:
            return (
                "No exact LBR iisy 6 R1300 semantic model was found; move_group "
                "launch remains blocked."
            )
        if (
            semantic_candidate_structurally_valid
            and tool_link_requires_validation
            and tool_link_candidate_valid_for_diagnostics
        ):
            return (
                "The exact SRDF candidate and diagnostic tool link candidate are "
                "valid for launch preparation, but motion and compute_ik remain "
                "blocked until a separate no-motion move_group diagnostic launch "
                "is prepared."
            )
        if semantic_candidate_structurally_valid and tool_link_requires_validation:
            return (
                "The exact SRDF candidate is structurally valid, but the tool "
                "link and end-effector assumptions still require validation."
            )
        if not semantic_candidate_complete:
            return (
                "The exact semantic model candidate exists, but its machine-readable "
                "validation checks are incomplete."
            )
        if not move_group_launch_found:
            return (
                "The exact semantic model exists, but no launch file that starts "
                "move_group was found."
            )
        if not compute_ik_service_available:
            if moveit_launch_ready:
                return (
                    "Safe diagnostic launch inputs are present, but /compute_ik "
                    "is not running."
                )
            return (
                "A move_group launch file exists, but required diagnostic launch "
                "inputs are incomplete."
            )
        return (
            "The /compute_ik service is visible; only no-motion diagnostic IK "
            "requests are appropriate."
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MoveItLaunchReadinessAudit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
