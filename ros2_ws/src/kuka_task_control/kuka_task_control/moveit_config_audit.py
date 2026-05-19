"""Diagnostic-only MoveIt configuration readiness audit."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
    get_packages_with_prefixes,
)
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class MoveItConfigAudit(Node):
    """Publish MoveIt config readiness without calling IK or executing motion."""

    AUDIT_TOPIC = "/moveit_config_audit"
    REQUIRED_MOVEIT_PACKAGES = (
        "moveit_ros_move_group",
        "moveit_msgs",
        "moveit_kinematics",
    )
    PACKAGE_NAME_HINTS = (
        "moveit_config",
        "lbr_iisy",
        "kuka",
        "iisy",
    )
    TARGET_FILES = (
        "*.srdf",
        "kinematics.yaml",
        "joint_limits.yaml",
        "ompl_planning.yaml",
        "move_group.launch.py",
        "demo.launch.py",
    )

    def __init__(self) -> None:
        super().__init__("moveit_config_audit")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("source_search_roots", "")

        self._joint_names: list[str] = []
        self._joint_states_observed = False
        self._robot_description_available = self.has_parameter("robot_description")
        self._robot_description_source = (
            "moveit_config_audit parameter"
            if self._robot_description_available
            else "not_observed"
        )
        self._robot_description_clients: dict[str, Any] = {}
        self._robot_description_futures: dict[str, Any] = {}
        self._robot_description_checked_services: set[str] = set()
        self._package_report_cache = self._package_report()
        self._moveit_config_report_cache = self._moveit_config_report()

        self._publisher = self.create_publisher(String, self.AUDIT_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_audit,
        )
        self.get_logger().info(
            "MoveIt config audit started in diagnostic-only no-motion mode."
        )

    def _on_joint_states(self, message: JointState) -> None:
        self._joint_states_observed = True
        self._joint_names = list(message.name)

    def _publish_audit(self) -> None:
        services = self._service_report()
        self._request_robot_description_if_visible(services["all_services"])
        self._collect_robot_description_results()

        package_report = self._package_report_cache
        config_report = self._moveit_config_report_cache
        robot_description_report = self._robot_description_report()
        compute_ik_service_available = services["compute_ik_service_available"]

        moveit_config_package_found = bool(config_report["moveit_config_package_found"])
        partial_moveit_config_found = bool(config_report["partial_moveit_config_found"])
        srdf_found = bool(config_report["srdf_files"])
        kinematics_yaml_found = bool(config_report["kinematics_yaml_files"])
        joint_limits_yaml_found = bool(config_report["joint_limits_yaml_files"])
        ompl_planning_yaml_found = bool(config_report["ompl_planning_yaml_files"])
        move_group_launch_found = bool(config_report["move_group_launch_files"])
        move_group_can_be_launched_safely = bool(config_report["can_construct_launch"])
        moveit_ready_for_compute_ik = bool(
            move_group_can_be_launched_safely and compute_ik_service_available
        )

        payload = {
            "status": "moveit_config_audit_diagnostic_only_no_motion",
            "controller_motion_allowed": False,
            "trajectory_execution_allowed": False,
            "motion_execution_enabled": False,
            "trajectory_execution_requested": False,
            "available_packages": package_report["available_packages"],
            "missing_packages": package_report["missing_packages"],
            "packages": package_report["packages"],
            "moveit_config_package_found": moveit_config_package_found,
            "partial_moveit_config_found": partial_moveit_config_found,
            "moveit_config_package_name": config_report["moveit_config_package_name"],
            "moveit_config_package_path": config_report["moveit_config_package_path"],
            "srdf_found": srdf_found,
            "srdf_files": config_report["srdf_files"],
            "kinematics_yaml_found": kinematics_yaml_found,
            "kinematics_yaml_files": config_report["kinematics_yaml_files"],
            "joint_limits_yaml_found": joint_limits_yaml_found,
            "joint_limits_yaml_files": config_report["joint_limits_yaml_files"],
            "ompl_planning_yaml_found": ompl_planning_yaml_found,
            "ompl_planning_yaml_files": config_report["ompl_planning_yaml_files"],
            "move_group_launch_found": move_group_launch_found,
            "move_group_launch_files": config_report["move_group_launch_files"],
            "compute_ik_service_available": compute_ik_service_available,
            "moveit_ready_for_compute_ik": moveit_ready_for_compute_ik,
            "move_group_can_be_launched_safely": move_group_can_be_launched_safely,
            "robot_description_available": robot_description_report["available"],
            "robot_description_reason": robot_description_report["reason"],
            "joint_states_available": self._joint_states_observed,
            "joint_names_observed": list(self._joint_names),
            "config_audit": config_report,
            "services": services,
            "recommended_next_step": self._recommended_next_step(
                moveit_config_package_found=moveit_config_package_found,
                partial_moveit_config_found=partial_moveit_config_found,
                srdf_found=srdf_found,
                kinematics_yaml_found=kinematics_yaml_found,
                compute_ik_service_available=compute_ik_service_available,
            ),
            "decision_reason": self._decision_reason(
                moveit_config_package_found=moveit_config_package_found,
                partial_moveit_config_found=partial_moveit_config_found,
                srdf_found=srdf_found,
                kinematics_yaml_found=kinematics_yaml_found,
                joint_limits_yaml_found=joint_limits_yaml_found,
                ompl_planning_yaml_found=ompl_planning_yaml_found,
                move_group_launch_found=move_group_launch_found,
                compute_ik_service_available=compute_ik_service_available,
            ),
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _service_report(self) -> dict[str, Any]:
        all_services = []
        compute_ik_services = []
        moveit_services = []
        compute_ik_service_available = False

        for service_name, service_types in self.get_service_names_and_types():
            service_types_list = list(service_types)
            entry = {"name": service_name, "types": service_types_list}
            all_services.append(entry)
            name_text = service_name.lower()
            type_text = " ".join(service_types_list).lower()
            if service_name == "/compute_ik":
                compute_ik_service_available = True
            if "compute_ik" in name_text or "getpositionik" in type_text:
                compute_ik_services.append(entry)
            if (
                "move_group" in name_text
                or "moveit" in name_text
                or "planning" in name_text
                or "move_group" in type_text
                or "planning" in type_text
            ):
                moveit_services.append(entry)

        return {
            "compute_ik_service_available": compute_ik_service_available,
            "compute_ik_services": sorted(
                compute_ik_services,
                key=lambda item: item["name"],
            ),
            "moveit_services": sorted(moveit_services, key=lambda item: item["name"]),
            "all_services": sorted(all_services, key=lambda item: item["name"]),
        }

    def _package_report(self) -> dict[str, Any]:
        packages = {
            package_name: self._package_availability(package_name)
            for package_name in self.REQUIRED_MOVEIT_PACKAGES
        }
        return {
            "packages": packages,
            "available_packages": sorted(
                name for name, report in packages.items() if report["available"]
            ),
            "missing_packages": sorted(
                name for name, report in packages.items() if not report["available"]
            ),
        }

    def _moveit_config_report(self) -> dict[str, Any]:
        package_candidates = self._package_candidates()
        complete_config_packages = []
        partial_config_packages = []
        srdf_files: list[str] = []
        kinematics_yaml_files: list[str] = []
        joint_limits_yaml_files: list[str] = []
        ompl_planning_yaml_files: list[str] = []
        move_group_launch_files: list[str] = []
        demo_launch_files: list[str] = []
        move_group_related_launch_files: list[str] = []

        for candidate in package_candidates:
            share_path = Path(candidate["share_path"])
            files = self._find_config_files(share_path)
            candidate_report = {**candidate, **files}
            has_moveit_files = any(files[file_key] for file_key in files)
            if has_moveit_files:
                candidate_report["complete_config"] = self._is_complete_config(files)
                candidate_report["can_construct_launch"] = self._can_construct_launch(
                    files
                )
                if candidate_report["complete_config"]:
                    complete_config_packages.append(candidate_report)
                else:
                    partial_config_packages.append(candidate_report)
            srdf_files.extend(files["srdf_files"])
            kinematics_yaml_files.extend(files["kinematics_yaml_files"])
            joint_limits_yaml_files.extend(files["joint_limits_yaml_files"])
            ompl_planning_yaml_files.extend(files["ompl_planning_yaml_files"])
            move_group_launch_files.extend(files["move_group_launch_files"])
            demo_launch_files.extend(files["demo_launch_files"])
            move_group_related_launch_files.extend(
                files["move_group_related_launch_files"]
            )

        selected_package = self._select_config_package(complete_config_packages)
        if selected_package:
            report_files = selected_package
        else:
            report_files = {
                "srdf_files": sorted(set(srdf_files)),
                "kinematics_yaml_files": sorted(set(kinematics_yaml_files)),
                "joint_limits_yaml_files": sorted(set(joint_limits_yaml_files)),
                "ompl_planning_yaml_files": sorted(set(ompl_planning_yaml_files)),
                "move_group_launch_files": sorted(set(move_group_launch_files)),
                "demo_launch_files": sorted(set(demo_launch_files)),
                "move_group_related_launch_files": sorted(
                    set(move_group_related_launch_files)
                ),
            }
        partial_moveit_config_found = bool(
            not selected_package
            and (
                partial_config_packages
                or srdf_files
                or kinematics_yaml_files
                or joint_limits_yaml_files
                or ompl_planning_yaml_files
                or move_group_launch_files
                or demo_launch_files
                or move_group_related_launch_files
            )
        )

        return {
            "package_candidates": package_candidates,
            "config_packages": complete_config_packages,
            "partial_config_packages": partial_config_packages,
            "moveit_config_package_found": bool(selected_package),
            "partial_moveit_config_found": partial_moveit_config_found,
            "moveit_config_package_name": (
                selected_package["package"] if selected_package else None
            ),
            "moveit_config_package_path": (
                selected_package["share_path"] if selected_package else None
            ),
            "can_construct_launch": bool(
                selected_package and selected_package["can_construct_launch"]
            ),
            "srdf_files": sorted(set(report_files["srdf_files"])),
            "kinematics_yaml_files": sorted(set(report_files["kinematics_yaml_files"])),
            "joint_limits_yaml_files": sorted(set(report_files["joint_limits_yaml_files"])),
            "ompl_planning_yaml_files": sorted(
                set(report_files["ompl_planning_yaml_files"])
            ),
            "move_group_launch_files": sorted(set(report_files["move_group_launch_files"])),
            "demo_launch_files": sorted(set(report_files["demo_launch_files"])),
            "move_group_related_launch_files": sorted(
                set(report_files["move_group_related_launch_files"])
            ),
            "source_search_roots": [str(path) for path in self._source_search_roots()],
            "install_search_roots": [str(path) for path in self._install_search_roots()],
        }

    def _package_candidates(self) -> list[dict[str, Any]]:
        candidates: dict[str, dict[str, Any]] = {}

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

        for source_root in self._source_search_roots():
            for package_xml in source_root.glob("**/package.xml"):
                package_name = self._package_name_from_xml(package_xml)
                if not package_name or not self._is_likely_package_name(package_name):
                    continue
                package_path = package_xml.parent
                candidates.setdefault(
                    str(package_path),
                    {
                        "package": package_name,
                        "share_path": str(package_path),
                        "source": "source_tree",
                    },
                )
            for package_path in self._hinted_directories_with_config_files(source_root):
                package_name = package_path.name
                candidates.setdefault(
                    str(package_path),
                    {
                        "package": package_name,
                        "share_path": str(package_path),
                        "source": "source_tree_directory",
                    },
                )

        for install_root in self._install_search_roots():
            share_root = install_root / "share"
            if share_root.exists():
                for share_path in share_root.iterdir():
                    if not share_path.is_dir():
                        continue
                    package_name = share_path.name
                    if not self._is_likely_package_name(package_name):
                        continue
                    candidates.setdefault(
                        str(share_path),
                        {
                            "package": package_name,
                            "share_path": str(share_path),
                            "source": "install_tree",
                        },
                    )
            for package_xml in install_root.glob("**/package.xml"):
                package_name = self._package_name_from_xml(package_xml)
                if not package_name or not self._is_likely_package_name(package_name):
                    continue
                package_path = package_xml.parent
                candidates.setdefault(
                    str(package_path),
                    {
                        "package": package_name,
                        "share_path": str(package_path),
                        "source": "install_tree",
                    },
                )
            for package_path in self._hinted_directories_with_config_files(install_root):
                package_name = package_path.name
                candidates.setdefault(
                    str(package_path),
                    {
                        "package": package_name,
                        "share_path": str(package_path),
                        "source": "install_tree_directory",
                    },
                )

        return sorted(candidates.values(), key=lambda entry: entry["package"])

    def _source_search_roots(self) -> list[Path]:
        roots: list[Path] = []
        configured_roots = self.get_parameter("source_search_roots").value
        if isinstance(configured_roots, str):
            roots.extend(
                Path(path.strip()).expanduser()
                for path in configured_roots.split(":")
                if path.strip()
            )

        cwd = Path.cwd()
        candidate_roots = [
            cwd / "ros2_ws" / "src",
            cwd / "src",
            cwd.parent / "src",
            Path(__file__).resolve().parents[3] / "src",
        ]
        roots.extend(candidate_roots)

        return self._existing_unique_paths(roots)

    def _install_search_roots(self) -> list[Path]:
        cwd = Path.cwd()
        candidate_roots = [
            cwd / "ros2_ws" / "install",
            cwd / "install",
            cwd.parent / "install",
            Path(__file__).resolve().parents[3] / "install",
        ]
        return self._existing_unique_paths(candidate_roots)

    def _find_config_files(self, root: Path) -> dict[str, list[str]]:
        srdf_files = sorted(
            str(path)
            for path in root.glob("**/*.srdf")
            if path.is_file()
        )
        kinematics_yaml_files = self._glob_named_files(root, "kinematics.yaml")
        joint_limits_yaml_files = self._glob_named_files(root, "joint_limits.yaml")
        ompl_planning_yaml_files = self._glob_named_files(root, "ompl_planning.yaml")
        move_group_launch_files = self._glob_named_files(root, "move_group.launch.py")
        demo_launch_files = self._glob_named_files(root, "demo.launch.py")
        move_group_related_launch_files = sorted(
            str(path)
            for path in root.glob("**/*.launch.py")
            if path.is_file() and self._launch_file_mentions_move_group(path)
        )
        return {
            "srdf_files": srdf_files,
            "kinematics_yaml_files": kinematics_yaml_files,
            "joint_limits_yaml_files": joint_limits_yaml_files,
            "ompl_planning_yaml_files": ompl_planning_yaml_files,
            "move_group_launch_files": move_group_launch_files,
            "demo_launch_files": demo_launch_files,
            "move_group_related_launch_files": move_group_related_launch_files,
        }

    def _hinted_directories_with_config_files(self, root: Path) -> list[Path]:
        directories: dict[str, Path] = {}
        for file_pattern in self.TARGET_FILES:
            for path in root.glob(f"**/{file_pattern}"):
                if not path.is_file():
                    continue
                for parent in [path.parent, *path.parents]:
                    if parent == root.parent:
                        break
                    if self._is_likely_package_name(parent.name):
                        directories[str(parent)] = parent
                        break
        return sorted(directories.values(), key=lambda path: path.name)

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
    def _can_construct_launch(files: dict[str, list[str]]) -> bool:
        return bool(
            files["move_group_launch_files"]
            or (
                files["srdf_files"]
                and files["kinematics_yaml_files"]
                and files["ompl_planning_yaml_files"]
            )
        )

    @classmethod
    def _is_complete_config(cls, files: dict[str, list[str]]) -> bool:
        return bool(
            files["srdf_files"]
            and files["kinematics_yaml_files"]
            and cls._can_construct_launch(files)
        )

    @staticmethod
    def _select_config_package(
        complete_config_packages: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        if not complete_config_packages:
            return None

        def score(candidate: dict[str, Any]) -> tuple[int, int, int, int, str]:
            package_name = candidate["package"].lower()
            return (
                1 if "lbr_iisy" in package_name else 0,
                1 if candidate["move_group_launch_files"] else 0,
                1 if candidate["joint_limits_yaml_files"] else 0,
                1 if candidate["source"] == "ament_index" else 0,
                package_name,
            )

        return sorted(complete_config_packages, key=score, reverse=True)[0]

    def _request_robot_description_if_visible(
        self,
        service_entries: list[dict[str, Any]],
    ) -> None:
        if self._robot_description_available:
            return
        for entry in service_entries:
            service_name = str(entry.get("name", ""))
            if (
                not service_name.endswith("/get_parameters")
                or "robot_state_publisher" not in service_name
                or service_name in self._robot_description_checked_services
            ):
                continue
            client = self._robot_description_clients.get(service_name)
            if client is None:
                client = self.create_client(GetParameters, service_name)
                self._robot_description_clients[service_name] = client
            if not client.service_is_ready():
                continue
            request = GetParameters.Request()
            request.names = ["robot_description"]
            self._robot_description_futures[service_name] = client.call_async(request)
            self._robot_description_checked_services.add(service_name)

    def _collect_robot_description_results(self) -> None:
        for service_name, future in list(self._robot_description_futures.items()):
            if not future.done():
                continue
            self._robot_description_futures.pop(service_name, None)
            try:
                response = future.result()
            except Exception as exc:  # pragma: no cover - defensive ROS graph handling
                self.get_logger().debug(
                    f"robot_description parameter query failed for {service_name}: {exc}"
                )
                continue
            for value in response.values:
                if value.string_value:
                    self._robot_description_available = True
                    self._robot_description_source = service_name
                    return

    def _robot_description_report(self) -> dict[str, bool | None | str]:
        if self._robot_description_available:
            return {
                "available": True,
                "reason": f"robot_description parameter returned by {self._robot_description_source}",
            }
        if self._robot_description_futures:
            return {
                "available": None,
                "reason": "robot_description parameter query pending",
            }
        if self._robot_description_checked_services:
            checked = ", ".join(sorted(self._robot_description_checked_services))
            return {
                "available": False,
                "reason": f"robot_description parameter was not populated by checked service(s): {checked}",
            }
        return {
            "available": None,
            "reason": "robot_state_publisher get_parameters service not observed yet",
        }

    @classmethod
    def _is_likely_package_name(cls, package_name: str) -> bool:
        lowered = package_name.lower()
        return any(hint in lowered for hint in cls.PACKAGE_NAME_HINTS)

    @staticmethod
    def _package_availability(package_name: str) -> dict[str, Any]:
        try:
            share_path = get_package_share_directory(package_name)
        except PackageNotFoundError:
            return {"available": False, "share_path": ""}
        return {"available": True, "share_path": share_path}

    @staticmethod
    def _package_name_from_xml(package_xml: Path) -> str | None:
        try:
            root = ElementTree.parse(package_xml).getroot()
        except (ElementTree.ParseError, OSError):
            return None
        name = root.findtext("name")
        return name.strip() if name else None

    @staticmethod
    def _glob_named_files(root: Path, file_name: str) -> list[str]:
        return sorted(str(path) for path in root.glob(f"**/{file_name}") if path.is_file())

    @staticmethod
    def _launch_file_mentions_move_group(path: Path) -> bool:
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            return False
        return "move_group" in text and "moveit_ros_move_group" in text

    @staticmethod
    def _recommended_next_step(
        *,
        moveit_config_package_found: bool,
        partial_moveit_config_found: bool,
        srdf_found: bool,
        kinematics_yaml_found: bool,
        compute_ik_service_available: bool,
    ) -> str:
        if compute_ik_service_available:
            return "test_compute_ik_service_no_motion"
        if moveit_config_package_found:
            return "prepare_move_group_diagnostic_launch"
        if partial_moveit_config_found or srdf_found or kinematics_yaml_found:
            return "complete_moveit_config_package"
        return "create_moveit_config_package"

    @staticmethod
    def _decision_reason(
        *,
        moveit_config_package_found: bool,
        partial_moveit_config_found: bool,
        srdf_found: bool,
        kinematics_yaml_found: bool,
        joint_limits_yaml_found: bool,
        ompl_planning_yaml_found: bool,
        move_group_launch_found: bool,
        compute_ik_service_available: bool,
    ) -> str:
        if compute_ik_service_available:
            return "The /compute_ik service is visible; only no-motion diagnostic IK calls are appropriate."
        if not moveit_config_package_found:
            if partial_moveit_config_found:
                present = []
                if srdf_found:
                    present.append("SRDF")
                if kinematics_yaml_found:
                    present.append("kinematics.yaml")
                if joint_limits_yaml_found:
                    present.append("joint_limits.yaml")
                if ompl_planning_yaml_found:
                    present.append("ompl_planning.yaml")
                if move_group_launch_found:
                    present.append("move_group.launch.py")
                return (
                    "Partial MoveIt config artifacts were found, but no complete "
                    f"config package was selected. Present artifacts: {', '.join(present)}."
                )
            return "No likely KUKA LBR iisy MoveIt config package was found in installed or source package shares."
        if move_group_launch_found:
            return "A complete MoveIt config package and move_group launch file were found, but /compute_ik is not running."
        return "A complete MoveIt config package was found and a diagnostic launch can be prepared from explicit config file paths, but /compute_ik is not running."


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MoveItConfigAudit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
