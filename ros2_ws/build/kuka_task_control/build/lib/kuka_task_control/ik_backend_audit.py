"""Diagnostic-only IK backend infrastructure audit."""

from __future__ import annotations

import json
from importlib import import_module
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class IkBackendAudit(Node):
    """Publish available IK infrastructure without solving IK or moving the robot."""

    AUDIT_TOPIC = "/ik_backend_audit"
    PACKAGE_NAMES = (
        "moveit_ros_move_group",
        "moveit_msgs",
        "moveit_kinematics",
        "trac_ik_kinematics_plugin",
        "kdl_parser_py",
        "urdf_parser_py",
    )
    KUKA_RESOURCE_PACKAGES = (
        "kuka_lbr_iisy_support",
        "kuka_lbr_iisy_moveit_config",
    )
    MOVEIT_SERVICE_HINTS = (
        "compute_ik",
        "plan_kinematic_path",
        "get_planning_scene",
        "query_planner_interface",
        "move_group",
        "moveit",
    )

    def __init__(self) -> None:
        super().__init__("ik_backend_audit")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("joint_limits_config_path", "")

        self._joint_names: list[str] = []
        self._joint_states_observed = False
        self._joint_positions_observed = False
        self._dry_run_payload: dict[str, Any] | None = None
        self._orientation_payload: dict[str, Any] | None = None
        self._execution_gate_payload: dict[str, Any] | None = None
        self._robot_description_available = False
        self._robot_description_source = "not_observed"
        self._robot_description_clients: dict[str, Any] = {}
        self._robot_description_futures: dict[str, Any] = {}
        self._robot_description_checked_services: set[str] = set()

        self._publisher = self.create_publisher(String, self.AUDIT_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(
            String,
            "/cartesian_insertion_dry_run_plan",
            self._on_dry_run_plan,
            10,
        )
        self.create_subscription(
            String,
            "/cartesian_orientation_targets",
            self._on_orientation_targets,
            10,
        )
        self.create_subscription(
            String,
            "/execution_gate_status",
            self._on_execution_gate_status,
            10,
        )
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_audit,
        )

        self.get_logger().info(
            "IK backend audit started in diagnostic-only no-motion mode."
        )

    def _on_joint_states(self, message: JointState) -> None:
        self._joint_states_observed = True
        self._joint_names = list(message.name)
        self._joint_positions_observed = (
            self._joint_positions_observed or bool(message.position)
        )

    def _on_dry_run_plan(self, message: String) -> None:
        self._dry_run_payload = self._parse_json(message.data)

    def _on_orientation_targets(self, message: String) -> None:
        self._orientation_payload = self._parse_json(message.data)

    def _on_execution_gate_status(self, message: String) -> None:
        self._execution_gate_payload = self._parse_json(message.data)

    def _publish_audit(self) -> None:
        services = self._service_report()
        package_report = self._package_report()
        self._request_robot_description_if_visible(services["all_services"])
        self._collect_robot_description_results()
        resources = self._robot_resource_report()
        project_readiness = self._project_readiness_report()
        robot_description_report = self._robot_description_report()

        moveit_msgs_available = bool(
            package_report["packages"]["moveit_msgs"]["available"]
        )
        moveit_ik_messages_importable = self._moveit_ik_messages_importable()
        exact_compute_ik_available = services["exact_compute_ik_available"]
        callable_compute_ik_available = bool(
            exact_compute_ik_available
            and moveit_msgs_available
            and moveit_ik_messages_importable
        )
        ik_backend_available = callable_compute_ik_available
        recommended_backend = self._recommended_backend(
            exact_compute_ik_available=exact_compute_ik_available,
            ik_backend_available=ik_backend_available,
            package_report=package_report,
        )

        payload = {
            "status": "ik_backend_audit_diagnostic_only_no_motion",
            "motion_execution_enabled": False,
            "trajectory_execution_requested": False,
            "controller_motion_allowed": False,
            "compute_ik_service_available": exact_compute_ik_available,
            "compute_ik_services": services["compute_ik_services"],
            "ik_services": services["ik_services"],
            "moveit_services": services["moveit_services"],
            "moveit_packages_available": package_report["moveit_packages_available"],
            "available_packages": package_report["available_packages"],
            "missing_packages": package_report["missing_packages"],
            "robot_description_available": robot_description_report["available"],
            "robot_description_reason": robot_description_report["reason"],
            "joint_states_available": self._joint_states_observed,
            "joint_names_observed": list(self._joint_names),
            "joint_positions_observed": self._joint_positions_observed,
            "joint_limits_file_available": resources["joint_limits_file_available"],
            "joint_limits_file_path": resources["joint_limits_file_path"] or None,
            "full_pose_dry_run_available": project_readiness[
                "full_pose_dry_run_available"
            ],
            "orientation_targets_available": project_readiness[
                "orientation_targets_available"
            ],
            "execution_gate_status_observed": project_readiness[
                "execution_gate_status_observed"
            ],
            "services": services,
            "packages": package_report,
            "robot_model_resources": resources,
            "existing_project_ik_readiness": project_readiness,
            "ik_backend_available": ik_backend_available,
            "recommended_backend": recommended_backend,
            "recommended_next_step": self._recommended_next_step(recommended_backend),
            "decision_reason": self._decision_reason(
                services=services,
                package_report=package_report,
                ik_backend_available=ik_backend_available,
            ),
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _service_report(self) -> dict[str, Any]:
        service_entries = []
        compute_ik_services = []
        ik_services = []
        moveit_services = []
        exact_compute_ik_available = False

        for service_name, service_types in self.get_service_names_and_types():
            service_types_list = list(service_types)
            service_entries.append(
                {"name": service_name, "types": service_types_list}
            )
            type_text = " ".join(service_types_list).lower()
            name_text = service_name.lower()
            if service_name == "/compute_ik":
                exact_compute_ik_available = True
            if "compute_ik" in name_text or "getpositionik" in type_text:
                service_entry = {"name": service_name, "types": service_types_list}
                compute_ik_services.append(service_entry)
                ik_services.append(service_entry)
            if (
                "move_group" in name_text
                or "move_group" in type_text
                or "planning" in name_text
                or "planning" in type_text
                or any(
                    hint in name_text or hint in type_text
                    for hint in self.MOVEIT_SERVICE_HINTS
                )
            ):
                moveit_services.append({"name": service_name, "types": service_types_list})

        return {
            "exact_compute_ik_available": exact_compute_ik_available,
            "compute_ik_services": sorted(
                compute_ik_services,
                key=lambda entry: entry["name"],
            ),
            "ik_services": sorted(
                ik_services,
                key=lambda entry: entry["name"],
            ),
            "moveit_services": sorted(
                moveit_services,
                key=lambda entry: entry["name"],
            ),
            "moveit_planning_services": sorted(
                moveit_services,
                key=lambda entry: entry["name"],
            ),
            "all_services": sorted(service_entries, key=lambda entry: entry["name"]),
        }

    def _package_report(self) -> dict[str, Any]:
        packages = {
            package_name: self._package_availability(package_name)
            for package_name in self.PACKAGE_NAMES
        }
        moveit_packages_available = any(
            packages[name]["available"]
            for name in ("moveit_ros_move_group", "moveit_msgs", "moveit_kinematics")
        )
        available_packages = sorted(
            name for name, report in packages.items() if report["available"]
        )
        missing_packages = sorted(
            name for name, report in packages.items() if not report["available"]
        )
        return {
            "packages": packages,
            "moveit_packages_available": moveit_packages_available,
            "available_packages": available_packages,
            "missing_packages": missing_packages,
        }

    def _robot_resource_report(self) -> dict[str, Any]:
        joint_limits = self._joint_limits_report()
        kuka_paths = self._kuka_resource_paths()
        return {
            "robot_description_parameter_available": self._robot_description_available,
            "robot_description_source": self._robot_description_source,
            "joint_names_available_from_joint_states": bool(self._joint_names),
            "joint_names_from_joint_states": self._joint_names,
            "joint_states_observed": self._joint_states_observed,
            "joint_positions_observed": self._joint_positions_observed,
            "joint_limits_file_available": joint_limits["available"],
            "joint_limits_file_readable": joint_limits["readable"],
            "joint_limits_file_path": joint_limits["path"],
            "kuka_lbr_iisy_resource_paths": kuka_paths,
            "kuka_lbr_iisy_urdf_xacro_discoverable": any(
                entry["urdf_xacro_paths"] for entry in kuka_paths
            ),
        }

    def _project_readiness_report(self) -> dict[str, Any]:
        return {
            "full_pose_dry_run_available": self._full_pose_dry_run_available(),
            "cartesian_insertion_dry_run_plan_observed": self._dry_run_payload is not None,
            "orientation_targets_available": self._orientation_targets_available(),
            "cartesian_orientation_targets_observed": self._orientation_payload is not None,
            "execution_gate_status_observed": self._execution_gate_payload is not None,
            "execution_gate_controller_execution_allowed": (
                self._execution_gate_payload.get("controller_execution_allowed")
                if self._execution_gate_payload is not None
                else None
            ),
            "dry_run_plan_executable": (
                self._dry_run_payload.get("plan_executable")
                if self._dry_run_payload is not None
                else None
            ),
            "dry_run_primary_block_reason": (
                self._dry_run_payload.get("primary_block_reason")
                if self._dry_run_payload is not None
                else None
            ),
        }

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

    def _joint_limits_report(self) -> dict[str, Any]:
        configured_path = str(self.get_parameter("joint_limits_config_path").value).strip()
        if configured_path:
            path = Path(configured_path).expanduser()
        else:
            try:
                path = (
                    Path(get_package_share_directory("kuka_lbr_iisy_support"))
                    / "config"
                    / "lbr_iisy3_r760_joint_limits.yaml"
                )
            except PackageNotFoundError:
                path = None

        if path is None:
            return {"available": False, "readable": False, "path": None}
        return {
            "available": path.exists(),
            "readable": path.is_file() and self._is_readable(path),
            "path": str(path),
        }

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

    def _kuka_resource_paths(self) -> list[dict[str, Any]]:
        results = []
        for package_name in self.KUKA_RESOURCE_PACKAGES:
            try:
                share_path = Path(get_package_share_directory(package_name))
            except PackageNotFoundError:
                results.append(
                    {
                        "package": package_name,
                        "available": False,
                        "share_path": "",
                        "urdf_xacro_paths": [],
                    }
                )
                continue
            urdf_dir = share_path / "urdf"
            urdf_paths = []
            if urdf_dir.exists():
                urdf_paths = sorted(
                    str(path)
                    for path in urdf_dir.glob("*iisy*.xacro")
                    if path.is_file()
                )
            results.append(
                {
                    "package": package_name,
                    "available": True,
                    "share_path": str(share_path),
                    "urdf_xacro_paths": urdf_paths,
                }
            )
        return results

    def _recommended_backend(
        self,
        *,
        exact_compute_ik_available: bool,
        ik_backend_available: bool,
        package_report: dict[str, Any],
    ) -> str:
        if exact_compute_ik_available and ik_backend_available:
            return "moveit_compute_ik"
        if package_report["moveit_packages_available"]:
            return "configure_moveit"
        return "add_moveit_or_custom_ik_service"

    @staticmethod
    def _recommended_next_step(recommended_backend: str) -> str:
        if recommended_backend == "moveit_compute_ik":
            return (
                "Use the visible /compute_ik service only for diagnostic IK requests, "
                "then require real joint solutions for every full-pose waypoint before "
                "any controller execution is reconsidered."
            )
        if recommended_backend == "configure_moveit":
            return (
                "Configure and launch MoveIt/move_group for the KUKA LBR iisy model so "
                "a real /compute_ik service is available; keep controller execution "
                "blocked during validation."
            )
        return (
            "Add a MoveIt compute_ik backend or a project-owned custom IK service; do "
            "not fabricate joint targets or enable trajectory execution."
        )

    @staticmethod
    def _decision_reason(
        *,
        services: dict[str, Any],
        package_report: dict[str, Any],
        ik_backend_available: bool,
    ) -> str:
        if ik_backend_available:
            return "The /compute_ik service is visible and MoveIt IK messages are importable."
        if package_report["moveit_packages_available"]:
            return "MoveIt-related packages are visible, but no callable IK service is running."
        if services["compute_ik_services"]:
            return "A compute_ik-like service is visible, but required IK message support is missing."
        return "No callable IK backend is visible on the ROS graph."

    def _full_pose_dry_run_available(self) -> bool:
        if self._dry_run_payload is None:
            return False
        return bool(self._dry_run_payload.get("all_waypoints_have_full_pose", False))

    def _orientation_targets_available(self) -> bool:
        if self._orientation_payload is None:
            return False
        return bool(self._orientation_payload.get("orientation_targets_available", False))

    @staticmethod
    def _parse_json(raw: str) -> dict[str, Any] | None:
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return None
        return parsed if isinstance(parsed, dict) else None

    @staticmethod
    def _package_availability(package_name: str) -> dict[str, Any]:
        try:
            share_path = get_package_share_directory(package_name)
        except PackageNotFoundError:
            return {"available": False, "share_path": ""}
        return {"available": True, "share_path": share_path}

    @staticmethod
    def _moveit_ik_messages_importable() -> bool:
        try:
            srv_module = import_module("moveit_msgs.srv")
            getattr(srv_module, "GetPositionIK")
        except (ImportError, AttributeError):
            return False
        return True

    @staticmethod
    def _is_readable(path: Path) -> bool:
        try:
            with path.open("r", encoding="utf-8"):
                return True
        except OSError:
            return False


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = IkBackendAudit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
