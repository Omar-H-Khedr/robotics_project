"""Diagnostic-only IK feasibility monitor for Cartesian peg/hole targets."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class IkFeasibilityDiagnostics(Node):
    """Publish conservative reachability diagnostics without commanding motion."""

    DIAGNOSTIC_TOPIC = "/ik_feasibility_diagnostics"
    DEFAULT_CONFIG_FILE = "peg_hole_cartesian_targets.yaml"
    TARGET_POSE_FRAMES = (
        "hole_center",
        "pre_insertion_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
    )
    FALLBACK_JOINT_LIMITS = {
        "joint_1": {"min_position": -2.8, "max_position": 2.8},
        "joint_2": {"min_position": -2.3, "max_position": 2.3},
        "joint_3": {"min_position": -2.8, "max_position": 2.8},
        "joint_4": {"min_position": -2.3, "max_position": 2.3},
        "joint_5": {"min_position": -2.8, "max_position": 2.8},
        "joint_6": {"min_position": -3.0, "max_position": 3.0},
    }

    def __init__(self) -> None:
        super().__init__("ik_feasibility_diagnostics")
        self.declare_parameter("config_path", "")
        self.declare_parameter("joint_limits_config_path", "")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("approx_workspace_min_radius_m", 0.10)
        self.declare_parameter("approx_workspace_max_radius_m", 0.76)

        self._config = self._load_config(self._resolve_config_path())
        self._world_frame = str(self._config.get("world_frame", "world"))
        self._base_frame = str(self._config.get("robot_base_frame", "base_link"))
        self._tool_frame = str(self._config.get("tool_frame", "tool0"))
        self._joint_limits, self._joint_limit_source = self._load_joint_limits()
        self._joint_names = list(self._joint_limits.keys())
        self._current_joint_names: list[str] = []
        self._current_joint_positions: list[float] = []

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(String, self.DIAGNOSTIC_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_diagnostics,
        )

        self.get_logger().info(
            "IK feasibility diagnostics started in diagnostic_only_no_motion mode."
        )

    def _resolve_config_path(self) -> Path:
        configured_path = str(self.get_parameter("config_path").value).strip()
        if configured_path:
            return Path(configured_path).expanduser()
        return (
            Path(get_package_share_directory("kuka_task_control"))
            / "config"
            / self.DEFAULT_CONFIG_FILE
        )

    def _resolve_joint_limits_path(self) -> Path | None:
        configured_path = str(self.get_parameter("joint_limits_config_path").value).strip()
        if configured_path:
            return Path(configured_path).expanduser()
        try:
            return (
                Path(get_package_share_directory("kuka_lbr_iisy_support"))
                / "config"
                / "lbr_iisy3_r760_joint_limits.yaml"
            )
        except PackageNotFoundError:
            return None

    @staticmethod
    def _load_config(config_path: Path) -> dict[str, Any]:
        with config_path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}
        if not isinstance(config, dict):
            raise ValueError(f"Cartesian insertion config must be a map: {config_path}")
        return config

    def _load_joint_limits(self) -> tuple[dict[str, dict[str, float | None]], str]:
        joint_limits_path = self._resolve_joint_limits_path()
        if joint_limits_path is not None and joint_limits_path.exists():
            config = self._load_config(joint_limits_path)
            raw_limits = config.get("joint_limits", {})
            if isinstance(raw_limits, dict) and raw_limits:
                limits = {}
                for name, values in raw_limits.items():
                    if not isinstance(values, dict):
                        continue
                    fallback = self.FALLBACK_JOINT_LIMITS.get(str(name), {})
                    limits[str(name)] = {
                        "min_position": self._optional_float(
                            values.get("min_position", fallback.get("min_position"))
                        ),
                        "max_position": self._optional_float(
                            values.get("max_position", fallback.get("max_position"))
                        ),
                    }
                if limits:
                    return limits, f"{joint_limits_path} with fallback position limits"

        return dict(self.FALLBACK_JOINT_LIMITS), "fallback_conservative_limits"

    def _joint_state_callback(self, message: JointState) -> None:
        self._current_joint_names = list(message.name)
        self._current_joint_positions = [float(position) for position in message.position]

    def _publish_diagnostics(self) -> None:
        current_base_pose_world = self._lookup_pose(self._world_frame, self._base_frame)
        current_tool_pose_world = self._lookup_pose(self._world_frame, self._tool_frame)
        current_tool_pose_base = self._lookup_pose(self._base_frame, self._tool_frame)
        hole_center_world = self._lookup_pose(self._world_frame, "hole_center")
        ik_solver_available, ik_solver_services = self._detect_ik_solver()

        targets = {}
        for target_name in self.TARGET_POSE_FRAMES:
            targets[target_name] = self._target_diagnostics(
                target_name=target_name,
                current_tool_pose_world=current_tool_pose_world,
                current_tool_pose_base=current_tool_pose_base,
                hole_center_world=hole_center_world,
                ik_solver_available=ik_solver_available,
            )

        all_geometrically_feasible = all(
            target["approximate_workspace_feasible"] is True
            for target in targets.values()
        )

        payload = {
            "status": "ik_feasibility_diagnostic_only_no_motion",
            "frames": {
                "world_frame": self._world_frame,
                "robot_base_frame": self._base_frame,
                "tool_frame": self._tool_frame,
            },
            "current_joint_names": self._current_joint_names,
            "current_joint_positions": self._current_joint_positions,
            "configured_joint_names": self._joint_names,
            "joint_limits": self._joint_limits,
            "joint_limits_source": self._joint_limit_source,
            "current_base_pose_world": current_base_pose_world,
            "current_tool_pose_world": current_tool_pose_world,
            "current_tool_pose_base": current_tool_pose_base,
            "object_frames_used": list(self.TARGET_POSE_FRAMES),
            "targets": targets,
            "all_targets_geometrically_feasible": all_geometrically_feasible,
            "ik_solver_available": ik_solver_available,
            "ik_solver_services": ik_solver_services,
            "motion_execution_enabled": False,
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _target_diagnostics(
        self,
        target_name: str,
        current_tool_pose_world: dict[str, Any] | None,
        current_tool_pose_base: dict[str, Any] | None,
        hole_center_world: dict[str, Any] | None,
        ik_solver_available: bool,
    ) -> dict[str, Any]:
        target_pose_world = self._lookup_pose(self._world_frame, target_name)
        target_pose_base = self._lookup_pose(self._base_frame, target_name)
        radial_distance = self._radial_distance(target_pose_base)
        approximate_workspace_feasible = self._within_workspace(radial_distance)

        if approximate_workspace_feasible is False:
            feasibility_status = "geometric_infeasible"
        elif approximate_workspace_feasible is True:
            feasibility_status = "geometric_feasible_no_ik_solver"
        else:
            feasibility_status = "geometric_infeasible"

        return {
            "target_pose_world": target_pose_world,
            "target_pose_base": target_pose_base,
            "current_tool_pose_world": current_tool_pose_world,
            "current_tool_pose_base": current_tool_pose_base,
            "translational_distance_from_current_tool": self._distance_between(
                current_tool_pose_world,
                target_pose_world,
            ),
            "z_offset_from_hole_center": self._z_offset(target_pose_world, hole_center_world),
            "target_distance_from_base": radial_distance,
            "approximate_workspace_min_radius_m": float(
                self.get_parameter("approx_workspace_min_radius_m").value
            ),
            "approximate_workspace_max_radius_m": float(
                self.get_parameter("approx_workspace_max_radius_m").value
            ),
            "approximate_workspace_feasible": approximate_workspace_feasible,
            "requires_ik_solver": True,
            "ik_solver_available": ik_solver_available,
            "ik_solution_available": None,
            "feasibility_status": feasibility_status,
        }

    def _detect_ik_solver(self) -> tuple[bool, list[str]]:
        services = []
        for service_name, service_types in self.get_service_names_and_types():
            type_text = " ".join(service_types)
            if service_name.endswith("compute_ik") or "GetPositionIK" in type_text:
                services.append(service_name)
        services = sorted(set(services))
        return bool(services), services

    def _lookup_pose(self, target_frame: str, source_frame: str) -> dict[str, Any] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(
                f"TF lookup unavailable for {target_frame} -> {source_frame}: {exc}"
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            "frame": target_frame,
            "child_frame": source_frame,
            "position_xyz": [translation.x, translation.y, translation.z],
            "orientation_xyzw": [rotation.x, rotation.y, rotation.z, rotation.w],
        }

    def _within_workspace(self, radial_distance: float | None) -> bool | None:
        if radial_distance is None:
            return None
        min_radius = float(self.get_parameter("approx_workspace_min_radius_m").value)
        max_radius = float(self.get_parameter("approx_workspace_max_radius_m").value)
        return min_radius <= radial_distance <= max_radius

    @staticmethod
    def _radial_distance(pose: dict[str, Any] | None) -> float | None:
        if pose is None:
            return None
        position = pose.get("position_xyz")
        if not isinstance(position, list) or len(position) != 3:
            return None
        return math.sqrt(sum(float(value) ** 2 for value in position))

    @staticmethod
    def _distance_between(
        first_pose: dict[str, Any] | None,
        second_pose: dict[str, Any] | None,
    ) -> float | None:
        if first_pose is None or second_pose is None:
            return None
        first_position = first_pose.get("position_xyz")
        second_position = second_pose.get("position_xyz")
        if not isinstance(first_position, list) or not isinstance(second_position, list):
            return None
        if len(first_position) != 3 or len(second_position) != 3:
            return None
        return math.sqrt(
            sum(
                (float(first_position[index]) - float(second_position[index])) ** 2
                for index in range(3)
            )
        )

    @staticmethod
    def _z_offset(
        target_pose: dict[str, Any] | None,
        hole_center_pose: dict[str, Any] | None,
    ) -> float | None:
        if target_pose is None or hole_center_pose is None:
            return None
        target_position = target_pose.get("position_xyz")
        hole_position = hole_center_pose.get("position_xyz")
        if not isinstance(target_position, list) or not isinstance(hole_position, list):
            return None
        if len(target_position) != 3 or len(hole_position) != 3:
            return None
        return float(target_position[2]) - float(hole_position[2])

    @staticmethod
    def _optional_float(value: Any) -> float | None:
        if value is None:
            return None
        return float(value)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = IkFeasibilityDiagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
