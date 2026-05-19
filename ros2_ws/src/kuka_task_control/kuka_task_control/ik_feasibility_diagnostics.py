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
        "staging_pose",
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
        "retreat_pose",
    )
    INSERTION_ALIGNED_TARGETS = (
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
    )
    ORIENTATION_AWARE_TARGETS = (
        "staging_pose",
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
        "retreat_pose",
    )
    TOOL_AXES_LOCAL = {
        "tool0_+X": [1.0, 0.0, 0.0],
        "tool0_-X": [-1.0, 0.0, 0.0],
        "tool0_+Y": [0.0, 1.0, 0.0],
        "tool0_-Y": [0.0, -1.0, 0.0],
        "tool0_+Z": [0.0, 0.0, 1.0],
        "tool0_-Z": [0.0, 0.0, -1.0],
    }
    XY_TOLERANCE_M = 0.002
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
        self._targets = self._config.get("targets", {})
        if not isinstance(self._targets, dict):
            raise ValueError("peg_hole_cartesian_targets.yaml field 'targets' must be a map")
        self._joint_limits, self._joint_limit_source = self._load_joint_limits()
        self._joint_names = list(self._joint_limits.keys())
        self._current_joint_names: list[str] = []
        self._current_joint_positions: list[float] = []
        self._last_safety_status: dict[str, Any] | None = None
        self._last_safety_status_raw: str | None = None
        self._orientation_targets_payload: dict[str, Any] | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(String, self.DIAGNOSTIC_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)
        self.create_subscription(String, "/safety_status", self._safety_status_callback, 10)
        self.create_subscription(
            String,
            "/cartesian_orientation_targets",
            self._orientation_targets_callback,
            10,
        )
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

    def _safety_status_callback(self, message: String) -> None:
        self._last_safety_status_raw = message.data.strip()
        try:
            parsed = json.loads(message.data)
        except json.JSONDecodeError:
            parsed = None
        self._last_safety_status = parsed if isinstance(parsed, dict) else None

    def _orientation_targets_callback(self, message: String) -> None:
        try:
            parsed = json.loads(message.data)
        except json.JSONDecodeError:
            parsed = None
        self._orientation_targets_payload = parsed if isinstance(parsed, dict) else None

    def _publish_diagnostics(self) -> None:
        current_base_pose_world = self._lookup_pose(self._world_frame, self._base_frame)
        current_tool_pose_world = self._lookup_pose(self._world_frame, self._tool_frame)
        current_tool_pose_base = self._lookup_pose(self._base_frame, self._tool_frame)
        hole_center_world, hole_center_world_source = self._target_pose_from_tf_or_yaml(
            self._world_frame,
            "hole_center",
        )
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

        target_poses_world = {
            "hole_center": hole_center_world,
            **{
                target_name: target["target_pose_world"]
                for target_name, target in targets.items()
            },
        }
        geometry_validity = self._geometry_validity(target_poses_world)
        all_geometrically_feasible = all(
            target["approximate_workspace_feasible"] is True
            for target in targets.values()
        )
        orientation_validated = self._tool_orientation_validated()
        orientation_targets_available = self._orientation_targets_available()
        full_pose_targets_available = self._full_pose_targets_available(targets)
        safety_guard_active = self._safety_guard_active()
        ik_solution_available = None if not ik_solver_available else False
        real_ik_solution_available = False

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
            "hole_center_world": hole_center_world,
            "hole_center_world_source": hole_center_world_source,
            "object_frames_used": list(self.TARGET_POSE_FRAMES),
            "targets": targets,
            "cartesian_geometry_valid": geometry_validity["cartesian_geometry_valid"],
            "geometry_validity": geometry_validity,
            "all_targets_geometrically_feasible": all_geometrically_feasible,
            "orientation_targets_available": orientation_targets_available,
            "orientation_aware_ik_checked": full_pose_targets_available,
            "full_pose_targets_available": full_pose_targets_available,
            "ik_solver_available": ik_solver_available,
            "ik_solver_services": ik_solver_services,
            "real_ik_solution_available": real_ik_solution_available,
            "executable_plan_available": False,
            "execution_gates": {
                "geometry_valid": geometry_validity["cartesian_geometry_valid"],
                "ik_available": ik_solver_available,
                "ik_solution_available": ik_solution_available,
                "orientation_targets_available": orientation_targets_available,
                "full_pose_targets_available": full_pose_targets_available,
                "tool_axis_orientation_validated": orientation_validated,
                "safety_guard_active": safety_guard_active,
            },
            "motion_execution_allowed": False,
            "motion_execution_block_reason": self._motion_block_reason(
                geometry_validity["cartesian_geometry_valid"],
                ik_solver_available,
                real_ik_solution_available,
                orientation_validated,
                safety_guard_active,
            ),
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
        target_pose_world, target_pose_world_source = self._target_pose_from_tf_or_yaml(
            self._world_frame,
            target_name,
        )
        target_pose_base, target_pose_base_source = self._target_pose_from_tf_or_yaml(
            self._base_frame,
            target_name,
        )
        target_position_world = self._position(target_pose_world)
        target_orientation_world = self._target_orientation_from_orientation_targets(target_name)
        orientation_target_available = target_orientation_world is not None
        full_target_pose_world = self._full_target_pose(
            target_pose_world,
            target_orientation_world,
        )
        alignment_dot, alignment_angle_deg = self._desired_tool_axis_alignment(
            target_orientation_world,
        )
        radial_distance = self._radial_distance(target_pose_base)
        approximate_workspace_feasible = self._within_workspace(radial_distance)

        if target_name not in self.ORIENTATION_AWARE_TARGETS:
            full_pose_feasibility_status = "position_only_diagnostic_target"
        elif full_target_pose_world is None:
            full_pose_feasibility_status = "full_pose_target_unavailable"
        elif not ik_solver_available:
            full_pose_feasibility_status = "full_pose_ready_but_no_ik_solver"
        else:
            full_pose_feasibility_status = "full_pose_ready_real_ik_not_computed"

        if approximate_workspace_feasible is False:
            feasibility_status = "geometric_infeasible"
        elif full_pose_feasibility_status == "full_pose_ready_but_no_ik_solver":
            feasibility_status = "full_pose_ready_but_no_ik_solver"
        elif full_pose_feasibility_status == "full_pose_ready_real_ik_not_computed":
            feasibility_status = "full_pose_ready_real_ik_not_computed"
        elif approximate_workspace_feasible is True:
            feasibility_status = "approx_geometric_feasible_orientation_target_unavailable"
        else:
            feasibility_status = "target_pose_unavailable"

        return {
            "target_pose_world": target_pose_world,
            "target_pose_world_source": target_pose_world_source,
            "target_position_world": target_position_world,
            "target_orientation_world": target_orientation_world,
            "full_target_pose_world": full_target_pose_world,
            "orientation_source": "cartesian_orientation_targets"
            if target_name in self.ORIENTATION_AWARE_TARGETS
            and orientation_target_available
            else None,
            "desired_tool_axis_alignment_dot": alignment_dot,
            "desired_tool_axis_alignment_angle_deg": alignment_angle_deg,
            "orientation_target_available": orientation_target_available,
            "orientation_validated": False,
            "full_pose_feasibility_status": full_pose_feasibility_status,
            "target_pose_base": target_pose_base,
            "target_pose_base_source": target_pose_base_source,
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
            "ik_solution_available": None if not ik_solver_available else False,
            "executable_plan_available": False,
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

    def _target_pose_from_tf_or_yaml(
        self,
        target_frame: str,
        target_name: str,
    ) -> tuple[dict[str, Any] | None, str | None]:
        pose = self._lookup_pose(target_frame, target_name)
        if pose is not None:
            return pose, "tf"
        if target_frame != self._world_frame:
            return None, None
        pose = self._target_pose_from_yaml(target_name)
        if pose is not None:
            return pose, "yaml_fallback"
        return None, None

    def _target_pose_from_yaml(self, target_name: str) -> dict[str, Any] | None:
        target = self._targets.get(target_name)
        if not isinstance(target, dict):
            return None
        position = target.get("position_xyz")
        if not self._is_vector(position, 3):
            return None
        orientation = target.get("orientation_xyzw")
        if not self._is_vector(orientation, 4):
            orientation = [0.0, 0.0, 0.0, 1.0]
        return {
            "frame": str(target.get("frame", self._world_frame)),
            "child_frame": target_name,
            "position_xyz": [float(value) for value in position],
            "orientation_xyzw": [float(value) for value in orientation],
            "orientation_placeholder": not self._is_vector(
                target.get("orientation_xyzw"),
                4,
            ),
        }

    @staticmethod
    def _is_vector(value: Any, length: int) -> bool:
        return isinstance(value, list) and len(value) == length

    def _tool_orientation_validated(self) -> bool:
        if self._orientation_targets_payload is None:
            return False
        return bool(self._orientation_targets_payload.get("orientation_validated", False))

    def _orientation_targets_available(self) -> bool:
        if self._orientation_targets_payload is None:
            return False
        if self._orientation_targets_payload.get("orientation_targets_available") is True:
            return True
        desired = self._orientation_targets_payload.get("desired_orientations_world", {})
        if not isinstance(desired, dict):
            return False
        return any(
            self._target_orientation_from_orientation_targets(target_name) is not None
            for target_name in self.ORIENTATION_AWARE_TARGETS
        )

    def _target_orientation_from_orientation_targets(
        self,
        target_name: str,
    ) -> list[float] | None:
        if self._orientation_targets_payload is None:
            return None
        desired = self._orientation_targets_payload.get("desired_orientations_world", {})
        if not isinstance(desired, dict):
            return None
        target = desired.get(target_name)
        if not isinstance(target, dict):
            return None
        orientation = target.get("orientation_xyzw")
        if not self._is_vector(orientation, 4):
            return None
        return [float(value) for value in orientation]

    def _full_pose_targets_available(self, targets: dict[str, dict[str, Any]]) -> bool:
        for target_name in self.ORIENTATION_AWARE_TARGETS:
            target = targets.get(target_name)
            if not isinstance(target, dict):
                return False
            if target.get("target_position_world") is None:
                return False
            if target.get("target_orientation_world") is None:
                return False
        return True

    def _full_target_pose(
        self,
        target_pose_world: dict[str, Any] | None,
        target_orientation_world: list[float] | None,
    ) -> dict[str, Any] | None:
        if target_pose_world is None or target_orientation_world is None:
            return None
        position = self._position(target_pose_world)
        if position is None:
            return None
        return {
            "frame": self._world_frame,
            "child_frame": target_pose_world.get("child_frame"),
            "position_xyz": position,
            "orientation_xyzw": target_orientation_world,
            "position_source": target_pose_world.get("frame_source")
            or "tf_or_yaml_fallback",
            "orientation_source": "cartesian_orientation_targets",
        }

    def _desired_tool_axis_alignment(
        self,
        target_orientation_world: list[float] | None,
    ) -> tuple[float | None, float | None]:
        if target_orientation_world is None:
            return None, None
        selected_axis = self._selected_tool_axis_candidate()
        local_axis = self.TOOL_AXES_LOCAL.get(selected_axis)
        if local_axis is None:
            return None, None
        desired_axis_world = self._normalize(
            self._rotate_vector(target_orientation_world, local_axis)
        )
        insertion_axis_world = self._insertion_axis_world()
        dot = self._dot(desired_axis_world, insertion_axis_world)
        clamped_dot = max(-1.0, min(1.0, dot))
        alignment_dot = 1.0 if abs(clamped_dot - 1.0) < 1e-9 else clamped_dot
        alignment_angle_deg = 0.0 if alignment_dot == 1.0 else math.degrees(
            math.acos(alignment_dot)
        )
        return alignment_dot, alignment_angle_deg

    def _selected_tool_axis_candidate(self) -> str:
        if self._orientation_targets_payload is not None:
            candidate = self._orientation_targets_payload.get("selected_tool_axis_candidate")
            if candidate is not None:
                return str(candidate)
        orientation_config = self._config.get("orientation_planning", {})
        if isinstance(orientation_config, dict):
            return str(orientation_config.get("selected_tool_axis_candidate", "tool0_+Z"))
        return "tool0_+Z"

    def _insertion_axis_world(self) -> list[float]:
        if self._orientation_targets_payload is not None:
            axis = self._orientation_targets_payload.get("insertion_axis_world")
            if self._is_vector(axis, 3):
                return self._normalize([float(value) for value in axis])
        orientation_config = self._config.get("orientation_planning", {})
        if isinstance(orientation_config, dict):
            axis = orientation_config.get("insertion_axis_world")
            if self._is_vector(axis, 3):
                return self._normalize([float(value) for value in axis])
        axis = self._config.get("insertion_axis_world")
        if self._is_vector(axis, 3):
            return self._normalize([float(value) for value in axis])
        return [0.0, 0.0, -1.0]

    def _safety_guard_active(self) -> bool:
        if self._last_safety_status is not None:
            level = str(self._last_safety_status.get("level", "")).strip()
            return level.startswith("OK")
        if self._last_safety_status_raw is not None:
            return self._last_safety_status_raw.startswith("OK")
        return False

    def _geometry_validity(
        self,
        target_poses: dict[str, dict[str, Any] | None],
    ) -> dict[str, Any]:
        hole_center = target_poses.get("hole_center")
        xy_checks = {}
        for target_name in self.INSERTION_ALIGNED_TARGETS:
            offset = self._xy_distance_between(target_poses.get(target_name), hole_center)
            xy_checks[target_name] = {
                "xy_offset_m": offset,
                "within_2mm": offset is not None and offset <= self.XY_TOLERANCE_M,
            }

        z_order_valid = self._z_order_valid(
            target_poses.get("axis_align_pose"),
            target_poses.get("insertion_touch_pose"),
            target_poses.get("insertion_hold_pose"),
            target_poses.get("final_insertion_pose"),
        )
        geometry_valid = (
            all(check["within_2mm"] for check in xy_checks.values())
            and z_order_valid
        )
        return {
            "cartesian_geometry_valid": geometry_valid,
            "xy_tolerance_m": self.XY_TOLERANCE_M,
            "insertion_aligned_xy_checks": xy_checks,
            "z_order_axis_touch_hold_final_valid": z_order_valid,
            "geometry_source": "ik_feasibility_diagnostics_recomputed_from_tf_or_yaml",
        }

    @staticmethod
    def _motion_block_reason(
        geometry_valid: bool,
        ik_solver_available: bool,
        ik_solution_available: bool,
        orientation_validated: bool,
        safety_guard_active: bool,
    ) -> str:
        reasons = []
        if not geometry_valid:
            reasons.append("cartesian geometry invalid")
        if not ik_solver_available:
            reasons.append("IK not available")
        if not ik_solution_available:
            reasons.append("real IK solutions unavailable for all targets")
        if not orientation_validated:
            reasons.append("tool insertion axis not validated")
        if not safety_guard_active:
            reasons.append("safety guard not active")
        return "; ".join(reasons)

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
    def _xy_distance_between(
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
                for index in (0, 1)
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

    @classmethod
    def _z_order_valid(
        cls,
        axis_align_pose: dict[str, Any] | None,
        insertion_touch_pose: dict[str, Any] | None,
        insertion_hold_pose: dict[str, Any] | None,
        final_insertion_pose: dict[str, Any] | None,
    ) -> bool:
        positions = [
            cls._position(axis_align_pose),
            cls._position(insertion_touch_pose),
            cls._position(insertion_hold_pose),
            cls._position(final_insertion_pose),
        ]
        if any(position is None for position in positions):
            return False
        z_values = [position[2] for position in positions if position is not None]
        return z_values[0] > z_values[1] > z_values[2] > z_values[3]

    @staticmethod
    def _position(pose: dict[str, Any] | None) -> list[float] | None:
        if pose is None:
            return None
        position = pose.get("position_xyz")
        if not isinstance(position, list) or len(position) != 3:
            return None
        return [float(value) for value in position]

    @staticmethod
    def _rotate_vector(quaternion_xyzw: list[float], vector: list[float]) -> list[float]:
        qx, qy, qz, qw = quaternion_xyzw
        vx, vy, vz = vector
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return [
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        ]

    @staticmethod
    def _dot(first: list[float], second: list[float]) -> float:
        return sum(first[index] * second[index] for index in range(3))

    @staticmethod
    def _normalize(vector: list[float]) -> list[float]:
        norm = math.sqrt(sum(value * value for value in vector))
        if norm == 0.0:
            return [0.0 for _ in vector]
        return [value / norm for value in vector]

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
