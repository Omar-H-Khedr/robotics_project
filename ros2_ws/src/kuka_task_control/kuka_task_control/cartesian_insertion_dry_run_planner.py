"""Diagnostic-only Cartesian insertion dry-run plan publisher."""

from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class CartesianInsertionDryRunPlanner(Node):
    """Assemble Cartesian insertion waypoints without commanding motion."""

    PLAN_TOPIC = "/cartesian_insertion_dry_run_plan"
    WAYPOINT_ORDER = (
        "current_tool_pose",
        "staging_pose",
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
        "retreat_pose",
    )
    PLANNED_WAYPOINTS = WAYPOINT_ORDER[1:]

    def __init__(self) -> None:
        super().__init__("cartesian_insertion_dry_run_planner")
        self.declare_parameter("publish_period_sec", 1.0)

        self._cartesian_payload: dict[str, Any] | None = None
        self._orientation_payload: dict[str, Any] | None = None
        self._ik_payload: dict[str, Any] | None = None
        self._execution_gate_payload: dict[str, Any] | None = None
        self._current_joint_names: list[str] = []
        self._current_joint_positions: list[float] = []

        self._publisher = self.create_publisher(String, self.PLAN_TOPIC, 10)
        self.create_subscription(
            String,
            "/cartesian_insertion_diagnostics",
            self._on_cartesian_diagnostics,
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
            "/ik_feasibility_diagnostics",
            self._on_ik_diagnostics,
            10,
        )
        self.create_subscription(
            String,
            "/execution_gate_status",
            self._on_execution_gate_status,
            10,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_plan,
        )

        self.get_logger().info(
            "Cartesian insertion dry-run planner started in diagnostic-only "
            "no-motion mode."
        )

    def _on_cartesian_diagnostics(self, message: String) -> None:
        self._cartesian_payload = self._parse_json(message.data)

    def _on_orientation_targets(self, message: String) -> None:
        self._orientation_payload = self._parse_json(message.data)

    def _on_ik_diagnostics(self, message: String) -> None:
        self._ik_payload = self._parse_json(message.data)

    def _on_execution_gate_status(self, message: String) -> None:
        self._execution_gate_payload = self._parse_json(message.data)

    def _on_joint_states(self, message: JointState) -> None:
        self._current_joint_names = list(message.name)
        self._current_joint_positions = [float(value) for value in message.position]

    def _publish_plan(self) -> None:
        current_tool_pose = self._current_tool_pose_world()
        waypoints = []
        previous_pose = None
        for waypoint_name in self.WAYPOINT_ORDER:
            waypoint = self._waypoint(waypoint_name, current_tool_pose, previous_pose)
            waypoints.append(waypoint)
            previous_pose = waypoint["pose_world"]

        all_waypoints_have_full_pose = all(
            waypoint["pose_world"] is not None
            and waypoint["position_xyz"] is not None
            and waypoint["orientation_xyzw"] is not None
            for waypoint in waypoints
        )
        all_waypoints_geometrically_feasible = self._all_waypoints_geometrically_feasible(
            waypoints
        )
        all_waypoints_have_ik_solution = all(
            waypoint["waypoint_executable"] is True
            for waypoint in waypoints
            if waypoint["waypoint_name"] in self.PLANNED_WAYPOINTS
        )
        plan_executable = (
            all_waypoints_have_full_pose
            and all_waypoints_geometrically_feasible
            and all_waypoints_have_ik_solution
        )
        block_reasons = self._block_reasons(
            all_waypoints_have_full_pose=all_waypoints_have_full_pose,
            all_waypoints_geometrically_feasible=all_waypoints_geometrically_feasible,
            all_waypoints_have_ik_solution=all_waypoints_have_ik_solution,
        )

        payload = {
            "status": "cartesian_dry_run_no_motion",
            "motion_execution_enabled": False,
            "trajectory_execution_requested": False,
            "controller_execution_allowed": False,
            "current_joint_names": self._current_joint_names,
            "current_joint_positions": self._current_joint_positions,
            "waypoint_order": list(self.WAYPOINT_ORDER),
            "waypoints": waypoints,
            "all_waypoints_have_full_pose": all_waypoints_have_full_pose,
            "all_waypoints_geometrically_feasible": all_waypoints_geometrically_feasible,
            "all_waypoints_have_ik_solution": all_waypoints_have_ik_solution,
            "plan_executable": plan_executable,
            "block_reasons": block_reasons,
            "primary_block_reason": block_reasons[0] if block_reasons else "",
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _waypoint(
        self,
        waypoint_name: str,
        current_tool_pose: dict[str, Any] | None,
        previous_pose: dict[str, Any] | None,
    ) -> dict[str, Any]:
        if waypoint_name == "current_tool_pose":
            pose_world = current_tool_pose
            position_source = self._pose_source(pose_world, "current_tool_pose")
            orientation = self._orientation(pose_world)
            orientation_source = "current_tool_pose"
            approximate_workspace_feasible = pose_world is not None
            orientation_target_available = orientation is not None
            ik_solver_available = self._ik_solver_available()
            ik_solution_available = False
            joint_solution = None
        else:
            ik_target = self._ik_target(waypoint_name)
            pose_world = self._planned_pose_world(waypoint_name, ik_target)
            position_source = self._planned_position_source(waypoint_name, ik_target)
            orientation = self._orientation(pose_world)
            orientation_source = self._planned_orientation_source(waypoint_name, ik_target)
            approximate_workspace_feasible = self._approximate_workspace_feasible(
                waypoint_name,
                ik_target,
            )
            orientation_target_available = orientation is not None
            ik_solver_available = self._ik_solver_available()
            ik_solution_available = self._real_ik_solution_available(ik_target)
            joint_solution = self._joint_solution(ik_target, ik_solution_available)

        return {
            "waypoint_name": waypoint_name,
            "pose_world": pose_world,
            "position_xyz": self._position(pose_world),
            "orientation_xyzw": orientation,
            "position_source": position_source,
            "orientation_source": orientation_source,
            "distance_from_previous_waypoint": self._distance_between(
                previous_pose,
                pose_world,
            ),
            "distance_from_current_tool": self._distance_between(
                current_tool_pose,
                pose_world,
            ),
            "approximate_workspace_feasible": approximate_workspace_feasible,
            "orientation_target_available": orientation_target_available,
            "ik_solver_available": ik_solver_available,
            "ik_solution_available": ik_solution_available,
            "joint_solution": joint_solution,
            "waypoint_executable": ik_solution_available is True
            and joint_solution is not None,
        }

    def _current_tool_pose_world(self) -> dict[str, Any] | None:
        if self._cartesian_payload is not None:
            pose = self._cartesian_payload.get("current_tool_pose_world")
            if isinstance(pose, dict):
                return pose
        if self._ik_payload is not None:
            pose = self._ik_payload.get("current_tool_pose_world")
            if isinstance(pose, dict):
                return pose
        return None

    def _planned_pose_world(
        self,
        waypoint_name: str,
        ik_target: dict[str, Any] | None,
    ) -> dict[str, Any] | None:
        if ik_target is not None:
            full_pose = ik_target.get("full_target_pose_world")
            if isinstance(full_pose, dict):
                return full_pose
            target_pose = ik_target.get("target_pose_world")
            if isinstance(target_pose, dict):
                position = self._position(target_pose)
                orientation = self._orientation_from_orientation_targets(waypoint_name)
                if position is not None and orientation is not None:
                    return {
                        "frame": target_pose.get("frame", "world"),
                        "child_frame": waypoint_name,
                        "position_xyz": position,
                        "orientation_xyzw": orientation,
                        "position_source": target_pose.get("frame_source")
                        or ik_target.get("target_pose_world_source")
                        or "tf_or_yaml_fallback",
                        "orientation_source": "cartesian_orientation_targets",
                    }

        if self._cartesian_payload is None:
            return None
        pose = self._cartesian_payload.get(f"{waypoint_name}_world")
        if not isinstance(pose, dict):
            return None
        position = self._position(pose)
        orientation = self._orientation_from_orientation_targets(waypoint_name)
        if position is None or orientation is None:
            return None
        return {
            "frame": pose.get("frame", "world"),
            "child_frame": waypoint_name,
            "position_xyz": position,
            "orientation_xyzw": orientation,
            "position_source": pose.get("frame_source")
            or self._cartesian_frame_source(waypoint_name)
            or "tf_or_yaml_fallback",
            "orientation_source": "cartesian_orientation_targets",
        }

    def _ik_target(self, waypoint_name: str) -> dict[str, Any] | None:
        if self._ik_payload is None:
            return None
        targets = self._ik_payload.get("targets", {})
        if not isinstance(targets, dict):
            return None
        target = targets.get(waypoint_name)
        return target if isinstance(target, dict) else None

    def _planned_position_source(
        self,
        waypoint_name: str,
        ik_target: dict[str, Any] | None,
    ) -> str | None:
        if ik_target is not None:
            source = ik_target.get("target_pose_world_source")
            if source is not None:
                return str(source)
            full_pose = ik_target.get("full_target_pose_world")
            if isinstance(full_pose, dict) and full_pose.get("position_source") is not None:
                return str(full_pose.get("position_source"))
        return self._cartesian_frame_source(waypoint_name)

    def _planned_orientation_source(
        self,
        waypoint_name: str,
        ik_target: dict[str, Any] | None,
    ) -> str | None:
        if ik_target is not None and ik_target.get("orientation_source") is not None:
            return str(ik_target.get("orientation_source"))
        desired = self._orientation_target_entry(waypoint_name)
        if desired is not None and desired.get("orientation_source") is not None:
            return str(desired.get("orientation_source"))
        if self._orientation_from_orientation_targets(waypoint_name) is not None:
            return "cartesian_orientation_targets"
        return None

    def _cartesian_frame_source(self, waypoint_name: str) -> str | None:
        if self._cartesian_payload is None:
            return None
        sources = self._cartesian_payload.get("frame_source", {})
        if isinstance(sources, dict) and sources.get(waypoint_name) is not None:
            return str(sources.get(waypoint_name))
        return None

    def _orientation_from_orientation_targets(
        self,
        waypoint_name: str,
    ) -> list[float] | None:
        desired = self._orientation_target_entry(waypoint_name)
        if desired is None:
            return None
        orientation = desired.get("orientation_xyzw")
        if self._is_vector(orientation, 4):
            return [float(value) for value in orientation]
        return None

    def _orientation_target_entry(self, waypoint_name: str) -> dict[str, Any] | None:
        if self._orientation_payload is None:
            return None
        desired = self._orientation_payload.get("desired_orientations_world", {})
        if not isinstance(desired, dict):
            return None
        target = desired.get(waypoint_name)
        return target if isinstance(target, dict) else None

    def _approximate_workspace_feasible(
        self,
        waypoint_name: str,
        ik_target: dict[str, Any] | None,
    ) -> bool | None:
        if ik_target is not None:
            value = ik_target.get("approximate_workspace_feasible")
            if isinstance(value, bool):
                return value
        if self._cartesian_payload is None:
            return None
        geometry = self._cartesian_payload.get("cartesian_geometry_valid")
        if waypoint_name in self.PLANNED_WAYPOINTS and isinstance(geometry, bool):
            return geometry
        return None

    def _all_waypoints_geometrically_feasible(
        self,
        waypoints: list[dict[str, Any]],
    ) -> bool:
        if self._cartesian_payload is not None:
            if self._cartesian_payload.get("cartesian_geometry_valid") is not True:
                return False
        planned = [
            waypoint
            for waypoint in waypoints
            if waypoint["waypoint_name"] in self.PLANNED_WAYPOINTS
        ]
        return all(
            waypoint["approximate_workspace_feasible"] is True
            for waypoint in planned
        )

    def _ik_solver_available(self) -> bool:
        if self._ik_payload is None:
            return False
        return bool(self._ik_payload.get("ik_solver_available", False))

    @staticmethod
    def _real_ik_solution_available(ik_target: dict[str, Any] | None) -> bool:
        if ik_target is None:
            return False
        joint_solution = ik_target.get("joint_solution")
        return ik_target.get("ik_solution_available") is True and isinstance(
            joint_solution,
            (dict, list),
        ) and bool(joint_solution)

    @staticmethod
    def _joint_solution(
        ik_target: dict[str, Any] | None,
        ik_solution_available: bool,
    ) -> Any | None:
        if not ik_solution_available or ik_target is None:
            return None
        joint_solution = ik_target.get("joint_solution")
        if isinstance(joint_solution, (dict, list)):
            return joint_solution
        return None

    def _block_reasons(
        self,
        *,
        all_waypoints_have_full_pose: bool,
        all_waypoints_geometrically_feasible: bool,
        all_waypoints_have_ik_solution: bool,
    ) -> list[str]:
        reasons = []
        if not self._ik_solver_available():
            reasons.append("IK solver not available")
        if not all_waypoints_have_ik_solution:
            reasons.append("real IK solutions unavailable for all waypoints")
        if not all_waypoints_have_full_pose:
            reasons.append("full Cartesian waypoint poses unavailable")
        if not all_waypoints_geometrically_feasible:
            reasons.append("Cartesian waypoint geometry invalid or unavailable")
        if self._execution_gate_payload is not None:
            gate_allows_controller = bool(
                self._execution_gate_payload.get("controller_execution_allowed", False)
            )
            if not gate_allows_controller:
                reasons.append("execution gate keeps controller execution disabled")
        return reasons

    @staticmethod
    def _pose_source(pose: dict[str, Any] | None, fallback: str) -> str | None:
        if pose is None:
            return None
        source = pose.get("frame_source") or pose.get("position_source")
        return str(source) if source is not None else fallback

    @staticmethod
    def _position(pose: dict[str, Any] | None) -> list[float] | None:
        if pose is None:
            return None
        position = pose.get("position_xyz")
        if not CartesianInsertionDryRunPlanner._is_vector(position, 3):
            return None
        return [float(value) for value in position]

    @staticmethod
    def _orientation(pose: dict[str, Any] | None) -> list[float] | None:
        if pose is None:
            return None
        orientation = pose.get("orientation_xyzw")
        if not CartesianInsertionDryRunPlanner._is_vector(orientation, 4):
            return None
        return [float(value) for value in orientation]

    @staticmethod
    def _distance_between(
        start_pose: dict[str, Any] | None,
        end_pose: dict[str, Any] | None,
    ) -> float | None:
        start_position = CartesianInsertionDryRunPlanner._position(start_pose)
        end_position = CartesianInsertionDryRunPlanner._position(end_pose)
        if start_position is None or end_position is None:
            return None
        return math.sqrt(
            sum(
                (end_position[index] - start_position[index]) ** 2
                for index in range(3)
            )
        )

    @staticmethod
    def _is_vector(value: Any, length: int) -> bool:
        return isinstance(value, list) and len(value) == length

    @staticmethod
    def _parse_json(data: str) -> dict[str, Any] | None:
        try:
            parsed = json.loads(data)
        except json.JSONDecodeError:
            return None
        return parsed if isinstance(parsed, dict) else None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CartesianInsertionDryRunPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
