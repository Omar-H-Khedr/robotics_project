"""Unified diagnostic-only execution gate monitor for Cartesian insertion."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExecutionGateMonitor(Node):
    """Publish the combined no-motion execution gate state."""

    STATUS_TOPIC = "/execution_gate_status"
    PLANNED_WAYPOINTS = (
        "staging_pose",
        "axis_align_pose",
        "insertion_touch_pose",
        "insertion_hold_pose",
        "final_insertion_pose",
        "retreat_pose",
    )

    def __init__(self) -> None:
        super().__init__("execution_gate_monitor")
        self.declare_parameter("publish_period_sec", 1.0)

        self._cartesian_payload: dict[str, Any] | None = None
        self._ik_payload: dict[str, Any] | None = None
        self._tool_axis_payload: dict[str, Any] | None = None
        self._orientation_targets_payload: dict[str, Any] | None = None
        self._dry_run_plan_payload: dict[str, Any] | None = None
        self._safety_payload: dict[str, Any] | None = None
        self._safety_raw: str | None = None
        self._metrics_payload: dict[str, Any] | None = None

        self._publisher = self.create_publisher(String, self.STATUS_TOPIC, 10)
        self.create_subscription(
            String,
            "/cartesian_insertion_diagnostics",
            self._on_cartesian_diagnostics,
            10,
        )
        self.create_subscription(
            String,
            "/ik_feasibility_diagnostics",
            self._on_ik_diagnostics,
            10,
        )
        self.create_subscription(String, "/tool_axis_audit", self._on_tool_axis_audit, 10)
        self.create_subscription(
            String,
            "/cartesian_orientation_targets",
            self._on_orientation_targets,
            10,
        )
        self.create_subscription(
            String,
            "/cartesian_insertion_dry_run_plan",
            self._on_dry_run_plan,
            10,
        )
        self.create_subscription(String, "/safety_status", self._on_safety_status, 10)
        self.create_subscription(
            String,
            "/insertion_metrics",
            self._on_insertion_metrics,
            10,
        )
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_status,
        )

        self.get_logger().info(
            "Execution gate monitor started in diagnostic_only_no_motion mode."
        )

    def _on_cartesian_diagnostics(self, message: String) -> None:
        self._cartesian_payload = self._parse_json(message.data)

    def _on_ik_diagnostics(self, message: String) -> None:
        self._ik_payload = self._parse_json(message.data)

    def _on_tool_axis_audit(self, message: String) -> None:
        self._tool_axis_payload = self._parse_json(message.data)

    def _on_orientation_targets(self, message: String) -> None:
        self._orientation_targets_payload = self._parse_json(message.data)

    def _on_dry_run_plan(self, message: String) -> None:
        self._dry_run_plan_payload = self._parse_json(message.data)

    def _on_safety_status(self, message: String) -> None:
        self._safety_raw = message.data.strip()
        self._safety_payload = self._parse_json(message.data)

    def _on_insertion_metrics(self, message: String) -> None:
        self._metrics_payload = self._parse_json(message.data)

    def _publish_status(self) -> None:
        geometry_valid = self._geometry_valid()
        ik_available = self._ik_available()
        ik_solution_available = self._ik_solution_available()
        all_targets_geometrically_feasible = self._all_targets_geometrically_feasible()
        tool_axis_orientation_validated = self._tool_axis_orientation_validated()
        orientation_targets_available = self._orientation_targets_available()
        orientation_aware_ik_checked = self._orientation_aware_ik_checked()
        full_pose_targets_available = self._full_pose_targets_available()
        selected_tool_axis_candidate = self._selected_tool_axis_candidate()
        orientation_validated = self._orientation_validated()
        safety_guard_active = self._safety_guard_active()
        contact_metrics_available = self._contact_metrics_available()
        force_guard_active = self._force_guard_active()
        dry_run_plan_available = self._dry_run_plan_available()
        dry_run_plan_executable = self._dry_run_plan_executable()
        dry_run_primary_block_reason = self._dry_run_primary_block_reason()

        execution_conditions_met = (
            geometry_valid
            and ik_available
            and ik_solution_available
            and dry_run_plan_executable
            and tool_axis_orientation_validated
            and safety_guard_active
            and force_guard_active
        )
        controller_execution_allowed = False
        block_reasons = self._block_reasons(
            geometry_valid=geometry_valid,
            ik_available=ik_available,
            ik_solution_available=ik_solution_available,
            tool_axis_orientation_validated=tool_axis_orientation_validated,
            safety_guard_active=safety_guard_active,
            force_guard_active=force_guard_active,
            dry_run_plan_available=dry_run_plan_available,
            dry_run_plan_executable=dry_run_plan_executable,
        )
        if execution_conditions_met:
            block_reasons.append("diagnostic-only launch keeps controller execution disabled")

        payload = {
            "status": "execution_gates_diagnostic_only_no_motion",
            "motion_execution_enabled": False,
            "trajectory_execution_requested": False,
            "controller_execution_allowed": controller_execution_allowed,
            "geometry_valid": geometry_valid,
            "geometry_source": self._geometry_source(),
            "ik_available": ik_available,
            "ik_solution_available": ik_solution_available,
            "all_targets_geometrically_feasible": all_targets_geometrically_feasible,
            "orientation_targets_available": orientation_targets_available,
            "orientation_aware_ik_checked": orientation_aware_ik_checked,
            "full_pose_targets_available": full_pose_targets_available,
            "dry_run_plan_available": dry_run_plan_available,
            "dry_run_plan_executable": dry_run_plan_executable,
            "dry_run_primary_block_reason": dry_run_primary_block_reason,
            "selected_tool_axis_candidate": selected_tool_axis_candidate,
            "orientation_validated": orientation_validated,
            "tool_axis_orientation_validated": tool_axis_orientation_validated,
            "safety_guard_active": safety_guard_active,
            "force_guard_active": force_guard_active,
            "contact_metrics_available": contact_metrics_available,
            "block_reasons": block_reasons,
            "primary_block_reason": block_reasons[0] if block_reasons else "",
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _geometry_valid(self) -> bool:
        if self._cartesian_payload is None:
            return False
        return bool(self._cartesian_payload.get("cartesian_geometry_valid", False))

    def _geometry_source(self) -> str:
        if self._cartesian_payload is None:
            return "cartesian_insertion_diagnostics_unobserved"
        return "cartesian_insertion_diagnostics.cartesian_geometry_valid"

    def _ik_available(self) -> bool:
        if self._ik_payload is None:
            return False
        return bool(self._ik_payload.get("ik_solver_available", False))

    def _ik_solution_available(self) -> bool:
        if self._ik_payload is None:
            return False
        if self._ik_payload.get("real_ik_solution_available") is True:
            targets = self._ik_payload.get("targets", {})
            if not isinstance(targets, dict) or not targets:
                return False
            return all(
                isinstance(target, dict)
                and target.get("ik_solution_available") is True
                for target in targets.values()
            )
        return False

    def _all_targets_geometrically_feasible(self) -> bool:
        if self._ik_payload is None:
            return False
        return bool(self._ik_payload.get("all_targets_geometrically_feasible", False))

    def _tool_axis_orientation_validated(self) -> bool:
        return self._orientation_validated()

    def _orientation_targets_available(self) -> bool:
        if self._orientation_targets_payload is None:
            return False
        return bool(
            self._orientation_targets_payload.get(
                "orientation_targets_available",
                False,
            )
        )

    def _orientation_aware_ik_checked(self) -> bool:
        if self._ik_payload is None:
            return False
        return bool(self._ik_payload.get("orientation_aware_ik_checked", False))

    def _full_pose_targets_available(self) -> bool:
        if self._ik_payload is None:
            return False
        targets = self._ik_payload.get("targets", {})
        if not isinstance(targets, dict):
            return False
        for waypoint in self.PLANNED_WAYPOINTS:
            target = targets.get(waypoint)
            if not isinstance(target, dict):
                return False
            if target.get("target_position_world") is None:
                return False
            if target.get("target_orientation_world") is None:
                return False
            if target.get("orientation_target_available") is not True:
                return False
        return True

    def _selected_tool_axis_candidate(self) -> str | None:
        if self._orientation_targets_payload is not None:
            candidate = self._orientation_targets_payload.get(
                "selected_tool_axis_candidate"
            )
            if candidate is not None:
                return str(candidate)
        if self._tool_axis_payload is not None:
            candidate = self._tool_axis_payload.get(
                "recommended_selected_tool_axis_candidate"
            )
            if candidate is not None:
                return str(candidate)
        return None

    def _orientation_validated(self) -> bool:
        if self._orientation_targets_payload is not None:
            if self._orientation_targets_payload.get("orientation_validated") is True:
                return True
        if self._tool_axis_payload is not None:
            return bool(self._tool_axis_payload.get("orientation_validated", False))
        return False

    def _safety_guard_active(self) -> bool:
        if self._safety_payload is not None:
            level = str(self._safety_payload.get("level", "")).strip()
            return level.startswith("OK")
        if self._safety_raw is not None:
            return self._safety_raw.startswith("OK")
        return False

    def _force_guard_active(self) -> bool:
        if self._metrics_payload is None:
            return False
        return bool(self._metrics_payload.get("force_guard_active", False))

    def _contact_metrics_available(self) -> bool:
        if self._metrics_payload is None:
            return False
        return bool(self._metrics_payload.get("contact_metrics_available", False))

    def _dry_run_plan_available(self) -> bool:
        return self._dry_run_plan_payload is not None

    def _dry_run_plan_executable(self) -> bool:
        if self._dry_run_plan_payload is None:
            return False
        return bool(self._dry_run_plan_payload.get("plan_executable", False))

    def _dry_run_primary_block_reason(self) -> str | None:
        if self._dry_run_plan_payload is None:
            return None
        reason = self._dry_run_plan_payload.get("primary_block_reason")
        return str(reason) if reason is not None else None

    @staticmethod
    def _block_reasons(
        *,
        geometry_valid: bool,
        ik_available: bool,
        ik_solution_available: bool,
        tool_axis_orientation_validated: bool,
        safety_guard_active: bool,
        force_guard_active: bool,
        dry_run_plan_available: bool,
        dry_run_plan_executable: bool,
    ) -> list[str]:
        reasons = []
        if not geometry_valid:
            reasons.append("cartesian geometry invalid or unavailable")
        if not ik_available:
            reasons.append("IK solver not available")
        if not ik_solution_available:
            reasons.append("real IK solutions unavailable for all targets")
        if not dry_run_plan_available:
            reasons.append("Cartesian dry-run plan unavailable")
        elif not dry_run_plan_executable:
            reasons.append("Cartesian dry-run plan not executable")
        if not tool_axis_orientation_validated:
            reasons.append("tool insertion axis orientation not validated")
        if not safety_guard_active:
            reasons.append("safety guard not active or unobserved")
        if not force_guard_active:
            reasons.append("force/contact guard not active")
        return reasons

    @staticmethod
    def _parse_json(data: str) -> dict[str, Any] | None:
        try:
            parsed = json.loads(data)
        except json.JSONDecodeError:
            return None
        return parsed if isinstance(parsed, dict) else None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExecutionGateMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
