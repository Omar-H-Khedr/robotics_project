"""Audit a diagnostic move_group runtime without calling IK or motion APIs."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MoveGroupRuntimeAudit(Node):
    """Publish whether move_group and /compute_ik are visible in no-motion mode."""

    TOPIC = "/move_group_runtime_audit"

    def __init__(self) -> None:
        super().__init__("move_group_runtime_audit")
        self.declare_parameter("publish_period_sec", 1.0)

        self._publisher = self.create_publisher(String, self.TOPIC, 10)
        self.create_timer(
            float(self.get_parameter("publish_period_sec").value),
            self._publish_audit,
        )
        self.get_logger().info(
            "move_group runtime audit started; it will not call /compute_ik."
        )

    def _publish_audit(self) -> None:
        services = self._service_report()
        move_group_node_detected = self._move_group_node_detected()
        compute_ik_service_available = bool(services["compute_ik_service_available"])

        if not move_group_node_detected:
            status = "move_group_not_launched_diagnostic_only"
            recommended_next_step = "launch_move_group_diagnostic_only"
        elif compute_ik_service_available:
            status = "move_group_detected_compute_ik_available_no_motion"
            recommended_next_step = "test_compute_ik_service_no_motion"
        else:
            status = "move_group_detected_compute_ik_missing"
            recommended_next_step = "missing_compute_ik_service_from_move_group"

        payload: dict[str, Any] = {
            "status": status,
            "move_group_node_detected": move_group_node_detected,
            "compute_ik_service_available": compute_ik_service_available,
            "compute_ik_services": services["compute_ik_services"],
            "trajectory_execution_disabled_expected": True,
            "controller_motion_allowed": False,
            "trajectory_execution_allowed": False,
            "compute_ik_test_allowed": False,
            "motion_execution_allowed": False,
            "recommended_next_step": recommended_next_step,
        }

        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publisher.publish(message)
        self.get_logger().info(message.data)

    def _move_group_node_detected(self) -> bool:
        return any(
            name.rsplit("/", 1)[-1] == "move_group"
            for name in self.get_node_names()
        )

    def _service_report(self) -> dict[str, Any]:
        compute_ik_services = []
        compute_ik_service_available = False

        for service_name, service_types in self.get_service_names_and_types():
            service_types_list = list(service_types)
            entry = {"name": service_name, "types": service_types_list}
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
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MoveGroupRuntimeAudit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
