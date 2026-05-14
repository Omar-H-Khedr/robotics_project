"""Wait for the KUKA trajectory controller action server before motion starts."""

import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node


class ControllerReadinessGate(Node):
    """One-shot readiness gate for the joint trajectory controller."""

    DEFAULT_ACTION_SERVER = "/joint_trajectory_controller/follow_joint_trajectory"
    DEFAULT_TIMEOUT_SEC = 60.0
    WAIT_PERIOD_SEC = 0.5

    def __init__(self) -> None:
        super().__init__("controller_readiness_gate")
        self.declare_parameter("action_server", self.DEFAULT_ACTION_SERVER)
        self.declare_parameter("timeout_sec", self.DEFAULT_TIMEOUT_SEC)

        self._action_server = (
            self.get_parameter("action_server").get_parameter_value().string_value
        )
        self._timeout_sec = (
            self.get_parameter("timeout_sec").get_parameter_value().double_value
        )
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._action_server,
        )

    def wait_until_ready(self) -> bool:
        """Return True once the trajectory action server is available."""
        if self._timeout_sec <= 0.0:
            self.get_logger().error(
                f"Controller readiness timeout must be positive; got "
                f"{self._timeout_sec:.2f}s."
            )
            return False

        self.get_logger().info(
            f"Waiting for controller action server {self._action_server} "
            f"for up to {self._timeout_sec:.1f}s."
        )

        deadline = time.monotonic() + self._timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._action_client.wait_for_server(timeout_sec=self.WAIT_PERIOD_SEC):
                self.get_logger().info(
                    f"Action server available: {self._action_server}"
                )
                self._log_controller_manager_service_state()
                self.get_logger().info("Controller readiness confirmed.")
                return True

        self.get_logger().error(
            f"Timed out after {self._timeout_sec:.1f}s waiting for controller "
            f"action server {self._action_server}."
        )
        return False

    def _log_controller_manager_service_state(self) -> None:
        service_names_and_types = dict(self.get_service_names_and_types())
        service_types = service_names_and_types.get(
            "/controller_manager/list_controllers"
        )
        if service_types:
            self.get_logger().info(
                "/controller_manager/list_controllers service is visible."
            )
        else:
            self.get_logger().warn(
                "/controller_manager/list_controllers service is not visible; "
                "continuing because the trajectory action server is ready."
            )


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = ControllerReadinessGate()
    try:
        return 0 if node.wait_until_ready() else 1
    except KeyboardInterrupt:
        node.get_logger().warn("Controller readiness gate interrupted.")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
