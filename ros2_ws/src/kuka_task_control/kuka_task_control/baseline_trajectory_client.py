"""Baseline FollowJointTrajectory action client for KUKA validation runs."""

from pathlib import Path
from typing import Any

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class BaselineTrajectoryClient(Node):
    """Send a configured deterministic KUKA joint trajectory to ros2_control."""

    _POSE_ORDER = ("home", "pre_task", "return_home")
    _TIMING_KEYS = ("home_sec", "pre_task_sec", "return_home_sec")

    def __init__(self) -> None:
        super().__init__("baseline_trajectory_client")
        self.declare_parameter("config_path", "")

        self._config_path = Path(
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        self._config = self._load_config(self._config_path)
        self._validate_config(self._config)

        action_name = self._config["controller_action_name"]
        self._action_client = ActionClient(self, FollowJointTrajectory, action_name)

        metadata = self._config.get("metadata", {})
        experiment_name = metadata.get("experiment_name", "unnamed_experiment")
        self.get_logger().info(
            f"Loaded baseline trajectory '{experiment_name}' from {self._config_path}"
        )
        self.get_logger().info(f"Target action server: {action_name}")

    def execute(self) -> bool:
        """Connect to the controller, send the goal, and wait for completion."""
        self.get_logger().info("Waiting for FollowJointTrajectory action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server is available.")

        goal_msg = self._build_goal()
        point_count = len(goal_msg.trajectory.points)
        joint_count = len(goal_msg.trajectory.joint_names)
        self.get_logger().info(
            f"Sending trajectory goal with {point_count} points for {joint_count} joints."
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error("Goal request failed before reaching the controller.")
            return False

        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected by the controller.")
            return False

        self.get_logger().info("Trajectory goal accepted; waiting for result.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error("Trajectory action finished without a result payload.")
            return False

        result = wrapped_result.result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Baseline trajectory completed successfully.")
            return True

        error_name = self._result_error_name(result.error_code)
        self.get_logger().error(
            f"Baseline trajectory failed: {error_name} "
            f"({result.error_code}) {result.error_string}"
        )
        return False

    def _build_goal(self) -> FollowJointTrajectory.Goal:
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(self._config["joint_names"])

        for pose_name, timing_key in zip(self._POSE_ORDER, self._TIMING_KEYS):
            seconds = int(self._config["timing"][timing_key])
            positions = [float(value) for value in self._config["poses"][pose_name]]

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=seconds, nanosec=0)
            goal_msg.trajectory.points.append(point)

            self.get_logger().info(
                f"Queued '{pose_name}' at t={seconds}s: positions={positions}"
            )

        return goal_msg

    @staticmethod
    def _load_config(config_path: Path) -> dict[str, Any]:
        if str(config_path) == ".":
            raise ValueError("Parameter 'config_path' must point to a YAML config file.")
        if not config_path.is_file():
            raise FileNotFoundError(f"Trajectory config does not exist: {config_path}")

        with config_path.open("r", encoding="utf-8") as config_file:
            loaded = yaml.safe_load(config_file)

        if not isinstance(loaded, dict):
            raise ValueError(f"Trajectory config must be a YAML mapping: {config_path}")
        return loaded

    @classmethod
    def _validate_config(cls, config: dict[str, Any]) -> None:
        cls._require_keys(
            config,
            ("controller_action_name", "joint_names", "poses", "timing", "metadata"),
            "trajectory config",
        )

        if not isinstance(config["controller_action_name"], str):
            raise ValueError("'controller_action_name' must be a string.")

        joint_names = config["joint_names"]
        if not isinstance(joint_names, list) or not all(
            isinstance(name, str) and name for name in joint_names
        ):
            raise ValueError("'joint_names' must be a non-empty list of joint names.")

        poses = config["poses"]
        timing = config["timing"]
        cls._require_keys(poses, cls._POSE_ORDER, "poses")
        cls._require_keys(timing, cls._TIMING_KEYS, "timing")

        expected_width = len(joint_names)
        for pose_name in cls._POSE_ORDER:
            positions = poses[pose_name]
            if not isinstance(positions, list) or len(positions) != expected_width:
                raise ValueError(
                    f"Pose '{pose_name}' must contain {expected_width} joint values."
                )
            for value in positions:
                if not isinstance(value, (float, int)):
                    raise ValueError(f"Pose '{pose_name}' contains a non-numeric value.")

        previous_time = 0
        for timing_key in cls._TIMING_KEYS:
            seconds = timing[timing_key]
            if not isinstance(seconds, int) or seconds <= previous_time:
                raise ValueError(
                    f"Timing '{timing_key}' must be an integer greater than "
                    f"{previous_time}."
                )
            previous_time = seconds

    @staticmethod
    def _require_keys(
        mapping: Any, keys: tuple[str, ...], context: str
    ) -> None:
        if not isinstance(mapping, dict):
            raise ValueError(f"{context} must be a YAML mapping.")

        missing = [key for key in keys if key not in mapping]
        if missing:
            raise ValueError(f"Missing required {context} keys: {missing}")

    @staticmethod
    def _result_error_name(error_code: int) -> str:
        names = {
            FollowJointTrajectory.Result.INVALID_GOAL: "INVALID_GOAL",
            FollowJointTrajectory.Result.INVALID_JOINTS: "INVALID_JOINTS",
            FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
                "PATH_TOLERANCE_VIOLATED",
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
                "GOAL_TOLERANCE_VIOLATED",
        }
        return names.get(error_code, "UNKNOWN_ERROR")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: BaselineTrajectoryClient | None = None
    exit_code = 0

    try:
        node = BaselineTrajectoryClient()
        if not node.execute():
            exit_code = 1
    except Exception as exc:  # noqa: BLE001 - top-level node failure logging.
        if node is not None:
            node.get_logger().error(f"Baseline trajectory client failed: {exc}")
        else:
            print(f"Baseline trajectory client failed during startup: {exc}")
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
