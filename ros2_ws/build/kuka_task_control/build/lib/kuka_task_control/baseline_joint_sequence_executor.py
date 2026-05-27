"""FollowJointTrajectory action executor for the KUKA peg-in-hole baseline."""

from pathlib import Path
from typing import Any

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class BaselineJointSequenceExecutor(Node):
    """Execute a configured KUKA joint-space task sequence through ros2_control."""

    JOINT_NAMES = (
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    )
    POSE_ORDER = (
        "home",
        "safe_above_table",
        "observe_scene",
        "pre_task",
        "approach_workspace",
        "retreat",
    )
    ACTION_SERVER = "/joint_trajectory_controller/follow_joint_trajectory"
    START_DELAY_SEC = 1.0

    def __init__(self) -> None:
        super().__init__("baseline_joint_sequence_executor")
        self.declare_parameter("config_path", "")

        self._config_path = Path(
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        self._poses = self._load_and_validate_poses(self._config_path)
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.ACTION_SERVER,
        )

        self.get_logger().info(
            f"Loaded baseline task poses from {self._config_path}"
        )
        self.get_logger().info(
            f"Using FollowJointTrajectory action server {self.ACTION_SERVER}"
        )

    def execute(self) -> bool:
        """Send the full configured trajectory sequence and wait for completion."""
        self.get_logger().info("Waiting for FollowJointTrajectory action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server is available.")

        goal_msg = self._build_goal()
        self.get_logger().info(
            f"Sending baseline joint sequence goal with "
            f"{len(goal_msg.trajectory.points)} poses."
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error("Goal request failed before reaching the controller.")
            return False

        if not goal_handle.accepted:
            self.get_logger().error("Baseline joint sequence goal was rejected.")
            return False

        self.get_logger().info("Baseline joint sequence goal accepted.")
        self.get_logger().info("Waiting for FollowJointTrajectory result...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error("Trajectory action finished without a result payload.")
            return False

        status_name = self._goal_status_name(wrapped_result.status)
        result = wrapped_result.result
        result_name = self._result_error_name(result.error_code)

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(
                f"Baseline joint sequence completed successfully: "
                f"status={status_name} ({wrapped_result.status}), "
                f"result={result_name} ({result.error_code})."
            )
            return True

        self.get_logger().error(
            f"Baseline joint sequence failed: status={status_name} "
            f"({wrapped_result.status}), result={result_name} "
            f"({result.error_code}) {result.error_string}"
        )
        return False

    def _build_goal(self) -> FollowJointTrajectory.Goal:
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(self.JOINT_NAMES)
        goal_msg.trajectory.header.stamp = (
            self.get_clock().now() + RclpyDuration(seconds=self.START_DELAY_SEC)
        ).to_msg()

        elapsed_sec = 0.0
        for pose_name in self.POSE_ORDER:
            pose = self._poses[pose_name]
            elapsed_sec += float(pose["duration_sec"])

            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in pose["positions"]]
            point.time_from_start = self._seconds_to_duration(elapsed_sec)
            goal_msg.trajectory.points.append(point)

            self.get_logger().info(
                f"Queued pose '{pose_name}' for execution at t={elapsed_sec:.2f}s: "
                f"positions={point.positions}"
            )

        return goal_msg

    @classmethod
    def _load_and_validate_poses(cls, config_path: Path) -> dict[str, dict[str, Any]]:
        if str(config_path) == ".":
            raise ValueError("Parameter 'config_path' must point to a YAML config file.")
        if not config_path.is_file():
            raise FileNotFoundError(
                f"Baseline task pose config does not exist: {config_path}"
            )

        with config_path.open("r", encoding="utf-8") as config_file:
            loaded = yaml.safe_load(config_file)

        if not isinstance(loaded, dict):
            raise ValueError(
                f"Baseline task pose config must be a YAML mapping: {config_path}"
            )

        poses = loaded.get("poses")
        if not isinstance(poses, dict):
            raise ValueError("Baseline task pose config must contain a 'poses' mapping.")

        missing = [pose_name for pose_name in cls.POSE_ORDER if pose_name not in poses]
        if missing:
            raise ValueError(f"Missing required baseline task poses: {missing}")

        for pose_name in cls.POSE_ORDER:
            pose = poses[pose_name]
            if not isinstance(pose, dict):
                raise ValueError(f"Pose '{pose_name}' must be a YAML mapping.")

            positions = pose.get("positions")
            if not isinstance(positions, list) or len(positions) != len(
                cls.JOINT_NAMES
            ):
                raise ValueError(
                    f"Pose '{pose_name}' must contain exactly "
                    f"{len(cls.JOINT_NAMES)} joint values."
                )
            for value in positions:
                if not isinstance(value, (float, int)):
                    raise ValueError(
                        f"Pose '{pose_name}' contains a non-numeric joint value."
                    )

            duration_sec = pose.get("duration_sec")
            if not isinstance(duration_sec, (float, int)) or duration_sec <= 0.0:
                raise ValueError(
                    f"Pose '{pose_name}' requires a positive numeric duration_sec."
                )

        return poses

    @staticmethod
    def _seconds_to_duration(seconds: float) -> Duration:
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
        if nanoseconds == 1_000_000_000:
            whole_seconds += 1
            nanoseconds = 0
        return Duration(sec=whole_seconds, nanosec=nanoseconds)

    @staticmethod
    def _goal_status_name(status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        return names.get(status, "UNRECOGNIZED")

    @staticmethod
    def _result_error_name(error_code: int) -> str:
        names = {
            FollowJointTrajectory.Result.SUCCESSFUL: "SUCCESSFUL",
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
    node: BaselineJointSequenceExecutor | None = None
    exit_code = 0

    try:
        node = BaselineJointSequenceExecutor()
        if not node.execute():
            exit_code = 1
    except Exception as exc:  # noqa: BLE001 - top-level node failure logging.
        if node is not None:
            node.get_logger().error(f"Baseline joint sequence executor failed: {exc}")
        else:
            print(f"Baseline joint sequence executor failed during startup: {exc}")
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
