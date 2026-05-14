"""Sequential FollowJointTrajectory executor for the KUKA task baseline."""

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
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


class TaskTrajectoryExecutor(Node):
    """Execute named KUKA task poses through the ros2_control action interface."""

    JOINT_NAMES = (
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    )
    POSE_ORDER = (
        "safe_home",
        "observe_scene",
        "pre_grasp",
        "grasp_approach",
        "lift_clearance",
        "pre_insert",
        "insertion_approach",
        "insertion_hold",
        "retreat",
        "return_home",
    )
    ACTION_SERVER = "/joint_trajectory_controller/follow_joint_trajectory"
    TASK_PHASE_TOPIC = "/task_phase"
    START_DELAY_SEC = 0.25

    def __init__(self) -> None:
        super().__init__("task_trajectory_executor")
        self.declare_parameter("config_path", "")
        self.declare_parameter("action_server", self.ACTION_SERVER)
        self.declare_parameter("task_phase_topic", self.TASK_PHASE_TOPIC)

        self._config_path = Path(
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        self._action_server = (
            self.get_parameter("action_server").get_parameter_value().string_value
        )
        self._task_phase_topic = (
            self.get_parameter("task_phase_topic").get_parameter_value().string_value
        )
        self._config = self._load_config(self._config_path)
        self._poses = self._validate_and_get_poses(self._config)
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._action_server,
        )
        self._phase_publisher = self.create_publisher(String, self._task_phase_topic, 10)

        metadata = self._config.get("metadata", {})
        if not isinstance(metadata, dict):
            metadata = {}
        experiment_name = metadata.get("experiment_name", "unnamed_task_sequence")
        self.get_logger().info(
            f"Loaded task trajectory '{experiment_name}' from {self._config_path}"
        )
        self.get_logger().info(
            f"Using FollowJointTrajectory action server {self._action_server}"
        )
        self.get_logger().info(f"Publishing task phase on {self._task_phase_topic}")

    def execute(self) -> bool:
        """Run the configured named poses in order, stopping at the first failure."""
        self.get_logger().info("Waiting for FollowJointTrajectory action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server is available.")
        self._publish_phase("sequence_start")

        for index, pose_name in enumerate(self.POSE_ORDER, start=1):
            pose = self._poses[pose_name]
            description = pose["description"]
            safety_tag = pose["safety_tag"]
            self.get_logger().info(
                f"Executing pose {index}/{len(self.POSE_ORDER)} '{pose_name}': "
                f"{description} [safety_tag={safety_tag}]"
            )
            self._publish_phase(pose_name)

            if not self._execute_pose(pose_name, pose):
                self.get_logger().error(
                    f"Stopping task sequence after failed pose '{pose_name}'."
                )
                self._publish_phase(f"failed:{pose_name}")
                return False

        self.get_logger().info("Task pose sequence completed successfully.")
        self._publish_phase("sequence_complete")
        return True

    def _execute_pose(self, pose_name: str, pose: dict[str, Any]) -> bool:
        goal_msg = self._build_goal(pose)

        self.get_logger().info(
            f"Sending FollowJointTrajectory goal for '{pose_name}' "
            f"with duration {float(pose['duration_sec']):.2f}s."
        )
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error(
                f"Goal request for pose '{pose_name}' failed before reaching "
                "the controller."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                f"FollowJointTrajectory goal for pose '{pose_name}' was rejected."
            )
            return False

        self.get_logger().info(
            f"Pose '{pose_name}' goal accepted; waiting for controller result."
        )
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                f"Pose '{pose_name}' finished without an action result payload."
            )
            return False

        result = wrapped_result.result
        status_name = self._goal_status_name(wrapped_result.status)
        result_name = self._result_error_name(result.error_code)

        if (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().info(
                f"Pose '{pose_name}' succeeded: status={status_name} "
                f"({wrapped_result.status}), result={result_name} "
                f"({result.error_code})."
            )
            return True

        self.get_logger().error(
            f"Pose '{pose_name}' failed: status={status_name} "
            f"({wrapped_result.status}), result={result_name} "
            f"({result.error_code}) {result.error_string}"
        )
        return False

    def _build_goal(self, pose: dict[str, Any]) -> FollowJointTrajectory.Goal:
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(self.JOINT_NAMES)
        goal_msg.trajectory.header.stamp = (
            self.get_clock().now() + RclpyDuration(seconds=self.START_DELAY_SEC)
        ).to_msg()

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in pose["positions"]]
        point.time_from_start = self._seconds_to_duration(float(pose["duration_sec"]))
        goal_msg.trajectory.points.append(point)
        return goal_msg

    def _publish_phase(self, phase: str) -> None:
        message = String()
        message.data = phase
        self._phase_publisher.publish(message)
        rclpy.spin_once(self, timeout_sec=0.05)

    @classmethod
    def _validate_and_get_poses(cls, config: dict[str, Any]) -> dict[str, dict[str, Any]]:
        poses = config.get("poses")
        if not isinstance(poses, dict):
            raise ValueError("Baseline task pose config must contain a 'poses' mapping.")

        missing = [pose_name for pose_name in cls.POSE_ORDER if pose_name not in poses]
        if missing:
            raise ValueError(f"Missing required task poses: {missing}")

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

            description = pose.get("description")
            if not isinstance(description, str) or not description.strip():
                raise ValueError(
                    f"Pose '{pose_name}' requires a non-empty description."
                )

            safety_tag = pose.get("safety_tag")
            if not isinstance(safety_tag, str) or not safety_tag.strip():
                raise ValueError(
                    f"Pose '{pose_name}' requires a non-empty safety_tag."
                )

        return poses

    @staticmethod
    def _load_config(config_path: Path) -> dict[str, Any]:
        if str(config_path) == ".":
            raise ValueError("Parameter 'config_path' must point to a YAML config file.")
        if not config_path.is_file():
            raise FileNotFoundError(f"Task pose config does not exist: {config_path}")

        with config_path.open("r", encoding="utf-8") as config_file:
            loaded = yaml.safe_load(config_file)

        if not isinstance(loaded, dict):
            raise ValueError(f"Task pose config must be a YAML mapping: {config_path}")
        return loaded

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
    node: TaskTrajectoryExecutor | None = None
    exit_code = 0

    try:
        node = TaskTrajectoryExecutor()
        if not node.execute():
            exit_code = 1
    except Exception as exc:  # noqa: BLE001 - top-level node failure logging.
        if node is not None:
            node.get_logger().error(f"Task trajectory executor failed: {exc}")
        else:
            print(f"Task trajectory executor failed during startup: {exc}")
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
