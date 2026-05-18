"""Segmented guarded FollowJointTrajectory executor for robot contact validation."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


@dataclass(frozen=True)
class Segment:
    name: str
    positions: tuple[float, float, float, float, float, float]
    duration_sec: float
    safety_tag: str
    description: str
    approach: bool = False


class SegmentedGuardedContactExecutor(Node):
    """Move toward contact in short segments and stop before pushing through."""

    JOINT_NAMES = (
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    )
    ACTION_SERVER = "/joint_trajectory_controller/follow_joint_trajectory"
    TASK_PHASE_TOPIC = "/task_phase"
    TASK_EVENT_TOPIC = "/task_event"
    TRIAL_STATUS_TOPIC = "/trial_status"
    FORCE_GUARD_STATUS_TOPIC = "/force_guard_status"
    INSERTION_METRICS_TOPIC = "/insertion_metrics"
    RESULT_TIMEOUT_MARGIN_SEC = 5.0
    SPIN_PERIOD_SEC = 0.05
    CANCEL_WAIT_TIMEOUT_SEC = 2.0
    PHASE_PUBLISH_PERIOD_SEC = 0.5

    SEGMENTS = (
        Segment(
            "safe_home",
            (0.0, -0.8, 1.2, 0.0, 0.8, 0.0),
            3.0,
            "nominal_clearance",
            "Raised parked configuration.",
        ),
        Segment(
            "observe_scene",
            (0.18, -0.92, 1.26, 0.0, 0.66, 0.0),
            3.0,
            "observation_clearance",
            "Observation posture before segmented approach.",
        ),
        Segment(
            "pre_approach",
            (0.32, -0.99, 1.25, 0.0, 0.57, 0.0),
            3.0,
            "robot_validation_approach_region",
            "Clear pre-approach posture outside contact.",
            True,
        ),
        Segment(
            "segment_01",
            (0.345, -1.035, 1.285, 0.0, 0.520, 0.0),
            2.5,
            "robot_validation_approach_region",
            "Short guarded approach segment 1.",
            True,
        ),
        Segment(
            "segment_02",
            (0.370, -1.080, 1.320, 0.0, 0.470, 0.0),
            2.5,
            "robot_validation_approach_region",
            "Short guarded approach segment 2.",
            True,
        ),
        Segment(
            "segment_03",
            (0.388, -1.115, 1.348, 0.0, 0.430, 0.0),
            2.5,
            "robot_validation_near_pad_region",
            "Short guarded approach segment 3.",
            True,
        ),
        Segment(
            "segment_04",
            (0.405, -1.150, 1.375, 0.0, 0.390, 0.0),
            2.5,
            "robot_validation_near_pad_region",
            "Short guarded approach segment 4.",
            True,
        ),
        Segment(
            "segment_05",
            (0.416, -1.178, 1.400, 0.0, 0.352, 0.0),
            2.5,
            "robot_validation_near_pad_region",
            "Short guarded approach segment 5.",
            True,
        ),
        Segment(
            "segment_06",
            (0.428, -1.205, 1.425, 0.0, 0.315, 0.0),
            2.5,
            "robot_validation_near_pad_region",
            "Short guarded approach segment 6.",
            True,
        ),
        Segment(
            "segment_07",
            (0.433, -1.218, 1.434, 0.0, 0.300, 0.0),
            2.5,
            "robot_validation_contact_region",
            "Short guarded approach segment 7.",
            True,
        ),
        Segment(
            "segment_08",
            (0.438, -1.230, 1.442, 0.0, 0.285, 0.0),
            2.5,
            "robot_validation_contact_region",
            "Final shallow guarded approach segment.",
            True,
        ),
    )
    RETREAT = Segment(
        "retreat",
        (0.22, -0.94, 1.23, 0.0, 0.62, 0.0),
        3.0,
        "robot_validation_retreat_clearance",
        "Retreat from the contact target.",
    )
    RETURN_HOME = Segment(
        "return_home",
        (0.0, -0.8, 1.2, 0.0, 0.8, 0.0),
        3.0,
        "nominal_clearance",
        "Return to raised parked home.",
    )

    def __init__(self) -> None:
        super().__init__("segmented_guarded_contact_executor")
        self.declare_parameter("action_server", self.ACTION_SERVER)
        self.declare_parameter("task_phase_topic", self.TASK_PHASE_TOPIC)
        self.declare_parameter("task_event_topic", self.TASK_EVENT_TOPIC)
        self.declare_parameter("trial_status_topic", self.TRIAL_STATUS_TOPIC)
        self.declare_parameter("force_guard_status_topic", self.FORCE_GUARD_STATUS_TOPIC)
        self.declare_parameter("insertion_metrics_topic", self.INSERTION_METRICS_TOPIC)
        self.declare_parameter("early_contact_force_threshold_n", 20.0)
        self.declare_parameter("force_violation_threshold_n", 100.0)
        self.declare_parameter("post_segment_guard_wait_sec", 0.3)

        self._action_server = (
            self.get_parameter("action_server").get_parameter_value().string_value
        )
        self._task_phase_topic = (
            self.get_parameter("task_phase_topic").get_parameter_value().string_value
        )
        self._task_event_topic = (
            self.get_parameter("task_event_topic").get_parameter_value().string_value
        )
        self._trial_status_topic = (
            self.get_parameter("trial_status_topic").get_parameter_value().string_value
        )
        self._force_guard_status_topic = (
            self.get_parameter("force_guard_status_topic")
            .get_parameter_value()
            .string_value
        )
        self._insertion_metrics_topic = (
            self.get_parameter("insertion_metrics_topic")
            .get_parameter_value()
            .string_value
        )
        self._early_contact_force_threshold_n = (
            self.get_parameter("early_contact_force_threshold_n")
            .get_parameter_value()
            .double_value
        )
        self._force_violation_threshold_n = (
            self.get_parameter("force_violation_threshold_n")
            .get_parameter_value()
            .double_value
        )
        self._post_segment_guard_wait_sec = (
            self.get_parameter("post_segment_guard_wait_sec")
            .get_parameter_value()
            .double_value
        )

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self._action_server,
        )
        self._phase_publisher = self.create_publisher(String, self._task_phase_topic, 10)
        self._event_publisher = self.create_publisher(String, self._task_event_topic, 10)
        self._status_publisher = self.create_publisher(
            String,
            self._trial_status_topic,
            10,
        )
        self.create_subscription(
            String,
            self._force_guard_status_topic,
            self._on_force_guard_status,
            100,
        )
        self.create_subscription(
            String,
            self._insertion_metrics_topic,
            self._on_insertion_metrics,
            100,
        )
        self.create_timer(self.PHASE_PUBLISH_PERIOD_SEC, self._publish_current_phase)

        self._current_phase = "idle"
        self._current_status = "idle"
        self._latest_physical_contact_observed = False
        self._latest_max_contact_force: float | None = None
        self._latest_contact_source = "unknown"
        self._latest_contact_count = 0
        self._force_extraction_available = False
        self._force_threshold_violation = False
        self._segment_count_executed = 0
        self._guarded_contact_stop = False
        self._active_approach_segment = False
        self._publish_trial_status("idle")

        self.get_logger().info(
            "Segmented guarded contact executor ready: "
            f"early_contact_force_threshold_n={self._early_contact_force_threshold_n:.2f}, "
            f"force_violation_threshold_n={self._force_violation_threshold_n:.2f}"
        )

    def execute(self) -> bool:
        self.get_logger().info("Waiting for FollowJointTrajectory action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server is available.")

        self._publish_trial_status("running")
        self._publish_phase("sequence_start")
        self._publish_event(
            "sequence_started",
            phase="sequence_start",
            pose_index=0,
            safety_tag="segmented_guarded_contact",
            message="Segmented guarded robot contact validation started.",
        )

        total_segments = len(self.SEGMENTS)
        for index, segment in enumerate(self.SEGMENTS, start=1):
            if segment.approach and self._guard_should_stop():
                self._publish_guard_stop(segment.name, index)
                self._execute_terminal_retreat(index)
                return True

            result = self._execute_segment(segment, index, total_segments)
            if result == "failed":
                self._publish_event(
                    "sequence_failed",
                    phase=segment.name,
                    pose_index=index,
                    safety_tag=segment.safety_tag,
                    message=f"Segmented sequence failed at '{segment.name}'.",
                )
                self._publish_terminal_state("failed", segment.name)
                return False
            if result == "guarded":
                self._publish_guard_stop(segment.name, index)
                self._execute_terminal_retreat(index)
                return True

            self._segment_count_executed += 1
            self._publish_event(
                "phase_succeeded",
                phase=segment.name,
                pose_index=index,
                safety_tag=segment.safety_tag,
                message=f"Segment '{segment.name}' completed.",
                extra_fields={"segment_count_executed": self._segment_count_executed},
            )

            self._wait_for_guard_update(self._post_segment_guard_wait_sec)
            if self._guard_should_stop():
                self._publish_guard_stop(segment.name, index)
                self._execute_terminal_retreat(index)
                return True

        self.get_logger().info(
            "Segmented approach completed without detecting physical contact."
        )
        self._execute_segment(self.RETREAT, total_segments + 1, total_segments + 2)
        self._execute_segment(self.RETURN_HOME, total_segments + 2, total_segments + 2)
        self._publish_event(
            "sequence_completed",
            phase="sequence_complete",
            pose_index=total_segments,
            safety_tag="segmented_guarded_contact",
            message="Segmented sequence completed without guarded contact stop.",
            extra_fields={"segment_count_executed": self._segment_count_executed},
        )
        self._publish_trial_status("completed")
        return True

    def _execute_terminal_retreat(self, interrupted_index: int) -> None:
        previous_active = self._active_approach_segment
        self._active_approach_segment = False
        try:
            self._execute_segment(self.RETREAT, interrupted_index + 1, interrupted_index + 2)
            self._execute_segment(
                self.RETURN_HOME,
                interrupted_index + 2,
                interrupted_index + 2,
            )
        finally:
            self._active_approach_segment = previous_active

    def _execute_segment(
        self,
        segment: Segment,
        index: int,
        total_segments: int,
    ) -> str:
        self._publish_phase(segment.name)
        self._publish_event(
            "phase_started",
            phase=segment.name,
            pose_index=index,
            safety_tag=segment.safety_tag,
            message=segment.description,
            extra_fields={"total_poses": total_segments},
        )
        goal_msg = self._build_goal(segment)
        self._publish_event(
            "goal_sent",
            phase=segment.name,
            pose_index=index,
            safety_tag=segment.safety_tag,
            message=(
                f"Sent short FollowJointTrajectory segment with duration "
                f"{segment.duration_sec:.2f}s."
            ),
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Goal for segment '{segment.name}' was rejected.")
            self._publish_event(
                "goal_rejected",
                phase=segment.name,
                pose_index=index,
                safety_tag=segment.safety_tag,
                message="FollowJointTrajectory segment goal was rejected.",
            )
            return "failed"

        self._publish_event(
            "goal_accepted",
            phase=segment.name,
            pose_index=index,
            safety_tag=segment.safety_tag,
            message="FollowJointTrajectory segment goal accepted.",
        )
        result_future = goal_handle.get_result_async()
        wait_status = self._wait_for_result(
            result_future,
            goal_handle=goal_handle,
            segment=segment,
            index=index,
            timeout_sec=segment.duration_sec + self.RESULT_TIMEOUT_MARGIN_SEC,
        )
        if wait_status != "completed":
            return wait_status

        wrapped_result = result_future.result()
        if wrapped_result is None:
            return "failed"
        result = wrapped_result.result
        if (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        ):
            return "completed"

        self.get_logger().error(
            f"Segment '{segment.name}' failed: "
            f"status={self._goal_status_name(wrapped_result.status)}, "
            f"result={self._result_error_name(result.error_code)}."
        )
        return "failed"

    def _wait_for_result(
        self,
        result_future: Future,
        *,
        goal_handle: Any,
        segment: Segment,
        index: int,
        timeout_sec: float,
    ) -> str:
        deadline = time.monotonic() + timeout_sec
        previous_active = self._active_approach_segment
        self._active_approach_segment = segment.approach
        try:
            while not result_future.done():
                if segment.approach and self._guard_should_stop():
                    self.get_logger().warning(
                        f"Guard detected during '{segment.name}'; canceling segment."
                    )
                    self._cancel_goal(goal_handle, segment.name)
                    return "guarded"
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        f"Timed out waiting for segment '{segment.name}'."
                    )
                    self._cancel_goal(goal_handle, segment.name)
                    return "failed"
                rclpy.spin_once(self, timeout_sec=self.SPIN_PERIOD_SEC)
            return "completed"
        finally:
            self._active_approach_segment = previous_active

    def _guard_should_stop(self) -> bool:
        force_trip = (
            self._latest_max_contact_force is not None
            and (
                self._latest_max_contact_force >= self._early_contact_force_threshold_n
                or self._latest_max_contact_force >= self._force_violation_threshold_n
            )
        )
        return (
            self._latest_physical_contact_observed
            or self._force_threshold_violation
            or force_trip
        )

    def _publish_guard_stop(self, phase: str, index: int) -> None:
        if self._guarded_contact_stop:
            return
        self._guarded_contact_stop = True
        force = self._latest_max_contact_force
        force_text = "unavailable" if force is None else f"{force:.3f}N"
        self._publish_event(
            "early_contact_guard_triggered",
            phase=phase,
            pose_index=index,
            safety_tag="segmented_guarded_contact",
            message=(
                "Segmented guard stopped further approach: "
                f"physical_contact_observed={self._latest_physical_contact_observed}, "
                f"source={self._latest_contact_source}, "
                f"contact_count={self._latest_contact_count}, "
                f"max_contact_force={force_text}."
            ),
            extra_fields={
                "early_contact_guard_trigger_force": force,
                "early_contact_guard_threshold": self._early_contact_force_threshold_n,
                "early_contact_guard_source": self._latest_contact_source,
                "early_contact_guard_contact_count": self._latest_contact_count,
                "segment_count_executed": self._segment_count_executed,
                "guarded_contact_stop": True,
            },
        )
        self._publish_event(
            "sequence_guarded_stop",
            phase=phase,
            pose_index=index,
            safety_tag="segmented_guarded_contact",
            message="Segmented guarded contact stop reached; retreating.",
            extra_fields={"segment_count_executed": self._segment_count_executed},
        )
        self._publish_terminal_state("guarded_contact_stop", phase)

    def _build_goal(self, segment: Segment) -> FollowJointTrajectory.Goal:
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(self.JOINT_NAMES)
        goal_msg.trajectory.header.stamp.sec = 0
        goal_msg.trajectory.header.stamp.nanosec = 0

        point = JointTrajectoryPoint()
        point.positions = list(segment.positions)
        point.time_from_start = self._seconds_to_duration(segment.duration_sec)
        goal_msg.trajectory.points.append(point)
        return goal_msg

    def _cancel_goal(self, goal_handle: Any, segment_name: str) -> None:
        cancel_future = goal_handle.cancel_goal_async()
        deadline = time.monotonic() + self.CANCEL_WAIT_TIMEOUT_SEC
        while not cancel_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=self.SPIN_PERIOD_SEC)
        if cancel_future.done():
            self.get_logger().info(f"Cancel completed for segment '{segment_name}'.")
        else:
            self.get_logger().warning(
                f"Cancel for segment '{segment_name}' did not complete before timeout."
            )

    def _wait_for_guard_update(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(self.SPIN_PERIOD_SEC, duration_sec))

    def _on_force_guard_status(self, message: String) -> None:
        payload = self._parse_json(message.data, "force_guard_status")
        if not payload:
            return
        self._update_guard_state(payload)

    def _on_insertion_metrics(self, message: String) -> None:
        payload = self._parse_json(message.data, "insertion_metrics")
        if not payload:
            return
        self._update_guard_state(payload)

    def _update_guard_state(self, payload: dict[str, Any]) -> None:
        force = self._coerce_optional_float(payload.get("max_contact_force"))
        if force is not None:
            self._latest_max_contact_force = (
                force
                if self._latest_max_contact_force is None
                else max(self._latest_max_contact_force, force)
            )
        self._latest_physical_contact_observed = (
            self._latest_physical_contact_observed
            or bool(payload.get("physical_contact_observed", False))
        )
        self._force_extraction_available = self._force_extraction_available or bool(
            payload.get("force_extraction_available", False)
        )
        self._force_threshold_violation = self._force_threshold_violation or bool(
            payload.get("force_threshold_violation", False)
        )
        source = str(payload.get("source", self._latest_contact_source)).strip()
        if source:
            self._latest_contact_source = source
        contact_count = self._coerce_int(payload.get("contact_count"), default=0)
        if contact_count > self._latest_contact_count:
            self._latest_contact_count = contact_count

    def _publish_phase(self, phase: str) -> None:
        self._current_phase = phase
        self._publish_current_phase()
        rclpy.spin_once(self, timeout_sec=0.05)

    def _publish_current_phase(self) -> None:
        message = String()
        message.data = self._current_phase
        self._phase_publisher.publish(message)

    def _publish_trial_status(self, status: str) -> None:
        self._current_status = status
        message = String()
        message.data = status
        self._status_publisher.publish(message)
        self.get_logger().info(f"trial_status={status}")
        rclpy.spin_once(self, timeout_sec=0.05)

    def _publish_terminal_state(self, status: str, phase: str) -> None:
        self._publish_phase(f"{status}:{phase}")
        self._publish_trial_status(status)

    def _publish_event(
        self,
        event_type: str,
        *,
        phase: str,
        pose_index: int,
        safety_tag: str,
        message: str,
        extra_fields: dict[str, Any] | None = None,
    ) -> None:
        payload = {
            "timestamp_ros_sec": self._now_sec(),
            "event_type": event_type,
            "phase": phase,
            "pose_index": pose_index,
            "total_poses": len(self.SEGMENTS),
            "safety_tag": safety_tag,
            "message": message,
            "trial_status": self._current_status,
            "segment_count_executed": self._segment_count_executed,
            "max_contact_force": self._latest_max_contact_force,
            "physical_contact_observed": self._latest_physical_contact_observed,
        }
        if extra_fields:
            payload.update(extra_fields)
        event_message = String()
        event_message.data = json.dumps(payload, sort_keys=True)
        self._event_publisher.publish(event_message)
        rclpy.spin_once(self, timeout_sec=0.02)

    def _parse_json(self, data: str, source: str) -> dict[str, Any] | None:
        try:
            payload = json.loads(data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Ignoring malformed {source} JSON: {exc}")
            return None
        if not isinstance(payload, dict):
            return None
        return payload

    @staticmethod
    def _seconds_to_duration(seconds: float) -> Duration:
        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - duration.sec) * 1_000_000_000)
        return duration

    @staticmethod
    def _coerce_optional_float(value: Any) -> float | None:
        if value is None or value == "":
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _coerce_int(value: Any, default: int) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

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
        return names.get(status, f"UNRECOGNIZED_{status}")

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
        return names.get(error_code, f"UNRECOGNIZED_{error_code}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SegmentedGuardedContactExecutor()
    try:
        node.execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
