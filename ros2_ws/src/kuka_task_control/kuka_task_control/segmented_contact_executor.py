"""Segmented guarded contact executor for KUKA validation trials."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


@dataclass(frozen=True)
class ContactSegment:
    name: str
    positions: tuple[float, float, float, float, float, float]
    duration_sec: float
    safety_tag: str
    description: str
    approach: bool = False


class SegmentedContactExecutor(Node):
    """Approach contact through short checked trajectory goals, then retreat."""

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
    DEFAULT_BASE_FRAME = "base_link"
    DEFAULT_TOOL_FRAME = "tool0"
    TOOL_FRAME_FALLBACKS = (
        "tool0",
        "tcp",
        "gripper_tcp",
        "end_effector_link",
        "flange",
        "link_6",
    )

    def __init__(self) -> None:
        super().__init__("segmented_contact_executor")
        self.declare_parameter("config_path", "")
        self.declare_parameter("action_server", self.ACTION_SERVER)
        self.declare_parameter("task_phase_topic", self.TASK_PHASE_TOPIC)
        self.declare_parameter("task_event_topic", self.TASK_EVENT_TOPIC)
        self.declare_parameter("trial_status_topic", self.TRIAL_STATUS_TOPIC)
        self.declare_parameter("force_guard_status_topic", self.FORCE_GUARD_STATUS_TOPIC)
        self.declare_parameter("insertion_metrics_topic", self.INSERTION_METRICS_TOPIC)
        self.declare_parameter("base_frame", self.DEFAULT_BASE_FRAME)
        self.declare_parameter("tool_frame", self.DEFAULT_TOOL_FRAME)

        config = self._load_config(self._resolve_config_path())
        thresholds = config.get("thresholds", {})
        if not isinstance(thresholds, dict):
            raise ValueError("Segmented contact config field 'thresholds' must be a map.")

        self._early_contact_force_threshold_n = float(
            thresholds.get("early_contact_force_threshold_n", 20.0)
        )
        self._force_violation_threshold_n = float(
            thresholds.get("force_violation_threshold_n", 100.0)
        )
        self._post_segment_guard_wait_sec = float(
            config.get("post_segment_guard_wait_sec", 0.3)
        )

        self._safe_home = self._parse_segment(
            "safe_home", config.get("safe_home"), approach=False
        )
        self._pre_approach = self._parse_segment(
            "pre_approach", config.get("pre_approach"), approach=False
        )
        self._approach_segments = self._parse_approach_segments(
            config.get("approach_segments")
        )
        self._retreat = self._parse_segment(
            "retreat", config.get("retreat"), approach=False
        )
        self._sequence = (self._safe_home, self._pre_approach, *self._approach_segments)

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
        self._base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
            or self.DEFAULT_BASE_FRAME
        )
        configured_tool_frame = (
            self.get_parameter("tool_frame").get_parameter_value().string_value
            or self.DEFAULT_TOOL_FRAME
        )
        self._tool_frame_candidates = self._dedupe_frame_candidates(
            (configured_tool_frame, *self.TOOL_FRAME_FALLBACKS)
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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
            self._on_guard_status,
            100,
        )
        self.create_subscription(
            String,
            self._insertion_metrics_topic,
            self._on_guard_status,
            100,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            100,
        )
        self.create_timer(self.PHASE_PUBLISH_PERIOD_SEC, self._publish_current_phase)

        self._current_phase = "idle"
        self._current_status = "idle"
        self._latest_physical_contact_observed = False
        self._latest_max_contact_force: float | None = None
        self._latest_contact_source = "unknown"
        self._latest_contact_count = 0
        self._latest_collision_pairs: list[str] = []
        self._latest_first_collision1: str | None = None
        self._latest_first_collision2: str | None = None
        self._force_threshold_violation = False
        self._segment_count_executed = 0
        self._guarded_contact_stop = False
        self._pre_contact_failed = False
        self._latest_joint_positions: dict[str, float] = {}
        self._publish_trial_status("idle")

        self.get_logger().info(
            "Segmented contact executor ready: "
            f"segments={len(self._approach_segments)}, "
            f"early_contact_force_threshold_n={self._early_contact_force_threshold_n:.2f}, "
            f"force_violation_threshold_n={self._force_violation_threshold_n:.2f}"
        )

    def execute(self) -> bool:
        self.get_logger().info("Waiting for FollowJointTrajectory action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server is available.")

        self._publish_trial_status("running")
        self._publish_phase("segmented_sequence_start")
        self._publish_event(
            "segmented_sequence_started",
            phase="segmented_sequence_start",
            segment_index=0,
            safety_tag="segmented_robot_contact_validation",
            message="Segmented guarded robot contact approach started.",
        )

        total_segments = len(self._sequence)
        for index, segment in enumerate(self._sequence, start=1):
            if segment.approach and self._guard_should_stop():
                self._publish_pre_approach_contact_failure(segment.name, index)
                self._execute_retreat(index)
                return False
            if segment.approach:
                self._refresh_guard_status()
                if self._guard_should_stop():
                    self._publish_pre_approach_contact_failure(segment.name, index)
                    self._execute_retreat(index)
                    return False

            result = self._execute_segment(segment, index, total_segments)
            if result == "failed":
                self._publish_event(
                    "segmented_sequence_failed",
                    phase=segment.name,
                    segment_index=index,
                    safety_tag=segment.safety_tag,
                    message=f"Segmented contact sequence failed at '{segment.name}'.",
                )
                self._publish_terminal_state("failed", segment.name)
                return False
            if result == "pre_contact":
                self._publish_pre_approach_contact_failure(segment.name, index)
                self._execute_retreat(index)
                return False
            if result == "guarded":
                self._publish_guarded_stop(segment.name, index)
                self._execute_retreat(index)
                return True

            if segment.approach:
                self._refresh_guard_status()
                if self._guard_should_stop():
                    self._segment_count_executed += 1
                    endpoint_fields = self._segment_endpoint_fields(segment, index)
                    self._publish_event(
                        "segment_succeeded",
                        phase=segment.name,
                        segment_index=index,
                        safety_tag=segment.safety_tag,
                        message=f"Segment '{segment.name}' completed.",
                        extra_fields=endpoint_fields,
                    )
                    self._publish_guarded_stop(segment.name, index)
                    self._execute_retreat(index)
                    return True

            self._segment_count_executed += 1
            endpoint_fields = self._segment_endpoint_fields(segment, index)
            self._publish_event(
                "segment_succeeded",
                phase=segment.name,
                segment_index=index,
                safety_tag=segment.safety_tag,
                message=f"Segment '{segment.name}' completed.",
                extra_fields=endpoint_fields,
            )
            if segment.approach and index == total_segments:
                self._publish_event(
                    "final_segment_endpoint",
                    phase=segment.name,
                    segment_index=index,
                    safety_tag=segment.safety_tag,
                    message="Final segmented approach endpoint diagnostic.",
                    extra_fields=endpoint_fields,
                )

            self._wait_for_guard_update(self._post_segment_guard_wait_sec)
            if not segment.approach and self._guard_should_stop():
                next_phase = (
                    self._approach_segments[0].name
                    if self._approach_segments
                    else "contact_segment_01"
                )
                self._publish_pre_approach_contact_failure(next_phase, index + 1)
                self._execute_retreat(index)
                return False
            if segment.approach and self._guard_should_stop():
                self._publish_guarded_stop(segment.name, index)
                self._execute_retreat(index)
                return True

        self._execute_retreat(total_segments)
        self._publish_event(
            "segmented_sequence_completed",
            phase="segmented_sequence_completed",
            segment_index=total_segments,
            safety_tag="segmented_robot_contact_validation",
            message="Segmented contact approach completed without guarded contact stop.",
        )
        self._publish_terminal_state("completed", "segmented_sequence_completed")
        return True

    def _execute_retreat(self, interrupted_index: int) -> bool:
        retreat_index = interrupted_index + 1
        self._publish_event(
            "retreat_started",
            phase=self._retreat.name,
            segment_index=retreat_index,
            safety_tag=self._retreat.safety_tag,
            message="Executing retreat after segmented contact approach.",
        )
        result = self._execute_segment(self._retreat, retreat_index, retreat_index)
        if result == "completed":
            self._publish_event(
                "retreat_completed",
                phase=self._retreat.name,
                segment_index=retreat_index,
                safety_tag=self._retreat.safety_tag,
                message="Retreat completed after segmented contact approach.",
                extra_fields=self._segment_endpoint_fields(self._retreat, retreat_index),
            )
            return True
        self._publish_event(
            "segmented_sequence_failed",
            phase=self._retreat.name,
            segment_index=retreat_index,
            safety_tag=self._retreat.safety_tag,
            message="Retreat failed after segmented contact approach.",
        )
        return False

    def _execute_segment(
        self,
        segment: ContactSegment,
        index: int,
        total_segments: int,
    ) -> str:
        self._publish_phase(segment.name)
        self._publish_event(
            "segment_started",
            phase=segment.name,
            segment_index=index,
            safety_tag=segment.safety_tag,
            message=segment.description,
            extra_fields={
                "total_segments": total_segments,
                "duration_sec": segment.duration_sec,
            },
        )

        goal_msg = self._build_goal(segment)
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Goal for segment '{segment.name}' was rejected.")
            return "failed"

        result_future = goal_handle.get_result_async()
        wait_status = self._wait_for_result(
            result_future,
            goal_handle=goal_handle,
            segment=segment,
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
        segment: ContactSegment,
        timeout_sec: float,
    ) -> str:
        deadline = time.monotonic() + timeout_sec
        while not result_future.done():
            if (
                self._is_pre_approach_clearance_segment(segment)
                and self._guard_should_stop()
            ):
                self.get_logger().warning(
                    f"Unexpected pre-approach contact during '{segment.name}'; canceling."
                )
                self._cancel_goal(goal_handle, segment.name)
                return "pre_contact"
            if segment.approach and self._guard_should_stop():
                self.get_logger().warning(
                    f"Contact guard detected during '{segment.name}'; canceling."
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

    def _guard_should_stop(self) -> bool:
        force_trip = (
            self._latest_max_contact_force is not None
            and self._latest_max_contact_force >= self._early_contact_force_threshold_n
        )
        violation_trip = (
            self._latest_max_contact_force is not None
            and self._latest_max_contact_force >= self._force_violation_threshold_n
        )
        return (
            self._latest_physical_contact_observed
            or self._force_threshold_violation
            or force_trip
            or violation_trip
        )

    def _is_pre_approach_clearance_segment(self, segment: ContactSegment) -> bool:
        return segment.name in {self._safe_home.name, self._pre_approach.name}

    def _publish_pre_approach_contact_failure(self, phase: str, index: int) -> None:
        if self._pre_contact_failed:
            return
        self._pre_contact_failed = True
        force = self._latest_max_contact_force
        self._publish_event(
            "unexpected_pre_approach_contact",
            phase=phase,
            segment_index=index,
            safety_tag="segmented_robot_contact_validation",
            message=(
                "Unexpected contact detected before contact_segment_01 started; "
                "failing segmented contact validation."
            ),
            extra_fields={
                "pre_approach_contact_detected": True,
                "early_contact_guard_triggered": True,
                "early_contact_guard_trigger_force": force,
                "early_contact_guard_threshold": self._early_contact_force_threshold_n,
                "early_contact_guard_source": self._latest_contact_source,
                "early_contact_guard_contact_count": self._latest_contact_count,
                "collision_pairs": list(self._latest_collision_pairs),
                "first_collision1": self._latest_first_collision1,
                "first_collision2": self._latest_first_collision2,
                "segmented_contact_success": False,
            },
        )
        self._publish_terminal_state("failed_pre_contact", phase)

    def _publish_guarded_stop(self, phase: str, index: int) -> None:
        if self._guarded_contact_stop:
            return
        self._guarded_contact_stop = True
        force = self._latest_max_contact_force
        self._publish_event(
            "early_contact_detected",
            phase=phase,
            segment_index=index,
            safety_tag="segmented_robot_contact_validation",
            message="Segmented contact guard detected physical contact or threshold force.",
            extra_fields={
                "early_contact_guard_triggered": True,
                "early_contact_guard_trigger_force": force,
                "early_contact_guard_threshold": self._early_contact_force_threshold_n,
                "early_contact_guard_source": self._latest_contact_source,
                "early_contact_guard_contact_count": self._latest_contact_count,
                "collision_pairs": list(self._latest_collision_pairs),
                "first_collision1": self._latest_first_collision1,
                "first_collision2": self._latest_first_collision2,
                "guarded_contact_stop": True,
            },
        )
        self._publish_event(
            "guarded_contact_stop",
            phase=phase,
            segment_index=index,
            safety_tag="segmented_robot_contact_validation",
            message="Segmented approach stopped before sending further contact motion.",
            extra_fields={"guarded_contact_stop": True},
        )
        self._publish_terminal_state("guarded_contact_stop", phase)

    def _build_goal(self, segment: ContactSegment) -> FollowJointTrajectory.Goal:
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
            rclpy.spin_once(self, timeout_sec=self.SPIN_PERIOD_SEC)

    def _refresh_guard_status(self) -> None:
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.0)

    def _on_guard_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Ignoring malformed guard JSON: {exc}")
            return
        if not isinstance(payload, dict):
            return

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
        self._force_threshold_violation = self._force_threshold_violation or bool(
            payload.get("force_threshold_violation", False)
        )
        source = str(payload.get("source", self._latest_contact_source)).strip()
        if source:
            self._latest_contact_source = source
        contact_count = self._coerce_int(payload.get("contact_count"), default=0)
        self._latest_contact_count = max(self._latest_contact_count, contact_count)
        self._update_collision_diagnostics(payload)

    def _on_joint_state(self, message: JointState) -> None:
        if not message.name or not message.position:
            return
        for name, position in zip(message.name, message.position):
            if name in self.JOINT_NAMES:
                self._latest_joint_positions[name] = float(position)

    def _segment_endpoint_fields(
        self,
        segment: ContactSegment,
        index: int,
    ) -> dict[str, Any]:
        fields: dict[str, Any] = {
            "segment_name": segment.name,
            "segment_index": index,
            "target_joint_positions": list(segment.positions),
        }
        fields.update(self._end_effector_pose_fields(segment.name, index))
        reached = self._reached_joint_positions()
        if reached is None:
            return fields
        fields["reached_joint_positions"] = reached
        fields["joint_position_error"] = [
            reached_position - target_position
            for reached_position, target_position in zip(reached, segment.positions)
        ]
        return fields

    def _end_effector_pose_fields(
        self,
        segment_name: str,
        segment_index: int,
    ) -> dict[str, Any]:
        attempted_frames: list[str] = []
        last_error = ""
        for tool_frame in self._tool_frame_candidates:
            attempted_frames.append(tool_frame)
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    tool_frame,
                    Time(),
                )
            except TransformException as exc:
                last_error = str(exc)
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return {
                "end_effector_base_frame": self._base_frame,
                "end_effector_tool_frame": tool_frame,
                "end_effector_position_xyz": [
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                ],
                "end_effector_orientation_xyzw": [
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ],
            }

        known_frames = self._known_tf_frames()
        message = (
            "TF lookup failed for segmented endpoint; "
            f"base_frame='{self._base_frame}', "
            f"attempted_tool_frames={attempted_frames}, "
            f"last_error='{last_error}'."
        )
        if known_frames:
            message = f"{message} Known TF frames: {known_frames}"
        self.get_logger().warning(message)
        self._publish_event(
            "tf_lookup_failed",
            phase=segment_name,
            segment_index=segment_index,
            safety_tag="segmented_robot_contact_validation",
            message=message,
            extra_fields={
                "end_effector_base_frame": self._base_frame,
                "attempted_tool_frames": attempted_frames,
                "known_tf_frames": known_frames,
            },
        )
        return {}

    def _known_tf_frames(self) -> str:
        try:
            return self._tf_buffer.all_frames_as_yaml()
        except Exception as exc:  # pragma: no cover - defensive TF diagnostics only.
            return f"unavailable ({exc})"

    def _reached_joint_positions(self) -> list[float] | None:
        if not all(name in self._latest_joint_positions for name in self.JOINT_NAMES):
            return None
        return [self._latest_joint_positions[name] for name in self.JOINT_NAMES]

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
        segment_index: int,
        safety_tag: str,
        message: str,
        extra_fields: dict[str, Any] | None = None,
    ) -> None:
        payload = {
            "timestamp_ros_sec": self._now_sec(),
            "event_type": event_type,
            "phase": phase,
            "pose_index": segment_index,
            "segment_index": segment_index,
            "total_poses": len(self._sequence),
            "total_segments": len(self._sequence),
            "safety_tag": safety_tag,
            "message": message,
            "trial_status": self._current_status,
            "segment_count_executed": self._segment_count_executed,
            "max_contact_force": self._latest_max_contact_force,
            "physical_contact_observed": self._latest_physical_contact_observed,
            "force_threshold_violation": self._force_threshold_violation,
            "collision_pairs": list(self._latest_collision_pairs),
            "first_collision1": self._latest_first_collision1,
            "first_collision2": self._latest_first_collision2,
        }
        if extra_fields:
            payload.update(extra_fields)
        event_message = String()
        event_message.data = json.dumps(payload, sort_keys=True)
        self._event_publisher.publish(event_message)
        self.get_logger().info(
            f"task_event={event_type} phase={phase} segment={segment_index}: {message}"
        )
        rclpy.spin_once(self, timeout_sec=0.02)

    def _resolve_config_path(self) -> Path:
        config_path = Path(
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        if str(config_path):
            return config_path
        return (
            Path(get_package_share_directory("kuka_task_control"))
            / "config"
            / "segmented_robot_contact_approach.yaml"
        )

    @classmethod
    def _parse_approach_segments(cls, value: Any) -> tuple[ContactSegment, ...]:
        if not isinstance(value, list) or not value:
            raise ValueError("Config field 'approach_segments' must be a non-empty list.")
        return tuple(
            cls._parse_segment(
                f"approach_segments[{index}]",
                segment,
                approach=True,
            )
            for index, segment in enumerate(value)
        )

    @classmethod
    def _parse_segment(
        cls,
        label: str,
        value: Any,
        *,
        approach: bool,
    ) -> ContactSegment:
        if not isinstance(value, dict):
            raise ValueError(f"Config field '{label}' must be a map.")
        name = str(value.get("name", label)).strip()
        if not name:
            raise ValueError(f"Config field '{label}.name' must be non-empty.")
        positions = value.get("positions")
        if not isinstance(positions, list) or len(positions) != len(cls.JOINT_NAMES):
            raise ValueError(
                f"Config field '{label}.positions' must contain exactly "
                f"{len(cls.JOINT_NAMES)} joint values."
            )
        duration_sec = float(value.get("duration_sec", 0.0))
        if duration_sec <= 0.0:
            raise ValueError(f"Config field '{label}.duration_sec' must be positive.")
        if approach and not 2.0 <= duration_sec <= 5.0:
            raise ValueError(
                f"Approach segment '{name}' duration must be between 2 and 5 seconds."
            )
        safety_tag = str(value.get("safety_tag", "segmented_contact")).strip()
        description = str(value.get("description", name)).strip()
        return ContactSegment(
            name=name,
            positions=tuple(float(position) for position in positions),  # type: ignore[arg-type]
            duration_sec=duration_sec,
            safety_tag=safety_tag,
            description=description,
            approach=approach,
        )

    @staticmethod
    def _load_config(config_path: Path) -> dict[str, Any]:
        if not config_path.is_file():
            raise FileNotFoundError(f"Segmented contact config does not exist: {config_path}")
        with config_path.open("r", encoding="utf-8") as config_file:
            loaded = yaml.safe_load(config_file)
        if not isinstance(loaded, dict):
            raise ValueError(f"Segmented contact config must be a YAML map: {config_path}")
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

    def _update_collision_diagnostics(self, payload: dict[str, Any]) -> None:
        pairs = payload.get("collision_pairs")
        if isinstance(pairs, list):
            for pair in pairs:
                pair_text = str(pair)
                if pair_text and pair_text not in self._latest_collision_pairs:
                    self._latest_collision_pairs.append(pair_text)
        first_collision1 = payload.get("first_collision1")
        first_collision2 = payload.get("first_collision2")
        if self._latest_first_collision1 is None and first_collision1 not in (None, ""):
            self._latest_first_collision1 = str(first_collision1)
            self._latest_first_collision2 = (
                None if first_collision2 in (None, "") else str(first_collision2)
            )

    @staticmethod
    def _dedupe_frame_candidates(candidates: tuple[str, ...]) -> tuple[str, ...]:
        deduped: list[str] = []
        for candidate in candidates:
            frame = candidate.strip()
            if frame and frame not in deduped:
                deduped.append(frame)
        return tuple(deduped)

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
    node = SegmentedContactExecutor()
    try:
        success = node.execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
