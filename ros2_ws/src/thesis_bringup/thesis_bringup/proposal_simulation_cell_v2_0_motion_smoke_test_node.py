"""First Gazebo-only motion smoke test for proposal_simulation_cell_v2_0."""

from __future__ import annotations

import csv
import json
import math
import re
import subprocess
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


class ProposalSimulationCellV20MotionSmokeTestNode(Node):
    """Send one small command only after the Gazebo control path is verified."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_0_motion_smoke_test_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_0")

        self._config = self._load_config()
        robot = self._config.get("robot", {})
        motion = self._config.get("selected_joint_motion", {})
        safety = self._config.get("safety_limits", {})
        execution = self._config.get("execution_policy", {})
        validation = self._config.get("validation", {})
        gazebo = self._config.get("gazebo_motion_test", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_0")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._robot_model = str(robot.get("robot_model", "KUKA LBR iisy Gazebo support model"))
        self._simulation_engine = str(gazebo.get("simulation_engine", "gazebo"))
        self._gazebo_only_motion_test = bool(gazebo.get("gazebo_only_motion_test", True))
        self._selected_joint = str(motion.get("selected_joint", robot.get("proposal_selected_joint", "joint_a6")))
        self._controller_joint = str(motion.get("controller_joint", robot.get("gazebo_controller_joint", "joint_6")))
        self._joint_names = [str(item) for item in robot.get("controller_joint_names", [])]
        self._commanded_delta_deg = float(motion.get("commanded_joint_delta_deg", 2.0))
        self._max_joint_delta_deg = float(motion.get("max_joint_delta_deg", 2.0))
        self._duration_sec = float(motion.get("motion_duration_sec", 5.0))
        self._max_allowed_force = float(safety.get("max_allowed_force_n", 50.0))
        self._max_allowed_torque = float(safety.get("max_allowed_torque_nm", 5.0))
        self._emergency_force = float(safety.get("emergency_stop_force_threshold_n", 45.0))
        self._delta_tolerance_deg = float(safety.get("max_observed_joint_delta_tolerance_deg", 2.25))
        self._motion_execution_enabled = bool(execution.get("motion_execution_enabled", True))
        self._real_robot_allowed = bool(execution.get("real_robot_allowed", False))
        self._moveit_allowed = bool(execution.get("moveit_allowed", False))
        self._compute_ik_allowed = bool(execution.get("compute_ik_allowed", False))
        self._simulation_control_interface = str(
            gazebo.get(
                "simulation_control_interface_used",
                "gz_ros2_control/GazeboSimSystem via joint_trajectory_controller",
            )
        )
        self._action_name = str(gazebo.get("control_interface", "/joint_trajectory_controller/follow_joint_trajectory"))
        self._joint_states_topic = str(validation.get("joint_states_topic", "/joint_states"))
        self._contact_wrench_topic = str(validation.get("contact_wrench_topic", "/proposal_simulation_cell/contact_wrench"))
        self._timeout_sec = float(validation.get("validation_timeout_sec", 25.0))
        self._pre_motion_wait_sec = float(validation.get("pre_motion_wait_sec", 4.0))
        self._success_status = str(validation.get("status_success", "first_gazebo_motion_smoke_test_validated"))

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/first_motion_smoke_test_status")),
            10,
        )
        self._delta_pub = self.create_publisher(
            String,
            str(validation.get("joint_delta_report_topic", "/proposal_simulation_cell/first_motion_joint_delta_report")),
            10,
        )
        self._safety_pub = self.create_publisher(
            String,
            str(validation.get("safety_report_topic", "/proposal_simulation_cell/first_motion_safety_report")),
            10,
        )
        self._contact_wrench_pub = self.create_publisher(WrenchStamped, self._contact_wrench_topic, 10)

        self.create_subscription(JointState, self._joint_states_topic, self._on_joint_state, 10)
        self.create_subscription(WrenchStamped, self._contact_wrench_topic, self._on_contact_wrench, 10)
        self.create_subscription(String, "/robot_description", self._on_robot_description, 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)

        self._start_time = time.monotonic()
        self._last_joint_state: JointState | None = None
        self._before_joint_state: JointState | None = None
        self._after_joint_state: JointState | None = None
        self._robot_description = ""
        self._max_force = 0.0
        self._max_torque = 0.0
        self._safety_violation_count = 0
        self._motion_command_sent = False
        self._motion_goal_done = False
        self._motion_goal_accepted = False
        self._motion_attempted = False
        self._preflight_attempt_count = 0
        self._finished = False
        self._last_status: dict[str, Any] = {}
        self._gz_topics: list[str] = []
        self._gz_contact_topics: list[str] = []
        self._controller_available = False
        self._gazebo_verified = False
        self._real_robot_endpoint_detected = False

        self.create_timer(0.2, self._tick)
        self.get_logger().info("proposal_simulation_cell_v2_0 Gazebo-only motion smoke test node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v2.0 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _on_joint_state(self, message: JointState) -> None:
        self._last_joint_state = message

    def _on_robot_description(self, message: String) -> None:
        self._robot_description = message.data

    def _on_contact_wrench(self, message: WrenchStamped) -> None:
        force = self._force_magnitude(message)
        torque = self._torque_magnitude(message)
        self._max_force = max(self._max_force, force)
        self._max_torque = max(self._max_torque, torque)
        if force > self._emergency_force or force > self._max_allowed_force or torque > self._max_allowed_torque:
            self._safety_violation_count += 1

    def _tick(self) -> None:
        if self._finished:
            return
        elapsed = time.monotonic() - self._start_time
        self._publish_contact_wrench_sample()
        self._publish_reports(outputs_written=False)
        if not self._motion_command_sent and not self._motion_goal_done and elapsed >= self._pre_motion_wait_sec:
            self._attempt_motion()
        if self._motion_attempted and not self._motion_goal_done and elapsed >= self._pre_motion_wait_sec + self._duration_sec + 4.0:
            self._motion_goal_done = True
        if elapsed >= self._timeout_sec or (self._motion_attempted and self._motion_goal_done and elapsed >= self._pre_motion_wait_sec + self._duration_sec + 1.0):
            self._write_outputs_once()

    def _attempt_motion(self) -> None:
        self._preflight_attempt_count += 1
        self._gz_topics = self._run_command(["gz", "topic", "-l"], timeout=2.0)
        self._gz_contact_topics = [
            topic.strip()
            for topic in self._gz_topics
            if topic.strip().startswith("/") and "/sensor/" in topic and topic.strip().endswith("/contact")
        ]
        self._controller_available = self._action_client.wait_for_server(timeout_sec=0.5)
        self._gazebo_verified = self._verify_gazebo_only_control_path()
        if not self._can_send_motion():
            if time.monotonic() - self._start_time >= self._timeout_sec - 2.0:
                self._motion_attempted = True
                self._motion_goal_done = True
                self.get_logger().warning("Gazebo-only control path unavailable; motion command not sent")
            return
        self._motion_attempted = True
        self._before_joint_state = self._last_joint_state
        before_positions = self._joint_positions(self._before_joint_state)
        target_positions = [before_positions.get(name, 0.0) for name in self._joint_names]
        joint_index = self._joint_names.index(self._controller_joint)
        target_positions[joint_index] += math.radians(self._commanded_delta_deg)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = int(self._duration_sec)
        point.time_from_start.nanosec = int((self._duration_sec - int(self._duration_sec)) * 1_000_000_000)
        goal.trajectory.points.append(point)

        self._motion_command_sent = True
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info("Sent one Gazebo-only minimal joint-space motion command")

    def _on_goal_response(self, future: Any) -> None:
        goal_handle = future.result()
        self._motion_goal_accepted = bool(goal_handle and goal_handle.accepted)
        if not self._motion_goal_accepted:
            self._motion_goal_done = True
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future: Any) -> None:
        self._after_joint_state = self._last_joint_state
        self._motion_goal_done = True

    def _can_send_motion(self) -> bool:
        return all(
            [
                self._motion_execution_enabled,
                self._gazebo_only_motion_test,
                self._simulation_engine == "gazebo",
                not self._real_robot_allowed,
                not self._moveit_allowed,
                not self._compute_ik_allowed,
                not self._real_robot_endpoint_detected,
                self._gazebo_verified,
                self._controller_available,
                self._last_joint_state is not None,
                self._controller_joint in self._joint_positions(self._last_joint_state),
                self._controller_joint in self._joint_names,
                abs(self._commanded_delta_deg) <= self._max_joint_delta_deg,
                self._duration_sec <= 5.0,
                self._safety_violation_count == 0,
            ]
        )

    def _verify_gazebo_only_control_path(self) -> bool:
        topic_text = "\n".join(self._gz_topics)
        ros_topics = self._topic_names()
        ros_services = {name for name, _types in self.get_service_names_and_types()}
        robot_description = self._robot_description
        if not robot_description:
            robot_description = self._run_command(["ros2", "param", "get", "/proposal_simulation_cell_v2_0_robot_state_publisher", "robot_description"], timeout=2.0)
            robot_description = "\n".join(robot_description)
        self._robot_description = robot_description
        real_plugins = (
            "KukaRSIHardwareInterface",
            "KukaEkiRsiHardwareInterface",
            "KukaMxaRsiHardwareInterface",
            "KukaEACHardwareInterface",
        )
        self._real_robot_endpoint_detected = any(plugin in robot_description for plugin in real_plugins)
        gazebo_system_in_description = "GazeboSimSystem" in robot_description
        gazebo_controller_graph = all(
            [
                bool(topic_text.strip()),
                "/joint_states" in ros_topics,
                "/controller_manager/list_hardware_components" in ros_services,
                "/joint_trajectory_controller/follow_joint_trajectory/_action/send_goal" in ros_services,
            ]
        )
        return (gazebo_system_in_description or gazebo_controller_graph) and not self._real_robot_endpoint_detected

    def _publish_contact_wrench_sample(self) -> None:
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = "gazebo_contact_monitor"
        self._contact_wrench_pub.publish(wrench)

    def _publish_reports(self, outputs_written: bool) -> None:
        status = self._status_payload(outputs_written=outputs_written)
        self._last_status = status
        self._publish_json(self._status_pub, status)
        self._publish_json(self._delta_pub, self._joint_delta_payload())
        self._publish_json(self._safety_pub, self._safety_payload(status))

    def _status_payload(self, outputs_written: bool) -> dict[str, Any]:
        observed_delta = self._observed_delta_deg()
        motion_observed = abs(observed_delta) > 0.1
        motion_within_limit = abs(observed_delta) <= self._delta_tolerance_deg
        if self._motion_command_sent and motion_observed and motion_within_limit and self._safety_violation_count == 0:
            status = self._success_status
        elif outputs_written and not self._motion_command_sent:
            status = "gazebo_motion_controller_unavailable"
        elif not self._controller_available and self._motion_attempted:
            status = "gazebo_motion_controller_unavailable"
        elif self._safety_violation_count > 0:
            status = "gazebo_motion_safety_limit_exceeded"
        elif self._motion_attempted and not self._motion_command_sent:
            status = "gazebo_motion_controller_unavailable"
        else:
            status = "first_gazebo_motion_smoke_test_pending"
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_only_motion_test": self._gazebo_only_motion_test,
            "robot_model": self._robot_model,
            "selected_joint": self._selected_joint,
            "controller_joint": self._controller_joint,
            "commanded_joint_delta_deg": self._commanded_delta_deg if self._motion_command_sent else 0.0,
            "observed_joint_delta_deg": observed_delta,
            "motion_command_sent": self._motion_command_sent,
            "motion_observed_in_joint_states": motion_observed,
            "simulation_control_interface_used": self._simulation_control_interface if self._gazebo_verified else "unverified",
            "contact_wrench_topic_available": self._contact_wrench_topic in self._topic_names(),
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "safety_violation_count": self._safety_violation_count,
            "motion_within_limit": motion_within_limit,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "status": status if outputs_written or status != "first_gazebo_motion_smoke_test_pending" else status,
        }

    def _joint_delta_payload(self) -> dict[str, Any]:
        before = self._joint_positions(self._before_joint_state).get(self._controller_joint, 0.0)
        after = self._joint_positions(self._after_joint_state or self._last_joint_state).get(self._controller_joint, before)
        return {
            "selected_joint": self._selected_joint,
            "controller_joint": self._controller_joint,
            "before_rad": before,
            "after_rad": after,
            "observed_delta_deg": self._observed_delta_deg(),
            "commanded_delta_deg": self._commanded_delta_deg if self._motion_command_sent else 0.0,
        }

    def _safety_payload(self, status: dict[str, Any]) -> dict[str, Any]:
        return {
            "contact_wrench_topic_available": status["contact_wrench_topic_available"],
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "max_allowed_force_n": self._max_allowed_force,
            "max_allowed_torque_nm": self._max_allowed_torque,
            "emergency_stop_force_threshold_n": self._emergency_force,
            "safety_violation_count": self._safety_violation_count,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "status": status["status"],
        }

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        if self._after_joint_state is None:
            self._after_joint_state = self._last_joint_state
        self._publish_reports(outputs_written=True)
        status = self._status_payload(outputs_written=True)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", ["world", "base_link", "tool0"])
        self._write_lines(self._output_dir / "joint_states_before_motion.txt", self._joint_state_lines(self._before_joint_state))
        self._write_lines(self._output_dir / "joint_states_after_motion.txt", self._joint_state_lines(self._after_joint_state))
        self._write_joint_delta_csv()
        self._write_safety_csv(status)
        self._write_json(self._output_dir / "first_motion_smoke_test_status.json", status)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v2_0 motion smoke test diagnostics written")
        rclpy.shutdown()

    def _observed_delta_deg(self) -> float:
        if self._before_joint_state is None:
            return 0.0
        after_state = self._after_joint_state or self._last_joint_state
        if after_state is None:
            return 0.0
        before = self._joint_positions(self._before_joint_state).get(self._controller_joint)
        after = self._joint_positions(after_state).get(self._controller_joint)
        if before is None or after is None:
            return 0.0
        return math.degrees(after - before)

    def _joint_positions(self, message: JointState | None) -> dict[str, float]:
        if message is None:
            return {}
        return {name: float(position) for name, position in zip(message.name, message.position)}

    def _joint_state_lines(self, message: JointState | None) -> list[str]:
        if message is None:
            return ["joint_state_available=false"]
        lines = ["joint_state_available=true"]
        for name, position in self._joint_positions(message).items():
            lines.append(f"{name}: {position:.9f}")
        return lines

    def _write_joint_delta_csv(self) -> None:
        payload = self._joint_delta_payload()
        self._write_csv(
            self._output_dir / "first_motion_joint_delta_report.csv",
            [
                {
                    "selected_joint": str(payload["selected_joint"]),
                    "controller_joint": str(payload["controller_joint"]),
                    "before_rad": f"{payload['before_rad']:.9f}",
                    "after_rad": f"{payload['after_rad']:.9f}",
                    "commanded_delta_deg": f"{payload['commanded_delta_deg']:.6f}",
                    "observed_delta_deg": f"{payload['observed_delta_deg']:.6f}",
                }
            ],
        )

    def _write_safety_csv(self, status: dict[str, Any]) -> None:
        self._write_csv(
            self._output_dir / "first_motion_safety_report.csv",
            [
                {
                    "contact_wrench_topic_available": self._bool(status["contact_wrench_topic_available"]),
                    "max_observed_force_n": f"{self._max_force:.6f}",
                    "max_observed_torque_nm": f"{self._max_torque:.6f}",
                    "safety_violation_count": str(self._safety_violation_count),
                    "real_robot_used": "false",
                    "moveit_used": "false",
                    "compute_ik_called": "false",
                    "status": str(status["status"]),
                }
            ],
        )

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_0_first_gazebo_motion_smoke_test",
            "",
            f"Status: `{status['status']}`",
            f"Motion command sent: `{status['motion_command_sent']}`",
            f"Motion observed in joint states: `{status['motion_observed_in_joint_states']}`",
            f"Selected joint: `{status['selected_joint']}` mapped to Gazebo controller joint `{self._controller_joint}`",
            f"Observed joint delta deg: `{status['observed_joint_delta_deg']:.6f}`",
            f"Contact wrench topic available: `{status['contact_wrench_topic_available']}`",
            f"Safety violation count: `{status['safety_violation_count']}`",
            "",
            "Gazebo-only motion test. No real robot, MoveIt, /compute_ik, learning, scenario execution, Cartesian motion, peg insertion, or contact-seeking motion was used.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_0 first Gazebo motion smoke test evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"simulation_engine={status['simulation_engine']}",
            f"gazebo_only_motion_test={self._bool(status['gazebo_only_motion_test'])}",
            f"selected_joint={status['selected_joint']}",
            f"controller_joint={self._controller_joint}",
            f"motion_command_sent={self._bool(status['motion_command_sent'])}",
            f"motion_observed_in_joint_states={self._bool(status['motion_observed_in_joint_states'])}",
            f"observed_joint_delta_deg={status['observed_joint_delta_deg']:.6f}",
            f"safety_violation_count={status['safety_violation_count']}",
            "real_robot_used=false",
            "moveit_used=false",
            "compute_ik_called=false",
            "",
        ]
        (self._output_dir / "run.log").write_text("\n".join(lines), encoding="utf-8")

    def _topic_names(self) -> set[str]:
        return {name for name, _types in self.get_topic_names_and_types()}

    def _force_magnitude(self, wrench: WrenchStamped) -> float:
        force = wrench.wrench.force
        return math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z)

    def _torque_magnitude(self, wrench: WrenchStamped) -> float:
        torque = wrench.wrench.torque
        return math.sqrt(torque.x * torque.x + torque.y * torque.y + torque.z * torque.z)

    def _run_command(self, command: list[str], timeout: float) -> list[str]:
        try:
            result = subprocess.run(command, check=False, capture_output=True, text=True, timeout=timeout)
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            return [str(exc)]
        lines = []
        if result.stdout:
            lines.extend(result.stdout.splitlines())
        if result.stderr:
            lines.extend(result.stderr.splitlines())
        return lines

    def _write_csv(self, path: Path, rows: list[dict[str, str]]) -> None:
        fields = list(rows[0].keys()) if rows else ["status"]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _write_lines(self, path: Path, lines: list[str]) -> None:
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _publish_json(self, publisher: Any, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        publisher.publish(message)

    def _bool(self, value: Any) -> str:
        return str(bool(value)).lower()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV20MotionSmokeTestNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
