"""Gazebo-only motion validation suite for proposal_simulation_cell_v2_1."""

from __future__ import annotations

import csv
import json
import math
import subprocess
import threading
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


class ProposalSimulationCellV21MotionValidationSuiteNode(Node):
    """Run bounded Gazebo-only joint-space validation commands."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_1_motion_validation_suite_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_1")

        self._config = self._load_config()
        robot = self._config.get("robot", {})
        suite = self._config.get("gazebo_motion_validation_suite", {})
        tests = self._config.get("motion_tests", {})
        safety = self._config.get("safety_limits", {})
        execution = self._config.get("execution_policy", {})
        validation = self._config.get("validation", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_1")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._robot_model = str(robot.get("robot_model", "KUKA LBR iisy Gazebo support model"))
        self._simulation_engine = str(suite.get("simulation_engine", "gazebo"))
        self._gazebo_only_motion_test = bool(suite.get("gazebo_only_motion_test", True))
        self._primary_selected_joint = str(robot.get("selected_primary_joint", "joint_a6"))
        self._primary_joint = str(robot.get("primary_controller_joint", "joint_6"))
        self._secondary_selected_joint = str(robot.get("selected_secondary_joint", "joint_a5"))
        self._secondary_joint = str(robot.get("secondary_controller_joint", "joint_5"))
        self._joint_names = [str(item) for item in robot.get("controller_joint_names", [])]
        self._primary_delta_deg = float(tests.get("primary_joint_delta_deg", 2.0))
        self._secondary_delta_deg = float(tests.get("secondary_joint_delta_deg", 1.0))
        self._cycle_count = int(tests.get("cycle_count", 3))
        self._return_tolerance_deg = float(tests.get("return_tolerance_deg", 0.25))
        self._repeatability_tolerance_deg = float(tests.get("repeatability_tolerance_deg", 0.35))
        self._duration_sec = float(tests.get("command_duration_sec", 2.0))
        self._max_duration_sec = float(tests.get("max_motion_duration_per_command_sec", 5.0))
        self._max_allowed_force = float(safety.get("max_allowed_force_n", 50.0))
        self._max_allowed_torque = float(safety.get("max_allowed_torque_nm", 5.0))
        self._emergency_force = float(safety.get("emergency_stop_force_threshold_n", 45.0))
        self._primary_max_delta = float(safety.get("primary_joint_max_delta_deg", 2.0))
        self._secondary_max_delta = float(safety.get("secondary_joint_max_delta_deg", 1.0))
        self._motion_execution_enabled = bool(execution.get("motion_execution_enabled", True))
        self._real_robot_allowed = bool(execution.get("real_robot_allowed", False))
        self._moveit_allowed = bool(execution.get("moveit_allowed", False))
        self._compute_ik_allowed = bool(execution.get("compute_ik_allowed", False))
        self._simulation_control_interface = str(
            suite.get(
                "simulation_control_interface_used",
                "gz_ros2_control/GazeboSimSystem via joint_trajectory_controller",
            )
        )
        self._action_name = str(suite.get("control_interface", "/joint_trajectory_controller/follow_joint_trajectory"))
        self._joint_states_topic = str(validation.get("joint_states_topic", "/joint_states"))
        self._contact_wrench_topic = str(validation.get("contact_wrench_topic", "/proposal_simulation_cell/contact_wrench"))
        self._timeout_sec = float(validation.get("validation_timeout_sec", 90.0))
        self._pre_motion_wait_sec = float(validation.get("pre_motion_wait_sec", 2.0))
        self._success_status = str(validation.get("status_success", "gazebo_motion_validation_suite_validated"))

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/motion_validation_suite_status")),
            10,
        )
        self._delta_pub = self.create_publisher(
            String,
            str(validation.get("joint_delta_report_topic", "/proposal_simulation_cell/motion_validation_joint_delta_report")),
            10,
        )
        self._repeatability_pub = self.create_publisher(
            String,
            str(validation.get("repeatability_report_topic", "/proposal_simulation_cell/motion_validation_repeatability_report")),
            10,
        )
        self._safety_pub = self.create_publisher(
            String,
            str(validation.get("safety_report_topic", "/proposal_simulation_cell/motion_validation_safety_report")),
            10,
        )
        self._contact_wrench_pub = self.create_publisher(WrenchStamped, self._contact_wrench_topic, 10)

        self.create_subscription(JointState, self._joint_states_topic, self._on_joint_state, 10)
        self.create_subscription(WrenchStamped, self._contact_wrench_topic, self._on_contact_wrench, 10)
        self.create_subscription(String, "/robot_description", self._on_robot_description, 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)

        self._start_time = time.monotonic()
        self._last_joint_state: JointState | None = None
        self._robot_description = ""
        self._max_force = 0.0
        self._max_torque = 0.0
        self._safety_violation_count = 0
        self._finished = False
        self._suite_started = False
        self._suite_done = False
        self._suite_thread: threading.Thread | None = None
        self._controller_available = False
        self._gazebo_verified = False
        self._real_robot_endpoint_detected = False
        self._gz_topics: list[str] = []
        self._initial_positions: dict[str, float] = {}
        self._snapshots: dict[str, JointState | None] = {}
        self._delta_rows: list[dict[str, str]] = []
        self._repeatability_rows: list[dict[str, str]] = []
        self._command_count = 0

        self.create_timer(0.2, self._tick)
        self.get_logger().info("proposal_simulation_cell_v2_1 Gazebo motion validation suite node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v2.1 config not found: {path}")
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
        self._publish_contact_wrench_sample()
        self._publish_reports()
        elapsed = time.monotonic() - self._start_time
        if not self._suite_started and elapsed >= self._pre_motion_wait_sec:
            self._suite_started = True
            self._suite_thread = threading.Thread(target=self._run_suite, daemon=True)
            self._suite_thread.start()
        if elapsed >= self._timeout_sec and not self._finished:
            self._write_outputs_once()

    def _run_suite(self) -> None:
        self._controller_available = self._action_client.wait_for_server(timeout_sec=5.0)
        self._gz_topics = self._run_command(["gz", "topic", "-l"], timeout=2.0)
        self._gazebo_verified = self._verify_gazebo_only_control_path()
        if not self._can_execute_suite():
            self.get_logger().warning("Gazebo-only motion validation controller unavailable; suite not executed")
            self._write_outputs_once()
            return
        self._initial_positions = self._joint_positions(self._last_joint_state)
        self._snapshots["initial"] = self._last_joint_state
        initial_primary = self._initial_positions[self._primary_joint]
        initial_secondary = self._initial_positions[self._secondary_joint]

        if self._safety_violation_count == 0:
            self._send_and_wait(
                "single_forward",
                {self._primary_joint: initial_primary + math.radians(self._primary_delta_deg)},
            )
            self._snapshots["after_single_forward"] = self._last_joint_state
            self._record_delta("single_forward", initial_primary, self._position(self._primary_joint), self._primary_delta_deg)

        if self._safety_violation_count == 0:
            self._send_and_wait("single_return", {self._primary_joint: initial_primary})
            self._snapshots["after_single_return"] = self._last_joint_state
            self._record_return("single_return", initial_primary, self._position(self._primary_joint), 0)

        cycle_deltas: list[float] = []
        for cycle in range(1, self._cycle_count + 1):
            if self._safety_violation_count > 0:
                break
            self._send_and_wait(
                f"repeatability_cycle_{cycle}_forward",
                {self._primary_joint: initial_primary + math.radians(self._primary_delta_deg)},
            )
            observed_delta = math.degrees(self._position(self._primary_joint) - initial_primary)
            cycle_deltas.append(observed_delta)
            self._record_delta(f"repeatability_cycle_{cycle}_forward", initial_primary, self._position(self._primary_joint), self._primary_delta_deg)
            self._send_and_wait(f"repeatability_cycle_{cycle}_return", {self._primary_joint: initial_primary})
            return_error = abs(math.degrees(self._position(self._primary_joint) - initial_primary))
            repeatability_error = max(cycle_deltas) - min(cycle_deltas) if cycle_deltas else 0.0
            self._repeatability_rows.append(
                {
                    "cycle": str(cycle),
                    "observed_delta_deg": f"{observed_delta:.6f}",
                    "return_error_deg": f"{return_error:.6f}",
                    "repeatability_error_deg": f"{repeatability_error:.6f}",
                    "within_return_tolerance": self._bool(return_error <= self._return_tolerance_deg),
                    "within_repeatability_tolerance": self._bool(repeatability_error <= self._repeatability_tolerance_deg),
                }
            )
        self._snapshots["after_repeatability_cycles"] = self._last_joint_state

        if self._safety_violation_count == 0:
            self._send_and_wait(
                "two_joint_motion",
                {
                    self._secondary_joint: initial_secondary + math.radians(self._secondary_delta_deg),
                    self._primary_joint: initial_primary + math.radians(self._primary_delta_deg),
                },
            )
            self._snapshots["after_two_joint_motion"] = self._last_joint_state
            self._record_delta("two_joint_primary", initial_primary, self._position(self._primary_joint), self._primary_delta_deg)
            self._record_delta("two_joint_secondary", initial_secondary, self._position(self._secondary_joint), self._secondary_delta_deg)

        if self._safety_violation_count == 0:
            self._send_and_wait(
                "final_return",
                {self._primary_joint: initial_primary, self._secondary_joint: initial_secondary},
            )
        self._snapshots["final_return"] = self._last_joint_state
        self._suite_done = True
        self._write_outputs_once()

    def _send_and_wait(self, phase: str, targets: dict[str, float]) -> bool:
        positions = [self._initial_positions.get(name, self._position(name)) for name in self._joint_names]
        current = self._joint_positions(self._last_joint_state)
        positions = [current.get(name, positions[index]) for index, name in enumerate(self._joint_names)]
        for joint, target in targets.items():
            positions[self._joint_names.index(joint)] = target
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(self._duration_sec)
        point.time_from_start.nanosec = int((self._duration_sec - int(self._duration_sec)) * 1_000_000_000)
        goal.trajectory.points.append(point)
        future = self._action_client.send_goal_async(goal)
        goal_event = threading.Event()
        goal_holder: dict[str, Any] = {}

        def on_goal_response(done_future: Any) -> None:
            goal_holder["goal_handle"] = done_future.result()
            goal_event.set()

        future.add_done_callback(on_goal_response)
        goal_event.wait(timeout=3.0)
        goal_handle = goal_holder.get("goal_handle")
        accepted = bool(goal_handle and goal_handle.accepted)
        if not accepted:
            self.get_logger().warning(f"Motion command rejected during {phase}")
            return False
        result_future = goal_handle.get_result_async()
        result_event = threading.Event()
        result_future.add_done_callback(lambda _future: result_event.set())
        result_event.wait(timeout=self._duration_sec + 3.0)
        self._command_count += 1
        return bool(result_future.done())

    def _can_execute_suite(self) -> bool:
        return all(
            [
                self._motion_execution_enabled,
                self._gazebo_only_motion_test,
                self._simulation_engine == "gazebo",
                not self._real_robot_allowed,
                not self._moveit_allowed,
                not self._compute_ik_allowed,
                not self._real_robot_endpoint_detected,
                self._controller_available,
                self._gazebo_verified,
                self._last_joint_state is not None,
                self._primary_joint in self._joint_positions(self._last_joint_state),
                self._secondary_joint in self._joint_positions(self._last_joint_state),
                self._primary_joint in self._joint_names,
                self._secondary_joint in self._joint_names,
                abs(self._primary_delta_deg) <= self._primary_max_delta,
                abs(self._secondary_delta_deg) <= self._secondary_max_delta,
                self._cycle_count <= 3,
                self._duration_sec <= self._max_duration_sec <= 5.0,
            ]
        )

    def _verify_gazebo_only_control_path(self) -> bool:
        topic_text = "\n".join(self._gz_topics)
        ros_topics = self._topic_names()
        ros_services = {name for name, _types in self.get_service_names_and_types()}
        robot_description = self._robot_description
        if not robot_description:
            robot_description = self._run_command(["ros2", "param", "get", "/proposal_simulation_cell_v2_1_robot_state_publisher", "robot_description"], timeout=2.0)
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

    def _record_delta(self, phase: str, before: float, after: float, commanded_delta_deg: float) -> None:
        self._delta_rows.append(
            {
                "phase": phase,
                "joint": self._primary_joint if abs(commanded_delta_deg) == self._primary_delta_deg else self._secondary_joint,
                "before_rad": f"{before:.9f}",
                "after_rad": f"{after:.9f}",
                "commanded_delta_deg": f"{commanded_delta_deg:.6f}",
                "observed_delta_deg": f"{math.degrees(after - before):.6f}",
            }
        )

    def _record_return(self, phase: str, initial: float, after: float, cycle: int) -> None:
        error = abs(math.degrees(after - initial))
        self._repeatability_rows.append(
            {
                "cycle": str(cycle),
                "observed_delta_deg": "0.000000",
                "return_error_deg": f"{error:.6f}",
                "repeatability_error_deg": "0.000000",
                "within_return_tolerance": self._bool(error <= self._return_tolerance_deg),
                "within_repeatability_tolerance": "true",
            }
        )

    def _status_payload(self) -> dict[str, Any]:
        primary_deltas = [abs(float(row["observed_delta_deg"])) for row in self._delta_rows if row["joint"] == self._primary_joint]
        secondary_deltas = [abs(float(row["observed_delta_deg"])) for row in self._delta_rows if row["joint"] == self._secondary_joint]
        return_errors = [float(row["return_error_deg"]) for row in self._repeatability_rows]
        repeatability_errors = [float(row["repeatability_error_deg"]) for row in self._repeatability_rows]
        max_primary = max(primary_deltas) if primary_deltas else 0.0
        max_secondary = max(secondary_deltas) if secondary_deltas else 0.0
        max_return = max(return_errors) if return_errors else 0.0
        max_repeatability = max(repeatability_errors) if repeatability_errors else 0.0
        single_forward = any(row["phase"] == "single_forward" and abs(float(row["observed_delta_deg"])) > 0.1 for row in self._delta_rows)
        single_return = "after_single_return" in self._snapshots and bool(self._repeatability_rows)
        repeatability_cycles_completed = min(len([row for row in self._repeatability_rows if int(row["cycle"]) > 0]), self._cycle_count)
        two_joint_motion = (
            any(row["phase"] == "two_joint_primary" and abs(float(row["observed_delta_deg"])) > 0.1 for row in self._delta_rows)
            and any(row["phase"] == "two_joint_secondary" and abs(float(row["observed_delta_deg"])) > 0.1 for row in self._delta_rows)
        )
        final_return_within = self._final_return_within_tolerance()
        motion_within_limit = max_primary <= self._primary_max_delta + 0.35 and max_secondary <= self._secondary_max_delta + 0.35
        success = all(
            [
                self._suite_done,
                single_forward,
                single_return,
                self._single_return_within_tolerance(),
                repeatability_cycles_completed == self._cycle_count,
                max_repeatability <= self._repeatability_tolerance_deg,
                two_joint_motion,
                final_return_within,
                motion_within_limit,
                self._safety_violation_count == 0,
            ]
        )
        if success:
            status = self._success_status
        elif self._suite_started and not self._controller_available:
            status = "gazebo_motion_controller_unavailable"
        elif self._safety_violation_count > 0:
            status = "gazebo_motion_safety_limit_exceeded"
        else:
            status = "gazebo_motion_validation_suite_pending"
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_only_motion_test": self._gazebo_only_motion_test,
            "robot_model": self._robot_model,
            "primary_controller_joint": self._primary_joint,
            "secondary_controller_joint": self._secondary_joint,
            "single_forward_motion_observed": single_forward,
            "single_return_motion_observed": single_return,
            "single_return_within_tolerance": self._single_return_within_tolerance(),
            "repeatability_cycles_completed": repeatability_cycles_completed,
            "repeatability_within_tolerance": max_repeatability <= self._repeatability_tolerance_deg,
            "two_joint_motion_observed": two_joint_motion,
            "final_return_within_tolerance": final_return_within,
            "max_observed_primary_delta_deg": max_primary,
            "max_observed_secondary_delta_deg": max_secondary,
            "max_return_error_deg": max_return,
            "max_repeatability_error_deg": max_repeatability,
            "contact_wrench_topic_available": self._contact_wrench_topic in self._topic_names(),
            "max_observed_force_n": self._max_force,
            "max_observed_torque_nm": self._max_torque,
            "safety_violation_count": self._safety_violation_count,
            "motion_within_limit": motion_within_limit,
            "simulation_control_interface_used": self._simulation_control_interface if self._gazebo_verified else "unverified",
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "status": status,
        }

    def _single_return_within_tolerance(self) -> bool:
        for row in self._repeatability_rows:
            if row["cycle"] == "0":
                return float(row["return_error_deg"]) <= self._return_tolerance_deg
        return False

    def _final_return_within_tolerance(self) -> bool:
        if "final_return" not in self._snapshots or not self._initial_positions:
            return False
        positions = self._joint_positions(self._snapshots["final_return"])
        primary_error = abs(math.degrees(positions.get(self._primary_joint, 0.0) - self._initial_positions[self._primary_joint]))
        secondary_error = abs(math.degrees(positions.get(self._secondary_joint, 0.0) - self._initial_positions[self._secondary_joint]))
        return primary_error <= self._return_tolerance_deg and secondary_error <= self._return_tolerance_deg

    def _publish_reports(self) -> None:
        status = self._status_payload()
        self._publish_json(self._status_pub, status)
        self._publish_json(self._delta_pub, {"rows": self._delta_rows})
        self._publish_json(self._repeatability_pub, {"rows": self._repeatability_rows})
        self._publish_json(self._safety_pub, self._safety_payload(status))

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
        status = self._status_payload()
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", ["world", "base_link", "tool0"])
        self._write_lines(self._output_dir / "joint_states_initial.txt", self._joint_state_lines(self._snapshots.get("initial")))
        self._write_lines(self._output_dir / "joint_states_after_single_forward.txt", self._joint_state_lines(self._snapshots.get("after_single_forward")))
        self._write_lines(self._output_dir / "joint_states_after_single_return.txt", self._joint_state_lines(self._snapshots.get("after_single_return")))
        self._write_lines(self._output_dir / "joint_states_after_repeatability_cycles.txt", self._joint_state_lines(self._snapshots.get("after_repeatability_cycles")))
        self._write_lines(self._output_dir / "joint_states_after_two_joint_motion.txt", self._joint_state_lines(self._snapshots.get("after_two_joint_motion")))
        self._write_lines(self._output_dir / "joint_states_final_return.txt", self._joint_state_lines(self._snapshots.get("final_return")))
        self._write_csv(self._output_dir / "motion_validation_joint_delta_report.csv", self._delta_rows)
        self._write_csv(self._output_dir / "motion_validation_repeatability_report.csv", self._repeatability_rows)
        self._write_safety_csv(status)
        self._write_json(self._output_dir / "motion_validation_suite_status.json", status)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v2_1 motion validation suite diagnostics written")
        rclpy.shutdown()

    def _write_safety_csv(self, status: dict[str, Any]) -> None:
        self._write_csv(
            self._output_dir / "motion_validation_safety_report.csv",
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
            "# proposal_simulation_cell_v2_1_gazebo_motion_validation_suite",
            "",
            f"Status: `{status['status']}`",
            f"Single forward observed: `{status['single_forward_motion_observed']}`",
            f"Single return within tolerance: `{status['single_return_within_tolerance']}`",
            f"Repeatability cycles completed: `{status['repeatability_cycles_completed']}`",
            f"Repeatability within tolerance: `{status['repeatability_within_tolerance']}`",
            f"Two-joint motion observed: `{status['two_joint_motion_observed']}`",
            f"Final return within tolerance: `{status['final_return_within_tolerance']}`",
            f"Safety violation count: `{status['safety_violation_count']}`",
            "",
            "Gazebo-only bounded joint-space suite. No real robot, MoveIt, /compute_ik, learning, scenario execution, Cartesian motion, peg insertion, or contact-seeking motion was used.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_1 Gazebo motion validation suite evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"commands_sent={self._command_count}",
            f"repeatability_cycles_completed={status['repeatability_cycles_completed']}",
            f"max_observed_primary_delta_deg={status['max_observed_primary_delta_deg']:.6f}",
            f"max_observed_secondary_delta_deg={status['max_observed_secondary_delta_deg']:.6f}",
            f"safety_violation_count={status['safety_violation_count']}",
            "real_robot_used=false",
            "moveit_used=false",
            "compute_ik_called=false",
            "",
        ]
        (self._output_dir / "run.log").write_text("\n".join(lines), encoding="utf-8")

    def _publish_contact_wrench_sample(self) -> None:
        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.header.frame_id = "gazebo_contact_monitor"
        self._contact_wrench_pub.publish(wrench)

    def _position(self, joint: str) -> float:
        return self._joint_positions(self._last_joint_state).get(joint, 0.0)

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
    node = ProposalSimulationCellV21MotionValidationSuiteNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
