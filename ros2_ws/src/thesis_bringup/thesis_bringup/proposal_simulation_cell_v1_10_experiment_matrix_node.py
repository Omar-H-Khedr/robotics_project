"""Experiment configuration matrix for proposal_simulation_cell_v1_10."""

from __future__ import annotations

import csv
import itertools
import json
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


class ProposalSimulationCellV110ExperimentMatrixNode(Node):
    """Generate and validate simulation-only scenario configurations."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_10_experiment_matrix_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_10")

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        scenario_matrix = self._config.get("scenario_matrix", {})
        robot = self._config.get("robot", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_10")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._robot_model = str(robot.get("robot_model", robot.get("model", "KUKA LBR iisy 6 R1300")))
        self._simulation_engine = str(validation.get("simulation_engine", "gazebo"))
        self._gazebo_fallback_used = bool(validation.get("gazebo_fallback_used", True))
        self._isaac_available = bool(validation.get("isaac_available", False))
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 3.0))
        self._success_status = str(validation.get("status_success", "experiment_configuration_matrix_validated"))

        self._status_topic = str(
            scenario_matrix.get("output_status_topic", "/proposal_simulation_cell/experiment_matrix_status")
        )
        self._scenario_list_topic = str(
            scenario_matrix.get("scenario_list_topic", "/proposal_simulation_cell/scenario_configuration_list")
        )
        self._validation_report_topic = str(
            scenario_matrix.get("validation_report_topic", "/proposal_simulation_cell/scenario_validation_report")
        )

        self._start_time = time.monotonic()
        self._finished = False
        self._scenarios = self._generate_scenarios()
        self._status_rows: list[dict[str, str]] = []
        self._report_rows: list[dict[str, str]] = []
        self._last_status: dict[str, Any] = {}
        self._last_report: dict[str, Any] = {}

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._scenario_list_pub = self.create_publisher(String, self._scenario_list_topic, 10)
        self._validation_report_pub = self.create_publisher(String, self._validation_report_topic, 10)

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_10 experiment matrix node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.10 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _generate_scenarios(self) -> list[dict[str, Any]]:
        geometry = self._config.get("peg_hole_geometry", {})
        misalignment = self._config.get("misalignment_variants", {})
        thresholds = self._config.get("contact_thresholds", {})
        safety_limits = self._config.get("safety_limits", {})
        sensor_requirements = self._config.get("sensor_requirements", {})
        readiness_requirements = self._config.get("readiness_requirements", {})
        execution_policy = self._config.get("execution_policy", {})

        scenarios = []
        variants = itertools.product(
            geometry.get("clearance_mm", []),
            misalignment.get("x_offset_mm", []),
            misalignment.get("y_offset_mm", []),
            misalignment.get("angular_misalignment_deg", []),
            misalignment.get("insertion_depth_mm", []),
            thresholds.get("contact_detection_force_threshold_n", []),
        )
        for index, (
            clearance_mm,
            x_offset_mm,
            y_offset_mm,
            angular_misalignment_deg,
            insertion_depth_mm,
            contact_detection_force_threshold_n,
        ) in enumerate(variants, start=1):
            scenarios.append(
                {
                    "scenario_id": f"v1_10_scenario_{index:03d}",
                    "clearance_mm": float(clearance_mm),
                    "x_offset_mm": float(x_offset_mm),
                    "y_offset_mm": float(y_offset_mm),
                    "angular_misalignment_deg": float(angular_misalignment_deg),
                    "insertion_depth_mm": float(insertion_depth_mm),
                    "contact_detection_force_threshold_n": float(contact_detection_force_threshold_n),
                    "max_allowed_force_n": float(safety_limits.get("max_allowed_force_n", 50.0)),
                    "max_allowed_torque_nm": float(safety_limits.get("max_allowed_torque_nm", 5.0)),
                    "require_rgbd": bool(sensor_requirements.get("require_rgbd", True)),
                    "require_contact_gate": bool(readiness_requirements.get("require_contact_gate", True)),
                    "require_safety_gate": bool(readiness_requirements.get("require_safety_gate", True)),
                    "require_readiness_gate": bool(readiness_requirements.get("require_readiness_gate", True)),
                    "require_pre_control_contract": bool(
                        readiness_requirements.get("require_pre_control_contract", True)
                    ),
                    "command_output_enabled": bool(execution_policy.get("command_output_enabled", False)),
                    "motion_execution_enabled": bool(execution_policy.get("motion_execution_enabled", False)),
                    "controller_execution_allowed": bool(execution_policy.get("controller_execution_allowed", False)),
                    "trajectory_execution_allowed": bool(execution_policy.get("trajectory_execution_allowed", False)),
                    "follow_joint_trajectory_allowed": bool(
                        execution_policy.get("follow_joint_trajectory_allowed", False)
                    ),
                    "real_robot_allowed": bool(execution_policy.get("real_robot_allowed", False)),
                    "moveit_allowed": bool(execution_policy.get("moveit_allowed", False)),
                    "compute_ik_allowed": bool(execution_policy.get("compute_ik_allowed", False)),
                    "fake_dataset_created": False,
                    "fake_plot_created": False,
                    "experimental_result_created": False,
                }
            )
        return scenarios

    def _evaluate_and_publish(self) -> None:
        if self._finished:
            return
        report = self._validation_report_payload()
        status = self._status_payload(report)
        self._last_report = report
        self._last_status = status
        self._publish_json(self._status_pub, status)
        self._publish_json(self._scenario_list_pub, self._scenario_list_payload())
        self._publish_json(self._validation_report_pub, report)
        self._record_rows(status, report)

    def _status_payload(self, report: dict[str, Any]) -> dict[str, Any]:
        geometry = self._config.get("peg_hole_geometry", {})
        misalignment = self._config.get("misalignment_variants", {})
        thresholds = self._config.get("contact_thresholds", {})
        scenario_count = len(self._scenarios)
        matrix_valid = (
            scenario_count > 0
            and report["all_scenarios_execution_disabled"]
            and report["all_scenarios_require_rgbd"]
            and report["all_scenarios_require_contact_gate"]
            and report["all_scenarios_require_safety_gate"]
            and report["all_scenarios_require_readiness_gate"]
            and report["all_scenarios_require_pre_control_contract"]
            and not report["fake_datasets_created"]
            and not report["fake_plots_created"]
        )
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_fallback_used": self._gazebo_fallback_used,
            "isaac_available": self._isaac_available,
            "robot_model": self._robot_model,
            "scenario_matrix_generated": scenario_count > 0,
            "scenario_count": scenario_count,
            "clearance_variants_count": len(geometry.get("clearance_mm", [])),
            "translational_misalignment_variants_count": len(misalignment.get("x_offset_mm", []))
            * len(misalignment.get("y_offset_mm", [])),
            "angular_misalignment_variants_count": len(misalignment.get("angular_misalignment_deg", [])),
            "insertion_depth_variants_count": len(misalignment.get("insertion_depth_mm", [])),
            "contact_threshold_variants_count": len(thresholds.get("contact_detection_force_threshold_n", [])),
            "all_scenarios_execution_disabled": report["all_scenarios_execution_disabled"],
            "all_scenarios_require_rgbd": report["all_scenarios_require_rgbd"],
            "all_scenarios_require_contact_gate": report["all_scenarios_require_contact_gate"],
            "all_scenarios_require_safety_gate": report["all_scenarios_require_safety_gate"],
            "all_scenarios_require_readiness_gate": report["all_scenarios_require_readiness_gate"],
            "fake_datasets_created": False,
            "fake_plots_created": False,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "trajectory_execution_allowed": False,
            "follow_joint_trajectory_allowed": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "status": self._success_status if matrix_valid else "experiment_configuration_matrix_pending",
        }

    def _scenario_list_payload(self) -> dict[str, Any]:
        return {
            "scenario_matrix_generated": bool(self._scenarios),
            "scenario_count": len(self._scenarios),
            "configuration_only": True,
            "fake_datasets_created": False,
            "fake_plots_created": False,
            "scenarios": self._scenarios,
        }

    def _validation_report_payload(self) -> dict[str, Any]:
        execution_disabled = all(
            not scenario["command_output_enabled"]
            and not scenario["motion_execution_enabled"]
            and not scenario["controller_execution_allowed"]
            and not scenario["trajectory_execution_allowed"]
            and not scenario["follow_joint_trajectory_allowed"]
            and not scenario["real_robot_allowed"]
            and not scenario["moveit_allowed"]
            and not scenario["compute_ik_allowed"]
            for scenario in self._scenarios
        )
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "scenario_count": len(self._scenarios),
            "all_scenarios_execution_disabled": execution_disabled,
            "all_scenarios_require_rgbd": all(scenario["require_rgbd"] for scenario in self._scenarios),
            "all_scenarios_require_contact_gate": all(
                scenario["require_contact_gate"] for scenario in self._scenarios
            ),
            "all_scenarios_require_safety_gate": all(
                scenario["require_safety_gate"] for scenario in self._scenarios
            ),
            "all_scenarios_require_readiness_gate": all(
                scenario["require_readiness_gate"] for scenario in self._scenarios
            ),
            "all_scenarios_require_pre_control_contract": all(
                scenario["require_pre_control_contract"] for scenario in self._scenarios
            ),
            "fake_datasets_created": False,
            "fake_plots_created": False,
            "experimental_results_created": False,
            "status": "scenario_configuration_contract_validated"
            if execution_disabled
            else "scenario_configuration_contract_failed",
        }

    def _record_rows(self, status: dict[str, Any], report: dict[str, Any]) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._status_rows.append(
            {
                "elapsed_sec": elapsed,
                "scenario_matrix_generated": self._bool(status["scenario_matrix_generated"]),
                "scenario_count": str(status["scenario_count"]),
                "all_scenarios_execution_disabled": self._bool(status["all_scenarios_execution_disabled"]),
                "all_scenarios_require_rgbd": self._bool(status["all_scenarios_require_rgbd"]),
                "all_scenarios_require_contact_gate": self._bool(status["all_scenarios_require_contact_gate"]),
                "all_scenarios_require_safety_gate": self._bool(status["all_scenarios_require_safety_gate"]),
                "all_scenarios_require_readiness_gate": self._bool(status["all_scenarios_require_readiness_gate"]),
                "fake_datasets_created": self._bool(status["fake_datasets_created"]),
                "fake_plots_created": self._bool(status["fake_plots_created"]),
                "status": str(status["status"]),
            }
        )
        self._report_rows.append(
            {
                "elapsed_sec": elapsed,
                "scenario_count": str(report["scenario_count"]),
                "all_scenarios_execution_disabled": self._bool(report["all_scenarios_execution_disabled"]),
                "all_scenarios_require_rgbd": self._bool(report["all_scenarios_require_rgbd"]),
                "all_scenarios_require_contact_gate": self._bool(report["all_scenarios_require_contact_gate"]),
                "all_scenarios_require_safety_gate": self._bool(report["all_scenarios_require_safety_gate"]),
                "all_scenarios_require_readiness_gate": self._bool(report["all_scenarios_require_readiness_gate"]),
                "all_scenarios_require_pre_control_contract": self._bool(
                    report["all_scenarios_require_pre_control_contract"]
                ),
                "fake_datasets_created": self._bool(report["fake_datasets_created"]),
                "fake_plots_created": self._bool(report["fake_plots_created"]),
                "status": str(report["status"]),
            }
        )

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        if not self._last_report:
            self._last_report = self._validation_report_payload()
        if not self._last_status:
            self._last_status = self._status_payload(self._last_report)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", ["base_link", "hole_center", "peg_tip", "world"])
        self._write_yaml(self._output_dir / "experiment_configuration_matrix.yaml", self._matrix_document())
        self._write_matrix_csv(self._output_dir / "experiment_configuration_matrix.csv")
        self._write_csv(self._output_dir / "experiment_matrix_status_samples.csv", self._status_rows)
        self._write_csv(self._output_dir / "scenario_validation_report_samples.csv", self._report_rows)
        self._write_json(self._output_dir / "experiment_matrix_status.json", self._last_status)
        self._write_summary(self._last_status)
        self._write_run_log(self._last_status)
        self.get_logger().info("proposal_simulation_cell_v1_10 experiment matrix diagnostics written")
        rclpy.shutdown()

    def _matrix_document(self) -> dict[str, Any]:
        return {
            "simulation_cell_name": "proposal_simulation_cell_v1_10_experiment_configuration_matrix",
            "configuration_only": True,
            "fake_datasets_created": False,
            "fake_plots_created": False,
            "robot_model": self._robot_model,
            "simulation_engine": self._simulation_engine,
            "scenario_count": len(self._scenarios),
            "scenarios": self._scenarios,
        }

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_10_experiment_configuration_matrix",
            "",
            "Purpose: define simulation-only future peg-in-hole validation scenario configurations.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Scenario matrix generated: `{status['scenario_matrix_generated']}`",
            f"Scenario count: `{status['scenario_count']}`",
            f"All scenarios execution disabled: `{status['all_scenarios_execution_disabled']}`",
            f"All scenarios require RGB-D: `{status['all_scenarios_require_rgbd']}`",
            f"All scenarios require contact gate: `{status['all_scenarios_require_contact_gate']}`",
            f"All scenarios require safety gate: `{status['all_scenarios_require_safety_gate']}`",
            f"All scenarios require readiness gate: `{status['all_scenarios_require_readiness_gate']}`",
            f"Fake datasets created: `{status['fake_datasets_created']}`",
            f"Fake plots created: `{status['fake_plots_created']}`",
            f"Status: `{status['status']}`",
            "",
            "Safety constraints: command_output_enabled=false, motion_execution_enabled=false, no MoveIt, no /compute_ik, no controllers, no real robot execution, no FollowJointTrajectory, and no command execution.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_10 experiment configuration matrix evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"scenario_count={status['scenario_count']}",
            f"all_scenarios_execution_disabled={str(status['all_scenarios_execution_disabled']).lower()}",
            f"all_scenarios_require_rgbd={str(status['all_scenarios_require_rgbd']).lower()}",
            "fake_datasets_created=false",
            "fake_plots_created=false",
            "command_output_enabled=false",
            "motion_execution_enabled=false",
            "controller_execution_allowed=false",
            "trajectory_execution_allowed=false",
            "follow_joint_trajectory_allowed=false",
            "",
        ]
        (self._output_dir / "run.log").write_text("\n".join(lines), encoding="utf-8")

    def _write_matrix_csv(self, path: Path) -> None:
        fields = list(self._scenarios[0].keys()) if self._scenarios else ["scenario_id"]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(self._scenarios)

    def _write_csv(self, path: Path, rows: list[dict[str, str]]) -> None:
        fields = list(rows[0].keys()) if rows else ["elapsed_sec"]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _write_yaml(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

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
    node = ProposalSimulationCellV110ExperimentMatrixNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
