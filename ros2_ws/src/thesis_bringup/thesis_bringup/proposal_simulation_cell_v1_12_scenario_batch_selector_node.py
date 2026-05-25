"""Scenario batch selector for proposal_simulation_cell_v1_12."""

from __future__ import annotations

import csv
import json
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


class ProposalSimulationCellV112ScenarioBatchSelectorNode(Node):
    """Select and validate a representative configuration-only scenario batch."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_12_scenario_batch_selector_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_12")

        self._config = self._load_config()
        diagnostics = self._config.get("diagnostics", {})
        robot = self._config.get("robot", {})
        scenario_source = self._config.get("scenario_source", {})
        selected_batch = self._config.get("selected_batch", {})
        selection_policy = self._config.get("selection_policy", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or diagnostics.get("output_dir", "diagnostics/proposal_simulation_cell_v1_12")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._robot_model = str(robot.get("robot_model", robot.get("model", "KUKA LBR iisy 6 R1300")))
        self._simulation_engine = str(diagnostics.get("simulation_engine", "gazebo"))
        self._gazebo_fallback_used = bool(diagnostics.get("gazebo_fallback_used", True))
        self._isaac_available = bool(diagnostics.get("isaac_available", False))
        self._sample_period = float(diagnostics.get("sample_period_sec", 0.2))
        self._validation_duration = float(diagnostics.get("validation_duration_sec", 3.0))
        self._success_status = str(diagnostics.get("status_success", "scenario_batch_selector_validated"))
        self._selection_policy_name = str(selection_policy.get("name", "representative_configuration_only_batch"))

        self._matrix_yaml_path = Path(
            str(scenario_source.get("scenario_matrix_yaml", "diagnostics/proposal_simulation_cell_v1_10/experiment_configuration_matrix.yaml"))
        )
        self._matrix_csv_path = Path(
            str(scenario_source.get("scenario_matrix_csv", "diagnostics/proposal_simulation_cell_v1_10/experiment_configuration_matrix.csv"))
        )
        ids = selected_batch.get("selected_scenario_ids", [])
        self._selected_scenario_ids = [str(scenario_id) for scenario_id in ids] or [
            "v1_10_scenario_001",
            "v1_10_scenario_365",
            "v1_10_scenario_729",
        ]

        self._status_pub = self.create_publisher(
            String,
            str(diagnostics.get("status_topic", "/proposal_simulation_cell/scenario_batch_status")),
            10,
        )
        self._batch_pub = self.create_publisher(
            String,
            str(diagnostics.get("selected_batch_topic", "/proposal_simulation_cell/selected_scenario_batch")),
            10,
        )
        self._report_pub = self.create_publisher(
            String,
            str(diagnostics.get("validation_report_topic", "/proposal_simulation_cell/scenario_batch_validation_report")),
            10,
        )

        self._start_time = time.monotonic()
        self._finished = False
        self._matrix_yaml = self._load_matrix_yaml()
        self._matrix_csv_rows = self._load_matrix_csv()
        self._selected_scenarios = self._select_scenarios()
        self._status_rows: list[dict[str, str]] = []
        self._report_rows: list[dict[str, str]] = []
        self._last_status: dict[str, Any] = {}
        self._last_report: dict[str, Any] = {}

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_12 scenario batch selector node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.12 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _load_matrix_yaml(self) -> dict[str, Any]:
        if not self._matrix_yaml_path.is_file():
            return {}
        with self._matrix_yaml_path.open("r", encoding="utf-8") as matrix_file:
            data = yaml.safe_load(matrix_file) or {}
        return data if isinstance(data, dict) else {}

    def _load_matrix_csv(self) -> list[dict[str, Any]]:
        if not self._matrix_csv_path.is_file():
            return []
        with self._matrix_csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            return [dict(row) for row in csv.DictReader(csv_file)]

    def _select_scenarios(self) -> list[dict[str, Any]]:
        selected = []
        yaml_scenarios = self._matrix_yaml.get("scenarios", [])
        yaml_by_id = {
            scenario.get("scenario_id"): scenario
            for scenario in yaml_scenarios
            if isinstance(scenario, dict) and scenario.get("scenario_id")
        } if isinstance(yaml_scenarios, list) else {}
        csv_by_id = {
            scenario.get("scenario_id"): self._coerce_csv_scenario(scenario)
            for scenario in self._matrix_csv_rows
            if scenario.get("scenario_id")
        }
        for scenario_id in self._selected_scenario_ids:
            selected.append(yaml_by_id.get(scenario_id) or csv_by_id.get(scenario_id) or {"scenario_id": scenario_id})
        return selected

    def _coerce_csv_scenario(self, scenario: dict[str, Any]) -> dict[str, Any]:
        bool_fields = {
            "require_rgbd",
            "require_contact_gate",
            "require_safety_gate",
            "require_readiness_gate",
            "require_pre_control_contract",
            "command_output_enabled",
            "motion_execution_enabled",
            "controller_execution_allowed",
            "trajectory_execution_allowed",
            "follow_joint_trajectory_allowed",
            "real_robot_allowed",
            "moveit_allowed",
            "compute_ik_allowed",
            "fake_dataset_created",
            "fake_plot_created",
            "experimental_result_created",
        }
        float_fields = {
            "clearance_mm",
            "x_offset_mm",
            "y_offset_mm",
            "angular_misalignment_deg",
            "insertion_depth_mm",
            "contact_detection_force_threshold_n",
            "max_allowed_force_n",
            "max_allowed_torque_nm",
        }
        coerced: dict[str, Any] = {}
        for key, value in scenario.items():
            if key in bool_fields:
                coerced[key] = str(value).lower() == "true"
            elif key in float_fields:
                coerced[key] = float(value)
            else:
                coerced[key] = value
        return coerced

    def _evaluate_and_publish(self) -> None:
        if self._finished:
            return
        report = self._validation_report_payload()
        status = self._status_payload(report, selected_batch_written=False)
        self._last_report = report
        self._last_status = status
        self._publish_json(self._status_pub, status)
        self._publish_json(self._batch_pub, self._batch_payload())
        self._publish_json(self._report_pub, report)
        self._record_rows(status, report)

    def _validation_report_payload(self) -> dict[str, Any]:
        per_scenario = []
        for scenario in self._selected_scenarios:
            checks = self._scenario_checks(scenario)
            per_scenario.append(
                {
                    "scenario_id": scenario.get("scenario_id", ""),
                    "scenario_found": self._scenario_found(scenario),
                    "scenario_validated": self._scenario_found(scenario) and all(checks.values()),
                    "checks": checks,
                }
            )
        all_found = all(item["scenario_found"] for item in per_scenario)
        all_validated = all(item["scenario_validated"] for item in per_scenario)
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "selected_scenario_ids": self._selected_scenario_ids,
            "selected_scenario_count": len(self._selected_scenarios),
            "all_selected_scenarios_found": all_found,
            "all_selected_scenarios_validated": all_validated,
            "batch_is_configuration_only": True,
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
            "per_scenario": per_scenario,
            "status": "scenario_batch_validation_passed" if all_found and all_validated else "scenario_batch_validation_failed",
        }

    def _scenario_checks(self, scenario: dict[str, Any]) -> dict[str, bool]:
        return {
            "clearance_mm_positive": self._number_positive(scenario, "clearance_mm"),
            "x_offset_mm_exists": "x_offset_mm" in scenario,
            "y_offset_mm_exists": "y_offset_mm" in scenario,
            "angular_misalignment_deg_exists": "angular_misalignment_deg" in scenario,
            "insertion_depth_mm_positive": self._number_positive(scenario, "insertion_depth_mm"),
            "contact_detection_force_threshold_n_positive": self._number_positive(
                scenario, "contact_detection_force_threshold_n"
            ),
            "max_allowed_force_n_positive": self._number_positive(scenario, "max_allowed_force_n"),
            "max_allowed_torque_nm_positive": self._number_positive(scenario, "max_allowed_torque_nm"),
            "require_rgbd": scenario.get("require_rgbd") is True,
            "require_contact_gate": scenario.get("require_contact_gate") is True,
            "require_safety_gate": scenario.get("require_safety_gate") is True,
            "require_readiness_gate": scenario.get("require_readiness_gate") is True,
            "require_pre_control_contract": scenario.get("require_pre_control_contract") is True,
            "command_output_disabled": scenario.get("command_output_enabled") is False,
            "motion_execution_disabled": scenario.get("motion_execution_enabled") is False,
            "controller_execution_disallowed": scenario.get("controller_execution_allowed") is False,
            "trajectory_execution_disallowed": scenario.get("trajectory_execution_allowed") is False,
            "follow_joint_trajectory_disallowed": scenario.get("follow_joint_trajectory_allowed") is False,
            "real_robot_disallowed": scenario.get("real_robot_allowed") is False,
            "moveit_disallowed": scenario.get("moveit_allowed") is False,
            "compute_ik_disallowed": scenario.get("compute_ik_allowed") is False,
            "fake_dataset_not_created": scenario.get("fake_dataset_created") is False,
            "fake_plot_not_created": scenario.get("fake_plot_created") is False,
            "experimental_result_not_created": scenario.get("experimental_result_created") is False,
        }

    def _status_payload(self, report: dict[str, Any], selected_batch_written: bool) -> dict[str, Any]:
        validated = bool(report.get("all_selected_scenarios_found")) and bool(
            report.get("all_selected_scenarios_validated")
        )
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_fallback_used": self._gazebo_fallback_used,
            "isaac_available": self._isaac_available,
            "robot_model": self._robot_model,
            "scenario_matrix_yaml_found": self._matrix_yaml_path.is_file(),
            "scenario_matrix_csv_found": self._matrix_csv_path.is_file(),
            "selected_scenario_ids": self._selected_scenario_ids,
            "selected_scenario_count": len(self._selected_scenarios),
            "all_selected_scenarios_found": bool(report.get("all_selected_scenarios_found", False)),
            "all_selected_scenarios_validated": bool(report.get("all_selected_scenarios_validated", False)),
            "selected_batch_written": selected_batch_written,
            "batch_is_configuration_only": True,
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "trajectory_execution_allowed": False,
            "follow_joint_trajectory_allowed": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "status": self._success_status if validated and selected_batch_written else "scenario_batch_selector_pending",
        }

    def _batch_payload(self) -> dict[str, Any]:
        return {
            "selected_scenario_ids": self._selected_scenario_ids,
            "selected_scenario_count": len(self._selected_scenarios),
            "selection_policy": self._selection_policy_name,
            "configuration_only": True,
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
            "selected_scenarios": self._selected_scenarios,
        }

    def _scenario_found(self, scenario: dict[str, Any]) -> bool:
        return "clearance_mm" in scenario and "contact_detection_force_threshold_n" in scenario

    def _number_positive(self, scenario: dict[str, Any], key: str) -> bool:
        try:
            return float(scenario[key]) > 0.0
        except (KeyError, TypeError, ValueError):
            return False

    def _record_rows(self, status: dict[str, Any], report: dict[str, Any]) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._status_rows.append(
            {
                "elapsed_sec": elapsed,
                "selected_scenario_count": str(status["selected_scenario_count"]),
                "all_selected_scenarios_found": self._bool(status["all_selected_scenarios_found"]),
                "all_selected_scenarios_validated": self._bool(status["all_selected_scenarios_validated"]),
                "selected_batch_written": self._bool(status["selected_batch_written"]),
                "batch_is_configuration_only": self._bool(status["batch_is_configuration_only"]),
                "command_output_enabled": self._bool(status["command_output_enabled"]),
                "motion_execution_enabled": self._bool(status["motion_execution_enabled"]),
                "fake_dataset_created": self._bool(status["fake_dataset_created"]),
                "fake_plot_created": self._bool(status["fake_plot_created"]),
                "experimental_result_created": self._bool(status["experimental_result_created"]),
                "status": str(status["status"]),
            }
        )
        self._report_rows.append(
            {
                "elapsed_sec": elapsed,
                "selected_scenario_count": str(report["selected_scenario_count"]),
                "all_selected_scenarios_found": self._bool(report["all_selected_scenarios_found"]),
                "all_selected_scenarios_validated": self._bool(report["all_selected_scenarios_validated"]),
                "batch_is_configuration_only": self._bool(report["batch_is_configuration_only"]),
                "fake_dataset_created": self._bool(report["fake_dataset_created"]),
                "fake_plot_created": self._bool(report["fake_plot_created"]),
                "experimental_result_created": self._bool(report["experimental_result_created"]),
                "status": str(report["status"]),
            }
        )

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        if not self._last_report:
            self._last_report = self._validation_report_payload()
        self._write_yaml(self._output_dir / "selected_scenario_batch.yaml", self._batch_payload())
        self._write_json(self._output_dir / "selected_scenario_batch.json", self._batch_payload())
        self._write_batch_csv(self._output_dir / "selected_scenario_batch.csv")
        self._last_status = self._status_payload(self._last_report, selected_batch_written=True)
        self._record_rows(self._last_status, self._last_report)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", ["base_link", "hole_center", "peg_tip", "world"])
        self._write_csv(self._output_dir / "scenario_batch_status_samples.csv", self._status_rows)
        self._write_csv(self._output_dir / "scenario_batch_validation_report_samples.csv", self._report_rows)
        self._write_json(self._output_dir / "scenario_batch_selector_status.json", self._last_status)
        self._write_summary(self._last_status)
        self._write_run_log(self._last_status)
        self.get_logger().info("proposal_simulation_cell_v1_12 scenario batch diagnostics written")
        rclpy.shutdown()

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_12_scenario_batch_selector",
            "",
            "Purpose: select and validate a representative configuration-only scenario batch.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Selected scenario count: `{status['selected_scenario_count']}`",
            f"All selected scenarios found: `{status['all_selected_scenarios_found']}`",
            f"All selected scenarios validated: `{status['all_selected_scenarios_validated']}`",
            f"Selected batch written: `{status['selected_batch_written']}`",
            f"Batch is configuration only: `{status['batch_is_configuration_only']}`",
            f"Fake dataset created: `{status['fake_dataset_created']}`",
            f"Fake plot created: `{status['fake_plot_created']}`",
            f"Experimental result created: `{status['experimental_result_created']}`",
            f"Status: `{status['status']}`",
            "",
            "Safety constraints: command_output_enabled=false, motion_execution_enabled=false, no MoveIt, no /compute_ik, no controllers, no real robot execution, no FollowJointTrajectory, and no command execution.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_12 scenario batch selector evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"selected_scenario_count={status['selected_scenario_count']}",
            f"all_selected_scenarios_found={str(status['all_selected_scenarios_found']).lower()}",
            f"all_selected_scenarios_validated={str(status['all_selected_scenarios_validated']).lower()}",
            f"selected_batch_written={str(status['selected_batch_written']).lower()}",
            "fake_dataset_created=false",
            "fake_plot_created=false",
            "experimental_result_created=false",
            "command_output_enabled=false",
            "motion_execution_enabled=false",
            "controller_execution_allowed=false",
            "trajectory_execution_allowed=false",
            "follow_joint_trajectory_allowed=false",
            "",
        ]
        (self._output_dir / "run.log").write_text("\n".join(lines), encoding="utf-8")

    def _write_batch_csv(self, path: Path) -> None:
        fields = list(self._selected_scenarios[0].keys()) if self._selected_scenarios else ["scenario_id"]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            writer.writerows(self._selected_scenarios)

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
    node = ProposalSimulationCellV112ScenarioBatchSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
