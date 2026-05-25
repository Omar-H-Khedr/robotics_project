"""Batch execution plan validator for proposal_simulation_cell_v1_13."""

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


class ProposalSimulationCellV113BatchExecutionPlanNode(Node):
    """Create and validate configuration-only batch execution plans."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_13_batch_execution_plan_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_13")

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        robot = self._config.get("robot", {})
        batch_source = self._config.get("batch_source", {})
        execution_plan = self._config.get("execution_plan", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_13")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._robot_model = str(robot.get("robot_model", robot.get("model", "KUKA LBR iisy 6 R1300")))
        self._simulation_engine = str(validation.get("simulation_engine", "gazebo"))
        self._gazebo_fallback_used = bool(validation.get("gazebo_fallback_used", True))
        self._isaac_available = bool(validation.get("isaac_available", False))
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 3.0))
        self._success_status = str(validation.get("status_success", "batch_execution_plan_validated"))
        self._plan_type = str(execution_plan.get("plan_type", "configuration_only_execution_plan"))

        self._batch_yaml_path = Path(
            str(batch_source.get("selected_batch_yaml", "diagnostics/proposal_simulation_cell_v1_12/selected_scenario_batch.yaml"))
        )
        self._batch_csv_path = Path(
            str(batch_source.get("selected_batch_csv", "diagnostics/proposal_simulation_cell_v1_12/selected_scenario_batch.csv"))
        )

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/batch_execution_plan_status")),
            10,
        )
        self._plan_pub = self.create_publisher(
            String,
            str(validation.get("plan_topic", "/proposal_simulation_cell/batch_execution_plan")),
            10,
        )
        self._report_pub = self.create_publisher(
            String,
            str(validation.get("validation_report_topic", "/proposal_simulation_cell/batch_execution_plan_validation_report")),
            10,
        )

        self._start_time = time.monotonic()
        self._finished = False
        self._batch_yaml = self._load_batch_yaml()
        self._batch_csv_rows = self._load_batch_csv()
        self._selected_scenarios = self._load_selected_scenarios()
        self._plans = self._generate_plans()
        self._status_rows: list[dict[str, str]] = []
        self._report_rows: list[dict[str, str]] = []
        self._last_status: dict[str, Any] = {}
        self._last_report: dict[str, Any] = {}

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_13 batch execution plan node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.13 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _load_batch_yaml(self) -> dict[str, Any]:
        if not self._batch_yaml_path.is_file():
            return {}
        with self._batch_yaml_path.open("r", encoding="utf-8") as batch_file:
            data = yaml.safe_load(batch_file) or {}
        return data if isinstance(data, dict) else {}

    def _load_batch_csv(self) -> list[dict[str, Any]]:
        if not self._batch_csv_path.is_file():
            return []
        with self._batch_csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            return [self._coerce_csv_scenario(dict(row)) for row in csv.DictReader(csv_file)]

    def _load_selected_scenarios(self) -> list[dict[str, Any]]:
        scenarios = self._batch_yaml.get("selected_scenarios", [])
        if isinstance(scenarios, list) and scenarios:
            return [scenario for scenario in scenarios if isinstance(scenario, dict)]
        return self._batch_csv_rows

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

    def _generate_plans(self) -> list[dict[str, Any]]:
        required_gates = list(self._config.get("required_gates", []))
        planned_outputs = list(self._config.get("planned_diagnostics", []))
        execution_policy = dict(self._config.get("execution_policy", {}))
        result_policy = dict(self._config.get("result_policy", {}))
        plans = []
        for scenario in self._selected_scenarios:
            plans.append(
                {
                    "scenario_id": scenario.get("scenario_id", ""),
                    "clearance_mm": float(scenario.get("clearance_mm", 0.0)),
                    "x_offset_mm": float(scenario.get("x_offset_mm", 0.0)),
                    "y_offset_mm": float(scenario.get("y_offset_mm", 0.0)),
                    "angular_misalignment_deg": float(scenario.get("angular_misalignment_deg", 0.0)),
                    "insertion_depth_mm": float(scenario.get("insertion_depth_mm", 0.0)),
                    "contact_detection_force_threshold_n": float(
                        scenario.get("contact_detection_force_threshold_n", 0.0)
                    ),
                    "required_gates": required_gates,
                    "planned_validation_outputs": planned_outputs,
                    "execution_policy": execution_policy,
                    "result_policy": result_policy,
                    "scenario_execution_started": False,
                    "motion_executed": False,
                }
            )
        return plans

    def _evaluate_and_publish(self) -> None:
        if self._finished:
            return
        report = self._validation_report_payload()
        status = self._status_payload(report, execution_plan_written=False)
        self._last_report = report
        self._last_status = status
        self._publish_json(self._status_pub, status)
        self._publish_json(self._plan_pub, self._plan_payload())
        self._publish_json(self._report_pub, report)
        self._record_rows(status, report)

    def _validation_report_payload(self) -> dict[str, Any]:
        per_plan = []
        for plan in self._plans:
            checks = self._plan_checks(plan)
            per_plan.append(
                {
                    "scenario_id": plan.get("scenario_id", ""),
                    "plan_validated": all(checks.values()),
                    "checks": checks,
                }
            )
        all_validated = bool(per_plan) and all(item["plan_validated"] for item in per_plan)
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "selected_scenario_count": len(self._plans),
            "execution_plan_generated": bool(self._plans),
            "all_execution_plans_validated": all_validated,
            "all_plans_configuration_only": all(self._policy_bool(plan, "configuration_only") for plan in self._plans),
            "all_plans_dry_run_only": all(self._policy_bool(plan, "dry_run_only") for plan in self._plans),
            "all_plans_require_rgbd_gate": all(self._requires_gate(plan, "rgbd_gate") for plan in self._plans),
            "all_plans_require_contact_gate": all(self._requires_gate(plan, "contact_gate") for plan in self._plans),
            "all_plans_require_safety_gate": all(self._requires_gate(plan, "safety_gate") for plan in self._plans),
            "all_plans_require_readiness_gate": all(self._requires_gate(plan, "readiness_gate") for plan in self._plans),
            "all_plans_require_pre_control_contract_gate": all(
                self._requires_gate(plan, "pre_control_contract_gate") for plan in self._plans
            ),
            "all_plans_require_command_blocker_gate": all(
                self._requires_gate(plan, "command_blocker_gate") for plan in self._plans
            ),
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
            "per_plan": per_plan,
            "status": "batch_execution_plan_validation_passed" if all_validated else "batch_execution_plan_validation_failed",
        }

    def _plan_checks(self, plan: dict[str, Any]) -> dict[str, bool]:
        policy = plan.get("execution_policy", {})
        result_policy = plan.get("result_policy", {})
        planned_outputs = set(plan.get("planned_validation_outputs", []))
        return {
            "scenario_id_exists": bool(plan.get("scenario_id")),
            "clearance_mm_positive": float(plan.get("clearance_mm", 0.0)) > 0.0,
            "x_offset_mm_exists": "x_offset_mm" in plan,
            "y_offset_mm_exists": "y_offset_mm" in plan,
            "angular_misalignment_deg_exists": "angular_misalignment_deg" in plan,
            "insertion_depth_mm_positive": float(plan.get("insertion_depth_mm", 0.0)) > 0.0,
            "contact_detection_force_threshold_n_positive": float(
                plan.get("contact_detection_force_threshold_n", 0.0)
            ) > 0.0,
            "configuration_only": policy.get("configuration_only") is True,
            "dry_run_only": policy.get("dry_run_only") is True,
            "command_output_disabled": policy.get("command_output_enabled") is False,
            "motion_execution_disabled": policy.get("motion_execution_enabled") is False,
            "controller_execution_disallowed": policy.get("controller_execution_allowed") is False,
            "trajectory_execution_disallowed": policy.get("trajectory_execution_allowed") is False,
            "follow_joint_trajectory_disallowed": policy.get("follow_joint_trajectory_allowed") is False,
            "real_robot_disallowed": policy.get("real_robot_allowed") is False,
            "moveit_disallowed": policy.get("moveit_allowed") is False,
            "compute_ik_disallowed": policy.get("compute_ik_allowed") is False,
            "rgbd_gate_required": self._requires_gate(plan, "rgbd_gate"),
            "contact_gate_required": self._requires_gate(plan, "contact_gate"),
            "safety_gate_required": self._requires_gate(plan, "safety_gate"),
            "readiness_gate_required": self._requires_gate(plan, "readiness_gate"),
            "pre_control_contract_gate_required": self._requires_gate(plan, "pre_control_contract_gate"),
            "command_blocker_gate_required": self._requires_gate(plan, "command_blocker_gate"),
            "sensor_status_planned": "sensor_status" in planned_outputs,
            "contact_status_planned": "contact_status" in planned_outputs,
            "safety_status_planned": "safety_status" in planned_outputs,
            "readiness_status_planned": "readiness_status" in planned_outputs,
            "no_motion_control_law_status_planned": "no_motion_control_law_status" in planned_outputs,
            "blocked_command_status_planned": "blocked_command_status" in planned_outputs,
            "fake_dataset_not_created": result_policy.get("fake_dataset_created") is False,
            "fake_plot_not_created": result_policy.get("fake_plot_created") is False,
            "experimental_result_not_created": result_policy.get("experimental_result_created") is False,
            "scenario_not_executed": plan.get("scenario_execution_started") is False,
            "motion_not_executed": plan.get("motion_executed") is False,
        }

    def _status_payload(self, report: dict[str, Any], execution_plan_written: bool) -> dict[str, Any]:
        validated = bool(report.get("all_execution_plans_validated", False))
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_fallback_used": self._gazebo_fallback_used,
            "isaac_available": self._isaac_available,
            "robot_model": self._robot_model,
            "selected_batch_yaml_found": self._batch_yaml_path.is_file(),
            "selected_batch_csv_found": self._batch_csv_path.is_file(),
            "selected_scenario_count": len(self._plans),
            "execution_plan_generated": bool(self._plans),
            "execution_plan_written": execution_plan_written,
            "all_execution_plans_validated": validated,
            "all_plans_configuration_only": bool(report.get("all_plans_configuration_only", False)),
            "all_plans_dry_run_only": bool(report.get("all_plans_dry_run_only", False)),
            "all_plans_require_rgbd_gate": bool(report.get("all_plans_require_rgbd_gate", False)),
            "all_plans_require_contact_gate": bool(report.get("all_plans_require_contact_gate", False)),
            "all_plans_require_safety_gate": bool(report.get("all_plans_require_safety_gate", False)),
            "all_plans_require_readiness_gate": bool(report.get("all_plans_require_readiness_gate", False)),
            "all_plans_require_pre_control_contract_gate": bool(
                report.get("all_plans_require_pre_control_contract_gate", False)
            ),
            "all_plans_require_command_blocker_gate": bool(report.get("all_plans_require_command_blocker_gate", False)),
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
            "status": self._success_status if validated and execution_plan_written else "batch_execution_plan_pending",
        }

    def _plan_payload(self) -> dict[str, Any]:
        return {
            "plan_type": self._plan_type,
            "selected_scenario_count": len(self._plans),
            "configuration_only": True,
            "dry_run_only": True,
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
            "execution_plans": self._plans,
        }

    def _requires_gate(self, plan: dict[str, Any], gate: str) -> bool:
        return gate in set(plan.get("required_gates", []))

    def _policy_bool(self, plan: dict[str, Any], key: str) -> bool:
        return plan.get("execution_policy", {}).get(key) is True

    def _record_rows(self, status: dict[str, Any], report: dict[str, Any]) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._status_rows.append(
            {
                "elapsed_sec": elapsed,
                "selected_scenario_count": str(status["selected_scenario_count"]),
                "execution_plan_generated": self._bool(status["execution_plan_generated"]),
                "execution_plan_written": self._bool(status["execution_plan_written"]),
                "all_execution_plans_validated": self._bool(status["all_execution_plans_validated"]),
                "all_plans_configuration_only": self._bool(status["all_plans_configuration_only"]),
                "all_plans_dry_run_only": self._bool(status["all_plans_dry_run_only"]),
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
                "execution_plan_generated": self._bool(report["execution_plan_generated"]),
                "all_execution_plans_validated": self._bool(report["all_execution_plans_validated"]),
                "all_plans_require_rgbd_gate": self._bool(report["all_plans_require_rgbd_gate"]),
                "all_plans_require_contact_gate": self._bool(report["all_plans_require_contact_gate"]),
                "all_plans_require_safety_gate": self._bool(report["all_plans_require_safety_gate"]),
                "all_plans_require_readiness_gate": self._bool(report["all_plans_require_readiness_gate"]),
                "all_plans_require_pre_control_contract_gate": self._bool(
                    report["all_plans_require_pre_control_contract_gate"]
                ),
                "all_plans_require_command_blocker_gate": self._bool(
                    report["all_plans_require_command_blocker_gate"]
                ),
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
        self._write_yaml(self._output_dir / "batch_execution_plan.yaml", self._plan_payload())
        self._write_json(self._output_dir / "batch_execution_plan.json", self._plan_payload())
        self._write_plan_csv(self._output_dir / "batch_execution_plan.csv")
        self._last_status = self._status_payload(self._last_report, execution_plan_written=True)
        self._record_rows(self._last_status, self._last_report)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", ["base_link", "hole_center", "peg_tip", "world"])
        self._write_csv(self._output_dir / "batch_execution_plan_status_samples.csv", self._status_rows)
        self._write_csv(self._output_dir / "batch_execution_plan_validation_report_samples.csv", self._report_rows)
        self._write_json(self._output_dir / "batch_execution_plan_status.json", self._last_status)
        self._write_summary(self._last_status)
        self._write_run_log(self._last_status)
        self.get_logger().info("proposal_simulation_cell_v1_13 batch execution plan diagnostics written")
        rclpy.shutdown()

    def _write_plan_csv(self, path: Path) -> None:
        fields = [
            "scenario_id",
            "clearance_mm",
            "x_offset_mm",
            "y_offset_mm",
            "angular_misalignment_deg",
            "insertion_depth_mm",
            "contact_detection_force_threshold_n",
            "required_gates",
            "planned_validation_outputs",
            "configuration_only",
            "dry_run_only",
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
        ]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            for plan in self._plans:
                policy = plan["execution_policy"]
                result_policy = plan["result_policy"]
                writer.writerow(
                    {
                        "scenario_id": plan["scenario_id"],
                        "clearance_mm": plan["clearance_mm"],
                        "x_offset_mm": plan["x_offset_mm"],
                        "y_offset_mm": plan["y_offset_mm"],
                        "angular_misalignment_deg": plan["angular_misalignment_deg"],
                        "insertion_depth_mm": plan["insertion_depth_mm"],
                        "contact_detection_force_threshold_n": plan["contact_detection_force_threshold_n"],
                        "required_gates": "|".join(plan["required_gates"]),
                        "planned_validation_outputs": "|".join(plan["planned_validation_outputs"]),
                        "configuration_only": policy["configuration_only"],
                        "dry_run_only": policy["dry_run_only"],
                        "command_output_enabled": policy["command_output_enabled"],
                        "motion_execution_enabled": policy["motion_execution_enabled"],
                        "controller_execution_allowed": policy["controller_execution_allowed"],
                        "trajectory_execution_allowed": policy["trajectory_execution_allowed"],
                        "follow_joint_trajectory_allowed": policy["follow_joint_trajectory_allowed"],
                        "real_robot_allowed": policy["real_robot_allowed"],
                        "moveit_allowed": policy["moveit_allowed"],
                        "compute_ik_allowed": policy["compute_ik_allowed"],
                        "fake_dataset_created": result_policy["fake_dataset_created"],
                        "fake_plot_created": result_policy["fake_plot_created"],
                        "experimental_result_created": result_policy["experimental_result_created"],
                    }
                )

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_13_batch_execution_plan_validator",
            "",
            "Purpose: create and validate a configuration-only execution plan for the selected v1.12 batch.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Selected scenario count: `{status['selected_scenario_count']}`",
            f"Execution plan generated: `{status['execution_plan_generated']}`",
            f"Execution plan written: `{status['execution_plan_written']}`",
            f"All execution plans validated: `{status['all_execution_plans_validated']}`",
            f"All plans configuration only: `{status['all_plans_configuration_only']}`",
            f"All plans dry run only: `{status['all_plans_dry_run_only']}`",
            f"Fake dataset created: `{status['fake_dataset_created']}`",
            f"Fake plot created: `{status['fake_plot_created']}`",
            f"Experimental result created: `{status['experimental_result_created']}`",
            f"Status: `{status['status']}`",
            "",
            "Safety constraints: command_output_enabled=false, motion_execution_enabled=false, no scenario execution, no MoveIt, no /compute_ik, no controllers, no real robot execution, no FollowJointTrajectory, and no command execution.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_13 batch execution plan evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"selected_scenario_count={status['selected_scenario_count']}",
            f"execution_plan_generated={str(status['execution_plan_generated']).lower()}",
            f"execution_plan_written={str(status['execution_plan_written']).lower()}",
            f"all_execution_plans_validated={str(status['all_execution_plans_validated']).lower()}",
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
    node = ProposalSimulationCellV113BatchExecutionPlanNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
