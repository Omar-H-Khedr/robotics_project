"""Batch dry-run orchestrator for proposal_simulation_cell_v1_14."""

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


class ProposalSimulationCellV114BatchDryRunOrchestratorNode(Node):
    """Create blocked dry-run orchestration records without scenario execution."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_14_batch_dry_run_orchestrator_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_14")

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        robot = self._config.get("robot", {})
        source = self._config.get("batch_execution_plan_source", {})
        orchestration = self._config.get("dry_run_orchestration", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_14")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._robot_model = str(robot.get("robot_model", robot.get("model", "KUKA LBR iisy 6 R1300")))
        self._simulation_engine = str(validation.get("simulation_engine", "gazebo"))
        self._gazebo_fallback_used = bool(validation.get("gazebo_fallback_used", True))
        self._isaac_available = bool(validation.get("isaac_available", False))
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 3.0))
        self._success_status = str(validation.get("status_success", "batch_dry_run_orchestrator_validated"))
        self._orchestrator_type = str(
            orchestration.get("orchestrator_type", "configuration_only_batch_dry_run_orchestrator")
        )

        self._plan_yaml_path = Path(
            str(source.get("batch_execution_plan_yaml", "diagnostics/proposal_simulation_cell_v1_13/batch_execution_plan.yaml"))
        )
        self._plan_csv_path = Path(
            str(source.get("batch_execution_plan_csv", "diagnostics/proposal_simulation_cell_v1_13/batch_execution_plan.csv"))
        )

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/batch_dry_run_orchestrator_status")),
            10,
        )
        self._schedule_pub = self.create_publisher(
            String,
            str(validation.get("schedule_topic", "/proposal_simulation_cell/batch_dry_run_schedule")),
            10,
        )
        self._report_pub = self.create_publisher(
            String,
            str(validation.get("validation_report_topic", "/proposal_simulation_cell/batch_dry_run_validation_report")),
            10,
        )
        self._blocked_report_pub = self.create_publisher(
            String,
            str(validation.get("blocked_report_topic", "/proposal_simulation_cell/blocked_batch_execution_report")),
            10,
        )

        self._start_time = time.monotonic()
        self._finished = False
        self._plan_yaml = self._load_plan_yaml()
        self._plan_csv_rows = self._load_plan_csv()
        self._execution_plans = self._load_execution_plans()
        self._records = self._generate_records()
        self._status_rows: list[dict[str, str]] = []
        self._validation_rows: list[dict[str, str]] = []
        self._blocked_rows: list[dict[str, str]] = []
        self._last_status: dict[str, Any] = {}
        self._last_validation_report: dict[str, Any] = {}
        self._last_blocked_report: dict[str, Any] = {}

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_14 batch dry-run orchestrator node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.14 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _load_plan_yaml(self) -> dict[str, Any]:
        if not self._plan_yaml_path.is_file():
            return {}
        with self._plan_yaml_path.open("r", encoding="utf-8") as plan_file:
            data = yaml.safe_load(plan_file) or {}
        return data if isinstance(data, dict) else {}

    def _load_plan_csv(self) -> list[dict[str, Any]]:
        if not self._plan_csv_path.is_file():
            return []
        with self._plan_csv_path.open("r", encoding="utf-8", newline="") as csv_file:
            return [self._coerce_csv_plan(dict(row)) for row in csv.DictReader(csv_file)]

    def _load_execution_plans(self) -> list[dict[str, Any]]:
        plans = self._plan_yaml.get("execution_plans", [])
        if isinstance(plans, list) and plans:
            return [plan for plan in plans if isinstance(plan, dict)]
        return self._plan_csv_rows

    def _coerce_csv_plan(self, plan: dict[str, Any]) -> dict[str, Any]:
        bool_fields = {
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
        }
        float_fields = {
            "clearance_mm",
            "x_offset_mm",
            "y_offset_mm",
            "angular_misalignment_deg",
            "insertion_depth_mm",
            "contact_detection_force_threshold_n",
        }
        execution_policy = {}
        result_policy = {}
        coerced: dict[str, Any] = {}
        for key, value in plan.items():
            if key in bool_fields:
                parsed = str(value).lower() == "true"
                if key in {"fake_dataset_created", "fake_plot_created", "experimental_result_created"}:
                    result_policy[key] = parsed
                else:
                    execution_policy[key] = parsed
            elif key in float_fields:
                coerced[key] = float(value)
            elif key == "required_gates":
                coerced[key] = [item for item in str(value).split("|") if item]
            elif key == "planned_validation_outputs":
                coerced[key] = [item for item in str(value).split("|") if item]
            else:
                coerced[key] = value
        coerced["execution_policy"] = execution_policy
        coerced["result_policy"] = result_policy
        return coerced

    def _generate_records(self) -> list[dict[str, Any]]:
        gate_order = list(self._config.get("gate_check_order", []))
        planned_diagnostics = list(self._config.get("planned_diagnostics", []))
        execution_policy = dict(self._config.get("execution_policy", {}))
        result_policy = dict(self._config.get("result_policy", {}))
        records = []
        for index, plan in enumerate(self._execution_plans, start=1):
            records.append(
                {
                    "dry_run_record_id": f"v1_14_dry_run_record_{index:03d}",
                    "scenario_id": plan.get("scenario_id", ""),
                    "clearance_mm": float(plan.get("clearance_mm", 0.0)),
                    "x_offset_mm": float(plan.get("x_offset_mm", 0.0)),
                    "y_offset_mm": float(plan.get("y_offset_mm", 0.0)),
                    "angular_misalignment_deg": float(plan.get("angular_misalignment_deg", 0.0)),
                    "insertion_depth_mm": float(plan.get("insertion_depth_mm", 0.0)),
                    "contact_detection_force_threshold_n": float(
                        plan.get("contact_detection_force_threshold_n", 0.0)
                    ),
                    "orchestration_stages": gate_order,
                    "required_gates": list(plan.get("required_gates", [])),
                    "planned_diagnostic_outputs": planned_diagnostics,
                    "execution_policy": execution_policy,
                    "result_policy": result_policy,
                    "blocked_dry_run_record_generated": True,
                    "scenario_execution_started": False,
                    "scenario_executed": False,
                    "motion_executed": False,
                    "command_sent": False,
                }
            )
        return records

    def _evaluate_and_publish(self) -> None:
        if self._finished:
            return
        validation_report = self._validation_report_payload()
        blocked_report = self._blocked_report_payload()
        status = self._status_payload(
            validation_report,
            blocked_report,
            dry_run_schedule_written=False,
        )
        self._last_validation_report = validation_report
        self._last_blocked_report = blocked_report
        self._last_status = status
        self._publish_json(self._status_pub, status)
        self._publish_json(self._schedule_pub, self._schedule_payload())
        self._publish_json(self._report_pub, validation_report)
        self._publish_json(self._blocked_report_pub, blocked_report)
        self._record_rows(status, validation_report, blocked_report)

    def _validation_report_payload(self) -> dict[str, Any]:
        per_record = []
        for record in self._records:
            checks = self._record_checks(record)
            per_record.append(
                {
                    "dry_run_record_id": record.get("dry_run_record_id", ""),
                    "scenario_id": record.get("scenario_id", ""),
                    "record_validated": all(checks.values()),
                    "checks": checks,
                }
            )
        all_validated = bool(per_record) and all(item["record_validated"] for item in per_record)
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "selected_scenario_count": len(self._records),
            "dry_run_schedule_generated": bool(self._records),
            "all_dry_run_records_validated": all_validated,
            "all_records_configuration_only": all(self._policy_bool(record, "configuration_only") for record in self._records),
            "all_records_dry_run_only": all(self._policy_bool(record, "dry_run_only") for record in self._records),
            "all_records_scenario_execution_disabled": all(
                record.get("execution_policy", {}).get("scenario_execution_enabled") is False
                for record in self._records
            ),
            "all_records_require_rgbd_gate": all(self._requires_gate(record, "rgbd_gate") for record in self._records),
            "all_records_require_contact_gate": all(self._requires_gate(record, "contact_gate") for record in self._records),
            "all_records_require_safety_gate": all(self._requires_gate(record, "safety_gate") for record in self._records),
            "all_records_require_readiness_gate": all(self._requires_gate(record, "readiness_gate") for record in self._records),
            "all_records_require_pre_control_contract_gate": all(
                self._requires_gate(record, "pre_control_contract_gate") for record in self._records
            ),
            "all_records_require_command_blocker_gate": all(
                self._requires_gate(record, "command_blocker_gate") for record in self._records
            ),
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
            "per_record": per_record,
            "status": "batch_dry_run_validation_passed" if all_validated else "batch_dry_run_validation_failed",
        }

    def _record_checks(self, record: dict[str, Any]) -> dict[str, bool]:
        policy = record.get("execution_policy", {})
        result_policy = record.get("result_policy", {})
        stages = record.get("orchestration_stages", [])
        outputs = set(record.get("planned_diagnostic_outputs", []))
        return {
            "scenario_id_exists": bool(record.get("scenario_id")),
            "load_scenario_configuration_stage": "load_scenario_configuration" in stages,
            "verify_rgbd_gate_stage": "verify_rgbd_gate" in stages,
            "verify_contact_gate_stage": "verify_contact_gate" in stages,
            "verify_safety_gate_stage": "verify_safety_gate" in stages,
            "verify_readiness_gate_stage": "verify_readiness_gate" in stages,
            "verify_pre_control_contract_gate_stage": "verify_pre_control_contract_gate" in stages,
            "verify_command_blocker_gate_stage": "verify_command_blocker_gate" in stages,
            "generate_blocked_dry_run_record_stage": "generate_blocked_dry_run_record" in stages,
            "mark_scenario_not_executed_stage": "mark_scenario_not_executed" in stages,
            "configuration_only": policy.get("configuration_only") is True,
            "dry_run_only": policy.get("dry_run_only") is True,
            "scenario_execution_disabled": policy.get("scenario_execution_enabled") is False,
            "command_output_disabled": policy.get("command_output_enabled") is False,
            "motion_execution_disabled": policy.get("motion_execution_enabled") is False,
            "controller_execution_disallowed": policy.get("controller_execution_allowed") is False,
            "trajectory_execution_disallowed": policy.get("trajectory_execution_allowed") is False,
            "follow_joint_trajectory_disallowed": policy.get("follow_joint_trajectory_allowed") is False,
            "real_robot_disallowed": policy.get("real_robot_allowed") is False,
            "moveit_disallowed": policy.get("moveit_allowed") is False,
            "compute_ik_disallowed": policy.get("compute_ik_allowed") is False,
            "rgbd_gate_required": self._requires_gate(record, "rgbd_gate"),
            "contact_gate_required": self._requires_gate(record, "contact_gate"),
            "safety_gate_required": self._requires_gate(record, "safety_gate"),
            "readiness_gate_required": self._requires_gate(record, "readiness_gate"),
            "pre_control_contract_gate_required": self._requires_gate(record, "pre_control_contract_gate"),
            "command_blocker_gate_required": self._requires_gate(record, "command_blocker_gate"),
            "sensor_status_planned": "sensor_status" in outputs,
            "contact_status_planned": "contact_status" in outputs,
            "safety_status_planned": "safety_status" in outputs,
            "readiness_status_planned": "readiness_status" in outputs,
            "no_motion_control_law_status_planned": "no_motion_control_law_status" in outputs,
            "blocked_command_status_planned": "blocked_command_status" in outputs,
            "blocked_batch_execution_report_planned": "blocked_batch_execution_report" in outputs,
            "fake_dataset_not_created": result_policy.get("fake_dataset_created") is False,
            "fake_plot_not_created": result_policy.get("fake_plot_created") is False,
            "experimental_result_not_created": result_policy.get("experimental_result_created") is False,
            "blocked_dry_run_record_generated": record.get("blocked_dry_run_record_generated") is True,
            "scenario_execution_not_started": record.get("scenario_execution_started") is False,
            "scenario_not_executed": record.get("scenario_executed") is False,
            "motion_not_executed": record.get("motion_executed") is False,
            "command_not_sent": record.get("command_sent") is False,
        }

    def _blocked_report_payload(self) -> dict[str, Any]:
        blocked_records = [
            {
                "dry_run_record_id": record.get("dry_run_record_id", ""),
                "scenario_id": record.get("scenario_id", ""),
                "scenario_execution_enabled": False,
                "scenario_executed": False,
                "motion_executed": False,
                "command_sent": False,
                "blocked": True,
            }
            for record in self._records
        ]
        all_blocked = bool(blocked_records) and all(record["blocked"] for record in blocked_records)
        return {
            "stamp_sec": self.get_clock().now().nanoseconds / 1.0e9,
            "blocked_batch_execution_report_available": True,
            "selected_scenario_count": len(blocked_records),
            "all_records_blocked": all_blocked,
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
            "blocked_records": blocked_records,
            "status": "blocked_batch_execution_confirmed" if all_blocked else "blocked_batch_execution_failed",
        }

    def _status_payload(
        self,
        validation_report: dict[str, Any],
        blocked_report: dict[str, Any],
        dry_run_schedule_written: bool,
    ) -> dict[str, Any]:
        validated = bool(validation_report.get("all_dry_run_records_validated", False))
        blocked_available = bool(blocked_report.get("blocked_batch_execution_report_available", False))
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_fallback_used": self._gazebo_fallback_used,
            "isaac_available": self._isaac_available,
            "robot_model": self._robot_model,
            "batch_execution_plan_yaml_found": self._plan_yaml_path.is_file(),
            "batch_execution_plan_csv_found": self._plan_csv_path.is_file(),
            "selected_scenario_count": len(self._records),
            "dry_run_schedule_generated": bool(self._records),
            "dry_run_schedule_written": dry_run_schedule_written,
            "all_dry_run_records_validated": validated,
            "all_records_configuration_only": bool(validation_report.get("all_records_configuration_only", False)),
            "all_records_dry_run_only": bool(validation_report.get("all_records_dry_run_only", False)),
            "all_records_scenario_execution_disabled": bool(
                validation_report.get("all_records_scenario_execution_disabled", False)
            ),
            "all_records_require_rgbd_gate": bool(validation_report.get("all_records_require_rgbd_gate", False)),
            "all_records_require_contact_gate": bool(validation_report.get("all_records_require_contact_gate", False)),
            "all_records_require_safety_gate": bool(validation_report.get("all_records_require_safety_gate", False)),
            "all_records_require_readiness_gate": bool(
                validation_report.get("all_records_require_readiness_gate", False)
            ),
            "all_records_require_pre_control_contract_gate": bool(
                validation_report.get("all_records_require_pre_control_contract_gate", False)
            ),
            "all_records_require_command_blocker_gate": bool(
                validation_report.get("all_records_require_command_blocker_gate", False)
            ),
            "blocked_batch_execution_report_available": blocked_available,
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
            "status": self._success_status
            if validated and blocked_available and dry_run_schedule_written
            else "batch_dry_run_orchestrator_pending",
        }

    def _schedule_payload(self) -> dict[str, Any]:
        return {
            "orchestrator_type": self._orchestrator_type,
            "selected_scenario_count": len(self._records),
            "configuration_only": True,
            "dry_run_only": True,
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
            "dry_run_records": self._records,
        }

    def _requires_gate(self, record: dict[str, Any], gate: str) -> bool:
        return gate in set(record.get("required_gates", []))

    def _policy_bool(self, record: dict[str, Any], key: str) -> bool:
        return record.get("execution_policy", {}).get(key) is True

    def _record_rows(
        self,
        status: dict[str, Any],
        validation_report: dict[str, Any],
        blocked_report: dict[str, Any],
    ) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._status_rows.append(
            {
                "elapsed_sec": elapsed,
                "selected_scenario_count": str(status["selected_scenario_count"]),
                "dry_run_schedule_generated": self._bool(status["dry_run_schedule_generated"]),
                "dry_run_schedule_written": self._bool(status["dry_run_schedule_written"]),
                "all_dry_run_records_validated": self._bool(status["all_dry_run_records_validated"]),
                "all_records_configuration_only": self._bool(status["all_records_configuration_only"]),
                "all_records_dry_run_only": self._bool(status["all_records_dry_run_only"]),
                "all_records_scenario_execution_disabled": self._bool(
                    status["all_records_scenario_execution_disabled"]
                ),
                "blocked_batch_execution_report_available": self._bool(
                    status["blocked_batch_execution_report_available"]
                ),
                "fake_dataset_created": self._bool(status["fake_dataset_created"]),
                "fake_plot_created": self._bool(status["fake_plot_created"]),
                "experimental_result_created": self._bool(status["experimental_result_created"]),
                "status": str(status["status"]),
            }
        )
        self._validation_rows.append(
            {
                "elapsed_sec": elapsed,
                "selected_scenario_count": str(validation_report["selected_scenario_count"]),
                "dry_run_schedule_generated": self._bool(validation_report["dry_run_schedule_generated"]),
                "all_dry_run_records_validated": self._bool(validation_report["all_dry_run_records_validated"]),
                "all_records_require_rgbd_gate": self._bool(validation_report["all_records_require_rgbd_gate"]),
                "all_records_require_contact_gate": self._bool(validation_report["all_records_require_contact_gate"]),
                "all_records_require_safety_gate": self._bool(validation_report["all_records_require_safety_gate"]),
                "all_records_require_readiness_gate": self._bool(validation_report["all_records_require_readiness_gate"]),
                "all_records_require_pre_control_contract_gate": self._bool(
                    validation_report["all_records_require_pre_control_contract_gate"]
                ),
                "all_records_require_command_blocker_gate": self._bool(
                    validation_report["all_records_require_command_blocker_gate"]
                ),
                "fake_dataset_created": self._bool(validation_report["fake_dataset_created"]),
                "fake_plot_created": self._bool(validation_report["fake_plot_created"]),
                "experimental_result_created": self._bool(validation_report["experimental_result_created"]),
                "status": str(validation_report["status"]),
            }
        )
        self._blocked_rows.append(
            {
                "elapsed_sec": elapsed,
                "blocked_batch_execution_report_available": self._bool(
                    blocked_report["blocked_batch_execution_report_available"]
                ),
                "selected_scenario_count": str(blocked_report["selected_scenario_count"]),
                "all_records_blocked": self._bool(blocked_report["all_records_blocked"]),
                "fake_dataset_created": self._bool(blocked_report["fake_dataset_created"]),
                "fake_plot_created": self._bool(blocked_report["fake_plot_created"]),
                "experimental_result_created": self._bool(blocked_report["experimental_result_created"]),
                "status": str(blocked_report["status"]),
            }
        )

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        if not self._last_validation_report:
            self._last_validation_report = self._validation_report_payload()
        if not self._last_blocked_report:
            self._last_blocked_report = self._blocked_report_payload()
        self._write_yaml(self._output_dir / "batch_dry_run_schedule.yaml", self._schedule_payload())
        self._write_json(self._output_dir / "batch_dry_run_schedule.json", self._schedule_payload())
        self._write_schedule_csv(self._output_dir / "batch_dry_run_schedule.csv")
        self._last_status = self._status_payload(
            self._last_validation_report,
            self._last_blocked_report,
            dry_run_schedule_written=True,
        )
        self._record_rows(self._last_status, self._last_validation_report, self._last_blocked_report)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", ["base_link", "hole_center", "peg_tip", "world"])
        self._write_csv(self._output_dir / "batch_dry_run_orchestrator_status_samples.csv", self._status_rows)
        self._write_csv(self._output_dir / "batch_dry_run_validation_report_samples.csv", self._validation_rows)
        self._write_csv(self._output_dir / "blocked_batch_execution_report_samples.csv", self._blocked_rows)
        self._write_json(self._output_dir / "batch_dry_run_orchestrator_status.json", self._last_status)
        self._write_summary(self._last_status)
        self._write_run_log(self._last_status)
        self.get_logger().info("proposal_simulation_cell_v1_14 batch dry-run orchestration diagnostics written")
        rclpy.shutdown()

    def _write_schedule_csv(self, path: Path) -> None:
        fields = [
            "dry_run_record_id",
            "scenario_id",
            "clearance_mm",
            "x_offset_mm",
            "y_offset_mm",
            "angular_misalignment_deg",
            "insertion_depth_mm",
            "contact_detection_force_threshold_n",
            "orchestration_stages",
            "planned_diagnostic_outputs",
            "configuration_only",
            "dry_run_only",
            "scenario_execution_enabled",
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
            "scenario_executed",
            "command_sent",
        ]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            for record in self._records:
                policy = record["execution_policy"]
                result_policy = record["result_policy"]
                writer.writerow(
                    {
                        "dry_run_record_id": record["dry_run_record_id"],
                        "scenario_id": record["scenario_id"],
                        "clearance_mm": record["clearance_mm"],
                        "x_offset_mm": record["x_offset_mm"],
                        "y_offset_mm": record["y_offset_mm"],
                        "angular_misalignment_deg": record["angular_misalignment_deg"],
                        "insertion_depth_mm": record["insertion_depth_mm"],
                        "contact_detection_force_threshold_n": record["contact_detection_force_threshold_n"],
                        "orchestration_stages": "|".join(record["orchestration_stages"]),
                        "planned_diagnostic_outputs": "|".join(record["planned_diagnostic_outputs"]),
                        "configuration_only": policy["configuration_only"],
                        "dry_run_only": policy["dry_run_only"],
                        "scenario_execution_enabled": policy["scenario_execution_enabled"],
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
                        "scenario_executed": record["scenario_executed"],
                        "command_sent": record["command_sent"],
                    }
                )

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_14_batch_dry_run_orchestrator",
            "",
            "Purpose: create and validate blocked dry-run orchestration records for the selected v1.13 batch.",
            "",
            f"Simulation engine: `{status['simulation_engine']}`",
            f"Gazebo fallback used: `{status['gazebo_fallback_used']}`",
            f"Selected scenario count: `{status['selected_scenario_count']}`",
            f"Dry-run schedule generated: `{status['dry_run_schedule_generated']}`",
            f"Dry-run schedule written: `{status['dry_run_schedule_written']}`",
            f"All dry-run records validated: `{status['all_dry_run_records_validated']}`",
            f"All records scenario execution disabled: `{status['all_records_scenario_execution_disabled']}`",
            f"Blocked batch execution report available: `{status['blocked_batch_execution_report_available']}`",
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
            "proposal_simulation_cell_v1_14 batch dry-run orchestration evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"selected_scenario_count={status['selected_scenario_count']}",
            f"dry_run_schedule_generated={str(status['dry_run_schedule_generated']).lower()}",
            f"dry_run_schedule_written={str(status['dry_run_schedule_written']).lower()}",
            f"all_dry_run_records_validated={str(status['all_dry_run_records_validated']).lower()}",
            f"blocked_batch_execution_report_available={str(status['blocked_batch_execution_report_available']).lower()}",
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
    node = ProposalSimulationCellV114BatchDryRunOrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
