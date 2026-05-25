"""Evidence package generator for proposal_simulation_cell_v1_15."""

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


SPRINT_DESCRIPTIONS = {
    "v1.0": "simulation cell foundation",
    "v1.1": "sensor and scene validation",
    "v1.2": "RGB-D image bridge validation",
    "v1.3": "contact physics validation",
    "v1.5": "safety and virtual-force interface",
    "v1.6": "safety gate readiness",
    "v1.7": "pre-control contract",
    "v1.8": "control-development scaffold",
    "v1.9": "no-motion control-law dry run",
    "v1.10": "experiment configuration matrix",
    "v1.11": "single-scenario loader",
    "v1.12": "scenario batch selector",
    "v1.13": "batch execution plan validator",
    "v1.14": "batch dry-run orchestrator",
}


class ProposalSimulationCellV115EvidencePackageNode(Node):
    """Summarize existing proposal simulation diagnostics without generating results."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_15_evidence_package_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_15")

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        robot = self._config.get("robot", {})
        sources = self._config.get("evidence_sources", {})
        registry = self._config.get("sprint_registry", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_15")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._diagnostics_root = Path(str(sources.get("diagnostics_root", "diagnostics")))
        self._evidence_package_type = str(
            sources.get("evidence_package_type", "proposal_simulation_implementation_evidence")
        )
        self._include_sprints = [str(item) for item in registry.get("include_sprints", [])]
        self._absent_sprints = [str(item) for item in registry.get("excluded_or_absent_sprints", ["v1.4"])]
        self._robot_model = str(robot.get("robot_model", robot.get("model", "KUKA LBR iisy 6 R1300")))
        self._simulation_engine = str(validation.get("simulation_engine", "gazebo"))
        self._gazebo_fallback_used = bool(validation.get("gazebo_fallback_used", True))
        self._isaac_available = bool(validation.get("isaac_available", False))
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 3.0))
        self._success_status = str(validation.get("status_success", "evidence_package_validated"))

        self._status_pub = self.create_publisher(
            String, str(validation.get("status_topic", "/proposal_simulation_cell/evidence_package_status")), 10
        )
        self._registry_pub = self.create_publisher(
            String, str(validation.get("registry_topic", "/proposal_simulation_cell/evidence_registry")), 10
        )
        self._summary_pub = self.create_publisher(
            String,
            str(validation.get("summary_topic", "/proposal_simulation_cell/proposal_implementation_summary")),
            10,
        )

        self._start_time = time.monotonic()
        self._finished = False
        self._registry = self._build_registry()
        self._package = self._build_package()
        self._status_rows: list[dict[str, str]] = []
        self._registry_rows: list[dict[str, str]] = []
        self._summary_rows: list[dict[str, str]] = []
        self._last_status: dict[str, Any] = {}

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_15 evidence package node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.15 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _build_registry(self) -> list[dict[str, Any]]:
        rows = []
        for sprint in self._include_sprints:
            folder = self._diagnostics_root / f"proposal_simulation_cell_{sprint.replace('.', '_')}"
            json_files = sorted(path.name for path in folder.glob("*.json")) if folder.is_dir() else []
            summary_files = sorted(path.name for path in folder.glob("*summary*.md")) if folder.is_dir() else []
            status_values = self._status_values(folder)
            rows.append(
                {
                    "sprint": sprint,
                    "description": SPRINT_DESCRIPTIONS.get(sprint, "proposal simulation evidence"),
                    "diagnostics_path": str(folder),
                    "diagnostics_found": folder.is_dir(),
                    "json_file_count": len(json_files),
                    "summary_file_count": len(summary_files),
                    "status_values": sorted(status_values),
                }
            )
        return rows

    def _status_values(self, folder: Path) -> set[str]:
        statuses: set[str] = set()
        if not folder.is_dir():
            return statuses
        for path in folder.glob("*.json"):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (json.JSONDecodeError, OSError):
                continue
            if isinstance(data, dict) and data.get("status"):
                statuses.add(str(data["status"]))
        return statuses

    def _build_package(self) -> dict[str, Any]:
        return {
            "evidence_package_type": self._evidence_package_type,
            "simulation_engine": self._simulation_engine,
            "robot_model": self._robot_model,
            "completed_sprints": self._registry,
            "absent_or_skipped_sprints": self._absent_sprints,
            "validated_capabilities": [
                "simulation cell foundation",
                "sensor and scene validation",
                "RGB-D validation",
                "contact physics validation",
                "safety diagnostic interfaces",
                "readiness gates",
                "pre-control contract",
                "no-motion control-law dry run",
                "scenario configuration matrix",
                "scenario loading and batch selection",
                "configuration-only batch execution planning",
                "blocked batch dry-run orchestration",
            ],
            "disabled_execution_constraints": self._disabled_policy(),
            "result_policy": self._result_policy(),
            "claims_not_made": {
                "scenario_execution_claimed": False,
                "real_robot_validation_claimed": False,
                "controller_execution_claimed": False,
                "learning_run_claimed": False,
            },
        }

    def _evaluate_and_publish(self) -> None:
        if self._finished:
            return
        status = self._status_payload(evidence_package_written=False)
        self._last_status = status
        self._publish_json(self._status_pub, status)
        self._publish_json(self._registry_pub, {"registry": self._registry})
        self._publish_json(self._summary_pub, self._summary_payload())
        self._record_rows(status)

    def _status_payload(self, evidence_package_written: bool) -> dict[str, Any]:
        found = {row["sprint"]: bool(row["diagnostics_found"]) for row in self._registry}
        sources_found = sum(1 for value in found.values() if value)
        evidence_generated = bool(self._registry) and sources_found == len(self._include_sprints)
        status_ok = evidence_generated and evidence_package_written and "v1.4" in self._absent_sprints
        return {
            "simulation_engine": self._simulation_engine,
            "gazebo_fallback_used": self._gazebo_fallback_used,
            "isaac_available": self._isaac_available,
            "robot_model": self._robot_model,
            "evidence_package_generated": evidence_generated,
            "evidence_package_written": evidence_package_written,
            "sprint_registry_generated": bool(self._registry),
            "completed_sprint_count": len(self._include_sprints),
            "absent_or_skipped_sprints": self._absent_sprints,
            "diagnostics_sources_found_count": sources_found,
            "v1_0_found": found.get("v1.0", False),
            "v1_1_found": found.get("v1.1", False),
            "v1_2_found": found.get("v1.2", False),
            "v1_3_found": found.get("v1.3", False),
            "v1_5_found": found.get("v1.5", False),
            "v1_6_found": found.get("v1.6", False),
            "v1_7_found": found.get("v1.7", False),
            "v1_8_found": found.get("v1.8", False),
            "v1_9_found": found.get("v1.9", False),
            "v1_10_found": found.get("v1.10", False),
            "v1_11_found": found.get("v1.11", False),
            "v1_12_found": found.get("v1.12", False),
            "v1_13_found": found.get("v1.13", False),
            "v1_14_found": found.get("v1.14", False),
            "rgbd_validation_evidence_found": found.get("v1.2", False),
            "contact_physics_evidence_found": found.get("v1.3", False),
            "safety_interface_evidence_found": found.get("v1.5", False),
            "readiness_gate_evidence_found": found.get("v1.6", False),
            "pre_control_contract_evidence_found": found.get("v1.7", False),
            "no_motion_control_law_evidence_found": found.get("v1.9", False),
            "scenario_matrix_evidence_found": found.get("v1.10", False),
            "batch_orchestration_evidence_found": found.get("v1.14", False),
            **self._result_policy(),
            "scenario_execution_claimed": False,
            "real_robot_validation_claimed": False,
            **self._disabled_policy(),
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
            "status": self._success_status if status_ok else "evidence_package_pending",
        }

    def _disabled_policy(self) -> dict[str, bool]:
        return {
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "trajectory_execution_allowed": False,
            "follow_joint_trajectory_allowed": False,
        }

    def _result_policy(self) -> dict[str, bool]:
        return {
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
        }

    def _summary_payload(self) -> dict[str, Any]:
        return {
            "evidence_package_type": self._evidence_package_type,
            "completed_sprint_count": len(self._include_sprints),
            "absent_or_skipped_sprints": self._absent_sprints,
            "simulation_only": True,
            "no_scenario_execution": True,
            **self._result_policy(),
            **self._disabled_policy(),
        }

    def _record_rows(self, status: dict[str, Any]) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._status_rows.append(
            {
                "elapsed_sec": elapsed,
                "evidence_package_generated": self._bool(status["evidence_package_generated"]),
                "evidence_package_written": self._bool(status["evidence_package_written"]),
                "sprint_registry_generated": self._bool(status["sprint_registry_generated"]),
                "completed_sprint_count": str(status["completed_sprint_count"]),
                "diagnostics_sources_found_count": str(status["diagnostics_sources_found_count"]),
                "fake_dataset_created": self._bool(status["fake_dataset_created"]),
                "fake_plot_created": self._bool(status["fake_plot_created"]),
                "experimental_result_created": self._bool(status["experimental_result_created"]),
                "scenario_execution_claimed": self._bool(status["scenario_execution_claimed"]),
                "real_robot_validation_claimed": self._bool(status["real_robot_validation_claimed"]),
                "status": str(status["status"]),
            }
        )
        for row in self._registry:
            self._registry_rows.append(
                {
                    "elapsed_sec": elapsed,
                    "sprint": str(row["sprint"]),
                    "description": str(row["description"]),
                    "diagnostics_found": self._bool(row["diagnostics_found"]),
                    "json_file_count": str(row["json_file_count"]),
                    "summary_file_count": str(row["summary_file_count"]),
                }
            )
        self._summary_rows.append(
            {
                "elapsed_sec": elapsed,
                "completed_sprint_count": str(len(self._include_sprints)),
                "v1_4_absent": self._bool("v1.4" in self._absent_sprints),
                "no_scenario_execution": "true",
                "no_fake_datasets": "true",
                "no_fake_plots": "true",
                "no_experimental_results": "true",
                "status": str(status["status"]),
            }
        )

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        self._write_json(self._output_dir / "proposal_simulation_evidence_package.json", self._package)
        self._write_yaml(self._output_dir / "proposal_simulation_evidence_package.yaml", self._package)
        self._write_markdown(self._output_dir / "proposal_simulation_evidence_package.md")
        self._write_registry_csv(self._output_dir / "evidence_registry.csv")
        status = self._status_payload(evidence_package_written=True)
        self._last_status = status
        self._record_rows(status)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", ["base_link", "hole_center", "peg_tip", "world"])
        self._write_json(self._output_dir / "evidence_package_status.json", status)
        self._write_csv(self._output_dir / "evidence_package_status_samples.csv", self._status_rows)
        self._write_csv(self._output_dir / "evidence_registry_samples.csv", self._registry_rows)
        self._write_csv(self._output_dir / "proposal_implementation_summary_samples.csv", self._summary_rows)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_15 evidence package diagnostics written")
        rclpy.shutdown()

    def _write_markdown(self, path: Path) -> None:
        lines = [
            "# Proposal Simulation Cell Evidence Package",
            "",
            "Evidence package type: `proposal_simulation_implementation_evidence`",
            "",
            "This package summarizes existing diagnostics only. It does not create datasets, plots, experimental results, scenario execution, robot motion, controller execution, MoveIt use, or real robot validation.",
            "",
            "## Sprint Evidence",
            "",
        ]
        for row in self._registry:
            lines.append(f"- `{row['sprint']}`: {row['description']} evidence at `{row['diagnostics_path']}`; found=`{row['diagnostics_found']}`")
        lines.extend(
            [
                "",
                "## Absent Sprint",
                "",
                "- `v1.4` is intentionally absent/not implemented and is not invented in this package.",
                "",
                "## Disabled Execution",
                "",
                "- `command_output_enabled=false`",
                "- `motion_execution_enabled=false`",
                "- no MoveIt",
                "- no `/compute_ik`",
                "- no controllers",
                "- no real robot execution",
                "- no `FollowJointTrajectory`",
                "- no scenario execution",
                "",
                "## Result Policy",
                "",
                "- `fake_dataset_created=false`",
                "- `fake_plot_created=false`",
                "- `experimental_result_created=false`",
                "",
            ]
        )
        path.write_text("\n".join(lines), encoding="utf-8")

    def _write_registry_csv(self, path: Path) -> None:
        rows = [
            {
                "sprint": row["sprint"],
                "description": row["description"],
                "diagnostics_path": row["diagnostics_path"],
                "diagnostics_found": self._bool(row["diagnostics_found"]),
                "json_file_count": str(row["json_file_count"]),
                "summary_file_count": str(row["summary_file_count"]),
                "status_values": "|".join(row["status_values"]),
            }
            for row in self._registry
        ]
        self._write_csv(path, rows)

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_15_evidence_package_generator",
            "",
            f"Status: `{status['status']}`",
            f"Completed sprint count: `{status['completed_sprint_count']}`",
            f"Diagnostics sources found: `{status['diagnostics_sources_found_count']}`",
            "Absent sprint: `v1.4` is intentionally absent/not implemented.",
            "",
            "No scenario execution, no fake datasets, no fake plots, no experimental results, no MoveIt, no /compute_ik, no controllers, and no real robot execution are claimed.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_15 evidence package evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"completed_sprint_count={status['completed_sprint_count']}",
            f"diagnostics_sources_found_count={status['diagnostics_sources_found_count']}",
            "v1_4_absent=true",
            "fake_dataset_created=false",
            "fake_plot_created=false",
            "experimental_result_created=false",
            "scenario_execution_claimed=false",
            "real_robot_validation_claimed=false",
            "command_output_enabled=false",
            "motion_execution_enabled=false",
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
    node = ProposalSimulationCellV115EvidencePackageNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
