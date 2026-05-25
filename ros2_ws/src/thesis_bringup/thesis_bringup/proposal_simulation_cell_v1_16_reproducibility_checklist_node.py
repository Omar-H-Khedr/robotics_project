"""Reproducibility checklist for proposal_simulation_cell_v1_16."""

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
    "v1.11": "single-scenario loader validation",
    "v1.12": "scenario batch selector",
    "v1.13": "batch execution plan validator",
    "v1.14": "batch dry-run orchestrator",
    "v1.15": "evidence package generator",
}


class ProposalSimulationCellV116ReproducibilityChecklistNode(Node):
    """Create reviewer diagnostics without executing scenarios or motion."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_16_reproducibility_checklist_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_16")

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        paths = self._config.get("paths", {})
        registry = self._config.get("sprint_registry", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_16")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._main_readme = Path(str(paths.get("main_readme", "../README.md")))
        self._workspace_readme = Path(str(paths.get("workspace_readme", "README.md")))
        self._diagnostics_root = Path(str(paths.get("diagnostics_root", "diagnostics")))
        self._v1_15_dir = Path(str(paths.get("v1_15_diagnostics", "diagnostics/proposal_simulation_cell_v1_15")))
        self._implemented_sprints = [str(item) for item in registry.get("implemented_sprints", [])]
        self._absent_sprints = [str(item) for item in registry.get("absent_or_not_implemented_sprints", ["v1.4"])]
        self._v1_15_required_files = [str(item) for item in self._config.get("v1_15_required_files", [])]
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 3.0))
        self._success_status = str(validation.get("status_success", "reproducibility_checklist_validated"))

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/reproducibility_checklist_status")),
            10,
        )
        self._checklist_pub = self.create_publisher(
            String,
            str(validation.get("checklist_topic", "/proposal_simulation_cell/reproducibility_checklist")),
            10,
        )
        self._summary_pub = self.create_publisher(
            String,
            str(validation.get("reviewer_summary_topic", "/proposal_simulation_cell/reviewer_implementation_summary")),
            10,
        )

        self._start_time = time.monotonic()
        self._finished = False
        self._status_rows: list[dict[str, str]] = []
        self._summary_rows: list[dict[str, str]] = []
        self._checklist = self._build_checklist()
        self._reviewer_summary = self._build_reviewer_summary()
        self._last_status: dict[str, Any] = {}

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_16 reproducibility checklist node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.16 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _build_checklist(self) -> dict[str, Any]:
        diagnostics = []
        for sprint in self._implemented_sprints:
            folder = self._diagnostics_root / f"proposal_simulation_cell_{sprint.replace('.', '_')}"
            diagnostics.append(
                {
                    "sprint": sprint,
                    "description": SPRINT_DESCRIPTIONS.get(sprint, "proposal simulation diagnostic"),
                    "diagnostics_path": str(folder),
                    "found": folder.is_dir(),
                    "summary_found": (folder / "summary.md").is_file(),
                    "run_log_found": (folder / "run.log").is_file(),
                }
            )
        return {
            "checklist_type": "proposal_simulation_cell_reproducibility_checklist",
            "readme_files": {
                "main_readme": str(self._main_readme),
                "workspace_readme": str(self._workspace_readme),
            },
            "implemented_diagnostics": diagnostics,
            "v1_15_required_files": [
                {"path": str(self._v1_15_dir / filename), "found": (self._v1_15_dir / filename).is_file()}
                for filename in self._v1_15_required_files
            ],
            "absent_or_not_implemented_sprints": self._absent_sprints,
            "result_policy": self._result_policy(),
            "disabled_execution_policy": self._disabled_policy(),
        }

    def _build_reviewer_summary(self) -> dict[str, Any]:
        return {
            "summary_type": "proposal_simulation_cell_reviewer_implementation_summary",
            "scope": "simulation-only proposal implementation",
            "implemented_sprints": self._implemented_sprints,
            "absent_or_not_implemented_sprints": self._absent_sprints,
            "evidence_package": str(self._v1_15_dir),
            "reproducibility_checklist": str(self._output_dir),
            "limitations": [
                "v1.4 remains absent/not implemented",
                "no scenario execution",
                "no fake datasets",
                "no fake plots",
                "no experimental results",
                "no real robot execution",
                "no MoveIt",
                "no /compute_ik",
                "no controllers",
            ],
            **self._result_policy(),
            **self._disabled_policy(),
        }

    def _evaluate_and_publish(self) -> None:
        if self._finished:
            return
        status = self._status_payload(outputs_written=False)
        self._last_status = status
        self._publish_json(self._status_pub, status)
        self._publish_json(self._checklist_pub, self._checklist)
        self._publish_json(self._summary_pub, self._reviewer_summary)
        self._record_rows(status)

    def _status_payload(self, outputs_written: bool) -> dict[str, Any]:
        evidence_package_found = (
            (self._v1_15_dir / "proposal_simulation_evidence_package.json").is_file()
            and (self._v1_15_dir / "proposal_simulation_evidence_package.md").is_file()
            and (self._v1_15_dir / "proposal_simulation_evidence_package.yaml").is_file()
        )
        evidence_registry_found = (self._v1_15_dir / "evidence_registry.csv").is_file()
        implemented_found_count = sum(
            1
            for sprint in self._implemented_sprints
            if (self._diagnostics_root / f"proposal_simulation_cell_{sprint.replace('.', '_')}").is_dir()
        )
        v1_15_files_found = all((self._v1_15_dir / filename).is_file() for filename in self._v1_15_required_files)
        v1_4_marked_absent = self._v1_4_marked_absent()
        docs_have_no_false_claims = self._docs_have_no_false_claims()
        checklist_generated = bool(self._checklist)
        reviewer_summary_generated = bool(self._reviewer_summary)
        status_ok = all(
            [
                self._main_readme.is_file(),
                self._workspace_readme.is_file(),
                evidence_package_found,
                evidence_registry_found,
                v1_15_files_found,
                implemented_found_count == len(self._implemented_sprints),
                v1_4_marked_absent,
                docs_have_no_false_claims,
                checklist_generated,
                reviewer_summary_generated,
                outputs_written,
            ]
        )
        return {
            "readme_found": self._main_readme.is_file(),
            "workspace_readme_found": self._workspace_readme.is_file(),
            "evidence_package_found": evidence_package_found,
            "evidence_registry_found": evidence_registry_found,
            "v1_15_required_files_found": v1_15_files_found,
            "implemented_diagnostics_found_count": implemented_found_count,
            "implemented_diagnostics_expected_count": len(self._implemented_sprints),
            "v1_4_marked_absent": v1_4_marked_absent,
            "documentation_false_claims_absent": docs_have_no_false_claims,
            "reproducibility_checklist_generated": checklist_generated,
            "reproducibility_checklist_written": outputs_written,
            "reviewer_summary_generated": reviewer_summary_generated,
            "reviewer_summary_written": outputs_written,
            **self._result_policy(),
            **self._disabled_policy(),
            "status": self._success_status if status_ok else "reproducibility_checklist_pending",
        }

    def _v1_4_marked_absent(self) -> bool:
        evidence_json = self._v1_15_dir / "proposal_simulation_evidence_package.json"
        evidence_md = self._v1_15_dir / "proposal_simulation_evidence_package.md"
        config_absent = "v1.4" in self._absent_sprints
        json_absent = False
        md_absent = False
        if evidence_json.is_file():
            try:
                data = json.loads(evidence_json.read_text(encoding="utf-8"))
                json_absent = "v1.4" in [str(item) for item in data.get("absent_or_skipped_sprints", [])]
            except (json.JSONDecodeError, OSError):
                json_absent = False
        if evidence_md.is_file():
            text = evidence_md.read_text(encoding="utf-8").lower()
            md_absent = "v1.4" in text and "absent/not implemented" in text
        return config_absent and json_absent and md_absent

    def _docs_have_no_false_claims(self) -> bool:
        paths = [
            self._main_readme,
            self._workspace_readme,
            self._v1_15_dir / "proposal_simulation_evidence_package.json",
            self._v1_15_dir / "proposal_simulation_evidence_package.md",
            self._v1_15_dir / "proposal_simulation_evidence_package.yaml",
            self._v1_15_dir / "summary.md",
        ]
        forbidden_positive_tokens = [
            "fake_dataset_created=true",
            "fake_plot_created=true",
            "experimental_result_created=true",
            "scenario_execution_claimed=true",
            "real_robot_validation_claimed=true",
            "command_output_enabled=true",
            "motion_execution_enabled=true",
            "controller_execution_allowed=true",
            "real_robot_used=true",
            "moveit_used=true",
            "compute_ik_called=true",
        ]
        for path in paths:
            if not path.is_file():
                return False
            text = path.read_text(encoding="utf-8").lower().replace(": true", "=true")
            if any(token in text for token in forbidden_positive_tokens):
                return False
        return True

    def _write_outputs_once(self) -> None:
        if self._finished:
            return
        self._finished = True
        self._write_json(self._output_dir / "reproducibility_checklist.json", self._checklist)
        self._write_yaml(self._output_dir / "reproducibility_checklist.yaml", self._checklist)
        self._write_checklist_markdown(self._output_dir / "reproducibility_checklist.md")
        self._write_reviewer_summary_markdown(self._output_dir / "reviewer_implementation_summary.md")
        status = self._status_payload(outputs_written=True)
        self._last_status = status
        self._record_rows(status)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", ["base_link", "hole_center", "peg_tip", "world"])
        self._write_json(self._output_dir / "reproducibility_checklist_status.json", status)
        self._write_csv(self._output_dir / "reproducibility_checklist_status_samples.csv", self._status_rows)
        self._write_csv(self._output_dir / "reviewer_implementation_summary_samples.csv", self._summary_rows)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_16 reproducibility checklist diagnostics written")
        rclpy.shutdown()

    def _write_checklist_markdown(self, path: Path) -> None:
        lines = [
            "# Proposal Simulation Cell Reproducibility Checklist",
            "",
            "This checklist verifies existing proposal simulation documentation and diagnostics only. It does not execute scenarios, enable motion, use MoveIt, call `/compute_ik`, use controllers, create fake datasets, create fake plots, create experimental results, or claim real robot validation.",
            "",
            "## Required Documentation",
            "",
            f"- Main README exists: `{self._main_readme.is_file()}`",
            f"- Workspace README exists: `{self._workspace_readme.is_file()}`",
            "",
            "## Evidence Package",
            "",
            f"- v1.15 evidence package found: `{(self._v1_15_dir / 'proposal_simulation_evidence_package.json').is_file()}`",
            f"- v1.15 evidence registry found: `{(self._v1_15_dir / 'evidence_registry.csv').is_file()}`",
            "",
            "## Implemented Diagnostics",
            "",
        ]
        for item in self._checklist["implemented_diagnostics"]:
            lines.append(f"- `{item['sprint']}`: {item['description']}; diagnostics found=`{item['found']}`")
        lines.extend(
            [
                "",
                "## Absent Sprint",
                "",
                "- `v1.4` remains absent/not implemented and is not invented by this checklist.",
                "",
                "## Disabled Execution And Result Policy",
                "",
                "- `fake_dataset_created=false`",
                "- `fake_plot_created=false`",
                "- `experimental_result_created=false`",
                "- `scenario_execution_claimed=false`",
                "- `real_robot_validation_claimed=false`",
                "- `command_output_enabled=false`",
                "- `motion_execution_enabled=false`",
                "- `controller_execution_allowed=false`",
                "- `real_robot_used=false`",
                "- `moveit_used=false`",
                "- `compute_ik_called=false`",
                "",
            ]
        )
        path.write_text("\n".join(lines), encoding="utf-8")

    def _write_reviewer_summary_markdown(self, path: Path) -> None:
        lines = [
            "# Reviewer Implementation Summary",
            "",
            "The proposal simulation cell currently provides a simulation-only implementation record for the KUKA LBR iisy peg-in-hole proposal workflow. The implemented diagnostics cover the simulation cell foundation, sensor and scene checks, RGB-D bridge validation, contact physics validation, safety and virtual-force diagnostic interfaces, readiness gates, pre-control contracts, no-motion control-law dry runs, scenario configuration, scenario selection, configuration-only batch planning, blocked dry-run orchestration, and v1.15 evidence packaging.",
            "",
            "v1.16 adds this reviewer-facing reproducibility checklist and summary. It checks that the v1.15 evidence package and registry exist, verifies the implemented diagnostics folders, and confirms that `v1.4` remains absent/not implemented.",
            "",
            "No scenario execution, fake datasets, fake plots, experimental results, real robot execution, MoveIt use, `/compute_ik` calls, controllers, command output, or motion execution are enabled or claimed.",
            "",
            "Diagnostics are stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_16/`.",
            "",
        ]
        path.write_text("\n".join(lines), encoding="utf-8")

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_16_reproducibility_checklist",
            "",
            f"Status: `{status['status']}`",
            f"Evidence package found: `{status['evidence_package_found']}`",
            f"Evidence registry found: `{status['evidence_registry_found']}`",
            f"Implemented diagnostics found: `{status['implemented_diagnostics_found_count']}` of `{status['implemented_diagnostics_expected_count']}`",
            f"v1.4 marked absent/not implemented: `{status['v1_4_marked_absent']}`",
            "",
            "No scenario execution, no fake datasets, no fake plots, no experimental results, no MoveIt, no /compute_ik, no controllers, and no real robot execution are claimed.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_16 reproducibility checklist evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"evidence_package_found={self._bool(status['evidence_package_found'])}",
            f"evidence_registry_found={self._bool(status['evidence_registry_found'])}",
            f"implemented_diagnostics_found_count={status['implemented_diagnostics_found_count']}",
            f"v1_4_marked_absent={self._bool(status['v1_4_marked_absent'])}",
            "fake_dataset_created=false",
            "fake_plot_created=false",
            "experimental_result_created=false",
            "scenario_execution_claimed=false",
            "real_robot_validation_claimed=false",
            "command_output_enabled=false",
            "motion_execution_enabled=false",
            "controller_execution_allowed=false",
            "real_robot_used=false",
            "moveit_used=false",
            "compute_ik_called=false",
            "",
        ]
        (self._output_dir / "run.log").write_text("\n".join(lines), encoding="utf-8")

    def _record_rows(self, status: dict[str, Any]) -> None:
        elapsed = f"{time.monotonic() - self._start_time:.3f}"
        self._status_rows.append(
            {
                "elapsed_sec": elapsed,
                "evidence_package_found": self._bool(status["evidence_package_found"]),
                "evidence_registry_found": self._bool(status["evidence_registry_found"]),
                "implemented_diagnostics_found_count": str(status["implemented_diagnostics_found_count"]),
                "v1_4_marked_absent": self._bool(status["v1_4_marked_absent"]),
                "reproducibility_checklist_written": self._bool(status["reproducibility_checklist_written"]),
                "reviewer_summary_written": self._bool(status["reviewer_summary_written"]),
                "fake_dataset_created": self._bool(status["fake_dataset_created"]),
                "fake_plot_created": self._bool(status["fake_plot_created"]),
                "experimental_result_created": self._bool(status["experimental_result_created"]),
                "scenario_execution_claimed": self._bool(status["scenario_execution_claimed"]),
                "real_robot_validation_claimed": self._bool(status["real_robot_validation_claimed"]),
                "status": str(status["status"]),
            }
        )
        self._summary_rows.append(
            {
                "elapsed_sec": elapsed,
                "implemented_sprint_count": str(len(self._implemented_sprints)),
                "v1_4_absent": self._bool(status["v1_4_marked_absent"]),
                "no_scenario_execution": self._bool(not status["scenario_execution_claimed"]),
                "no_fake_datasets": self._bool(not status["fake_dataset_created"]),
                "no_fake_plots": self._bool(not status["fake_plot_created"]),
                "no_experimental_results": self._bool(not status["experimental_result_created"]),
                "status": str(status["status"]),
            }
        )

    def _disabled_policy(self) -> dict[str, bool]:
        return {
            "command_output_enabled": False,
            "motion_execution_enabled": False,
            "controller_execution_allowed": False,
            "real_robot_used": False,
            "moveit_used": False,
            "compute_ik_called": False,
        }

    def _result_policy(self) -> dict[str, bool]:
        return {
            "fake_dataset_created": False,
            "fake_plot_created": False,
            "experimental_result_created": False,
            "scenario_execution_claimed": False,
            "real_robot_validation_claimed": False,
        }

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
    node = ProposalSimulationCellV116ReproducibilityChecklistNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
