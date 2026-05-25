"""Release documentation index verifier for proposal_simulation_cell_v1_17."""

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


class ProposalSimulationCellV117ReleaseIndexNode(Node):
    """Verify release documentation without executing scenarios or motion."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v1_17_release_index_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v1_17")

        self._config = self._load_config()
        validation = self._config.get("validation", {})
        paths = self._config.get("paths", {})
        registry = self._config.get("sprint_registry", {})

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v1_17")
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._main_readme = Path(str(paths.get("main_readme", "../README.md")))
        self._workspace_readme = Path(str(paths.get("workspace_readme", "README.md")))
        self._release_index = Path(str(paths.get("release_index", "../docs/proposal_simulation_cell_release_index.md")))
        self._reviewer_quickstart = Path(
            str(paths.get("reviewer_quickstart", "../docs/proposal_simulation_cell_reviewer_quickstart.md"))
        )
        self._sprint_traceability = Path(
            str(paths.get("sprint_traceability", "../docs/proposal_simulation_cell_sprint_traceability.md"))
        )
        self._no_false_claims_statement = Path(
            str(paths.get("no_false_claims_statement", "../docs/proposal_simulation_cell_no_false_claims_statement.md"))
        )
        self._evidence_package_dir = Path(
            str(paths.get("evidence_package_dir", "diagnostics/proposal_simulation_cell_v1_15"))
        )
        self._reproducibility_checklist_dir = Path(
            str(paths.get("reproducibility_checklist_dir", "diagnostics/proposal_simulation_cell_v1_16"))
        )
        self._completed_sprints = [str(item) for item in registry.get("completed_sprints", [])]
        self._absent_sprints = [str(item) for item in registry.get("absent_or_not_implemented_sprints", ["v1.4"])]
        self._sample_period = float(validation.get("sample_period_sec", 0.2))
        self._validation_duration = float(validation.get("validation_duration_sec", 3.0))
        self._success_status = str(validation.get("status_success", "release_documentation_index_validated"))

        self._release_status_pub = self.create_publisher(
            String,
            str(validation.get("release_index_status_topic", "/proposal_simulation_cell/release_index_status")),
            10,
        )
        self._quickstart_status_pub = self.create_publisher(
            String,
            str(
                validation.get(
                    "reviewer_quickstart_status_topic",
                    "/proposal_simulation_cell/reviewer_quickstart_status",
                )
            ),
            10,
        )

        self._start_time = time.monotonic()
        self._finished = False
        self._release_rows: list[dict[str, str]] = []
        self._quickstart_rows: list[dict[str, str]] = []

        self.create_timer(self._sample_period, self._evaluate_and_publish)
        self.create_timer(self._validation_duration, self._write_outputs_once)
        self.get_logger().info("proposal_simulation_cell_v1_17 release index node started")

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v1.17 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _evaluate_and_publish(self) -> None:
        if self._finished:
            return
        status = self._status_payload(outputs_written=False)
        self._publish_json(self._release_status_pub, status)
        self._publish_json(self._quickstart_status_pub, self._quickstart_payload(status))
        self._record_rows(status)

    def _status_payload(self, outputs_written: bool) -> dict[str, Any]:
        release_index_written = self._release_index.is_file()
        reviewer_quickstart_written = self._reviewer_quickstart.is_file()
        sprint_traceability_written = self._sprint_traceability.is_file()
        no_false_claims_statement_written = self._no_false_claims_statement.is_file()
        evidence_package_found = (
            (self._evidence_package_dir / "proposal_simulation_evidence_package.json").is_file()
            and (self._evidence_package_dir / "proposal_simulation_evidence_package.md").is_file()
            and (self._evidence_package_dir / "proposal_simulation_evidence_package.yaml").is_file()
        )
        reproducibility_checklist_found = (
            (self._reproducibility_checklist_dir / "reproducibility_checklist.json").is_file()
            and (self._reproducibility_checklist_dir / "reproducibility_checklist.md").is_file()
            and (self._reproducibility_checklist_dir / "reproducibility_checklist.yaml").is_file()
        )
        v1_4_marked_absent = self._v1_4_marked_absent()
        status_ok = all(
            [
                outputs_written,
                release_index_written,
                reviewer_quickstart_written,
                sprint_traceability_written,
                no_false_claims_statement_written,
                evidence_package_found,
                reproducibility_checklist_found,
                self._main_readme.is_file(),
                self._workspace_readme.is_file(),
                v1_4_marked_absent,
                self._docs_include_required_sprints(),
                self._docs_have_no_false_claims(),
            ]
        )
        return {
            "release_index_written": release_index_written,
            "reviewer_quickstart_written": reviewer_quickstart_written,
            "sprint_traceability_written": sprint_traceability_written,
            "no_false_claims_statement_written": no_false_claims_statement_written,
            "evidence_package_found": evidence_package_found,
            "reproducibility_checklist_found": reproducibility_checklist_found,
            "main_readme_found": self._main_readme.is_file(),
            "workspace_readme_found": self._workspace_readme.is_file(),
            "v1_4_marked_absent": v1_4_marked_absent,
            **self._result_policy(),
            **self._disabled_policy(),
            "status": self._success_status if status_ok else "release_documentation_index_pending",
        }

    def _quickstart_payload(self, status: dict[str, Any]) -> dict[str, Any]:
        return {
            "reviewer_quickstart_written": status["reviewer_quickstart_written"],
            "release_index_written": status["release_index_written"],
            "evidence_package_found": status["evidence_package_found"],
            "reproducibility_checklist_found": status["reproducibility_checklist_found"],
            "v1_4_marked_absent": status["v1_4_marked_absent"],
            **self._result_policy(),
            **self._disabled_policy(),
            "status": status["status"],
        }

    def _v1_4_marked_absent(self) -> bool:
        if "v1.4" not in self._absent_sprints:
            return False
        docs = [
            self._release_index,
            self._reviewer_quickstart,
            self._sprint_traceability,
            self._no_false_claims_statement,
        ]
        for path in docs:
            if not path.is_file():
                return False
            text = path.read_text(encoding="utf-8").lower()
            if "v1.4" not in text or "absent/not implemented" not in text:
                return False
        return True

    def _docs_include_required_sprints(self) -> bool:
        docs = [self._release_index, self._sprint_traceability]
        for path in docs:
            if not path.is_file():
                return False
            text = path.read_text(encoding="utf-8")
            if any(sprint not in text for sprint in self._completed_sprints):
                return False
        return True

    def _docs_have_no_false_claims(self) -> bool:
        docs = [
            self._release_index,
            self._reviewer_quickstart,
            self._sprint_traceability,
            self._no_false_claims_statement,
            self._main_readme,
            self._workspace_readme,
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
        for path in docs:
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
        status = self._status_payload(outputs_written=True)
        self._record_rows(status)
        topics = sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types())
        services = sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types())
        nodes = sorted(name for name in self.get_node_names() if name)
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)
        self._write_lines(self._output_dir / "tf_frames.txt", ["base_link", "hole_center", "peg_tip", "world"])
        self._write_json(self._output_dir / "release_index_status.json", status)
        self._write_csv(self._output_dir / "release_index_status_samples.csv", self._release_rows)
        self._write_csv(self._output_dir / "reviewer_quickstart_status_samples.csv", self._quickstart_rows)
        self._write_summary(status)
        self._write_run_log(status)
        self.get_logger().info("proposal_simulation_cell_v1_17 release index diagnostics written")
        rclpy.shutdown()

    def _write_summary(self, status: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v1_17_release_documentation_index",
            "",
            f"Status: `{status['status']}`",
            f"Release index written: `{status['release_index_written']}`",
            f"Reviewer quickstart written: `{status['reviewer_quickstart_written']}`",
            f"Sprint traceability written: `{status['sprint_traceability_written']}`",
            f"No-false-claims statement written: `{status['no_false_claims_statement_written']}`",
            f"Evidence package found: `{status['evidence_package_found']}`",
            f"Reproducibility checklist found: `{status['reproducibility_checklist_found']}`",
            f"v1.4 marked absent/not implemented: `{status['v1_4_marked_absent']}`",
            "",
            "Documentation only. No scenario execution, no fake datasets, no fake plots, no experimental results, no MoveIt, no /compute_ik, no controllers, no FollowJointTrajectory, and no real robot execution are claimed.",
            "",
        ]
        (self._output_dir / "summary.md").write_text("\n".join(lines), encoding="utf-8")

    def _write_run_log(self, status: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v1_17 release documentation index evidence",
            f"elapsed_sec={time.monotonic() - self._start_time:.3f}",
            f"status={status['status']}",
            f"release_index_written={self._bool(status['release_index_written'])}",
            f"reviewer_quickstart_written={self._bool(status['reviewer_quickstart_written'])}",
            f"sprint_traceability_written={self._bool(status['sprint_traceability_written'])}",
            f"no_false_claims_statement_written={self._bool(status['no_false_claims_statement_written'])}",
            f"evidence_package_found={self._bool(status['evidence_package_found'])}",
            f"reproducibility_checklist_found={self._bool(status['reproducibility_checklist_found'])}",
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
        self._release_rows.append(
            {
                "elapsed_sec": elapsed,
                "release_index_written": self._bool(status["release_index_written"]),
                "reviewer_quickstart_written": self._bool(status["reviewer_quickstart_written"]),
                "sprint_traceability_written": self._bool(status["sprint_traceability_written"]),
                "no_false_claims_statement_written": self._bool(status["no_false_claims_statement_written"]),
                "evidence_package_found": self._bool(status["evidence_package_found"]),
                "reproducibility_checklist_found": self._bool(status["reproducibility_checklist_found"]),
                "v1_4_marked_absent": self._bool(status["v1_4_marked_absent"]),
                "status": str(status["status"]),
            }
        )
        self._quickstart_rows.append(
            {
                "elapsed_sec": elapsed,
                "reviewer_quickstart_written": self._bool(status["reviewer_quickstart_written"]),
                "release_index_written": self._bool(status["release_index_written"]),
                "evidence_package_found": self._bool(status["evidence_package_found"]),
                "reproducibility_checklist_found": self._bool(status["reproducibility_checklist_found"]),
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
    node = ProposalSimulationCellV117ReleaseIndexNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
