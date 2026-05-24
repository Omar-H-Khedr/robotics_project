"""Dry-run experiment runner for Research Baseline v2.4."""

from __future__ import annotations

import csv
import json
import random
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


PHASES = (
    "initialize",
    "approach",
    "pre_contact_align",
    "contact_search",
    "insertion_attempt",
    "finish",
    "log_results",
)


@dataclass(frozen=True)
class TrialInputs:
    run_id: str
    x_offset_mm: float
    y_offset_mm: float
    angular_error_deg: float
    hole_tolerance_mm: float


class ResearchBaselineV24ExperimentRunner(Node):
    """Generate reproducible dry-run peg-in-hole experiment artifacts."""

    def __init__(self) -> None:
        super().__init__("research_baseline_v2_4_experiment_runner")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/research_baseline_v2_4")
        self.declare_parameter("run_batch", False)
        self.declare_parameter("batch_size", 20)
        self.declare_parameter("seed", 24)

        self._config = self._load_config()
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._run_batch = bool(
            self.get_parameter("run_batch").get_parameter_value().bool_value
        )
        self._batch_size = int(
            self.get_parameter("batch_size").get_parameter_value().integer_value
        )
        self._seed = int(self.get_parameter("seed").get_parameter_value().integer_value)
        self._rng = random.Random(self._seed)

        self._phase_publisher = self.create_publisher(String, "/task_phase", 10)
        self._trial_status_publisher = self.create_publisher(String, "/trial_status", 10)
        self._metrics_publisher = self.create_publisher(
            String,
            "/research_baseline_v2_4/metrics",
            10,
        )
        self._started = False
        self.create_timer(0.1, self._run_once)
        self.get_logger().info(
            "Research Baseline v2.4 experiment runner ready in dry-run mode."
        )

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"v2.4 experiment config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _run_once(self) -> None:
        if self._started:
            return
        self._started = True
        try:
            metrics = self._run_single_experiment()
            if self._run_batch:
                self._run_batch_experiments()
            self._write_graph_snapshots()
            self._write_summary(metrics)
            self.get_logger().info("v2.4 experiment artifacts written.")
        finally:
            rclpy.shutdown()

    def _run_single_experiment(self) -> dict[str, Any]:
        inputs = self._single_trial_inputs()
        phase_rows = self._simulate_phases(inputs)
        metrics = self._metrics_for(inputs, phase_rows)
        self._write_phase_log(phase_rows)
        self._write_json(self._output_dir / "metrics.json", metrics)
        message = String()
        message.data = json.dumps(metrics, sort_keys=True)
        self._metrics_publisher.publish(message)
        self.get_logger().info(message.data)
        return metrics

    def _single_trial_inputs(self) -> TrialInputs:
        single = self._config.get("single_run", {})
        randomization = self._config.get("randomization", {})
        return TrialInputs(
            run_id=str(single.get("run_id", "v2_4_single_001")),
            x_offset_mm=float(single.get("x_offset_mm", 0.0)),
            y_offset_mm=float(single.get("y_offset_mm", 0.0)),
            angular_error_deg=float(single.get("angular_error_deg", 0.0)),
            hole_tolerance_mm=float(single.get("hole_tolerance_mm", randomization.get("hole_tolerance_mm", 2.0))),
        )

    def _simulate_phases(self, inputs: TrialInputs) -> list[dict[str, Any]]:
        rows = []
        start = time.monotonic()
        self._publish_text(self._trial_status_publisher, "dry_run_running")
        for index, phase in enumerate(PHASES, start=1):
            elapsed = time.monotonic() - start
            self._publish_text(self._phase_publisher, phase)
            row = {
                "run_id": inputs.run_id,
                "phase_index": index,
                "phase": phase,
                "elapsed_sec": round(elapsed, 4),
                "x_offset_mm": inputs.x_offset_mm,
                "y_offset_mm": inputs.y_offset_mm,
                "angular_error_deg": inputs.angular_error_deg,
                "status": "dry_run_phase_complete",
            }
            rows.append(row)
            self.get_logger().info(json.dumps(row, sort_keys=True))
            time.sleep(float(self._config.get("phase_sleep_sec", 0.01)))
        self._publish_text(self._trial_status_publisher, "dry_run_completed")
        return rows

    def _metrics_for(
        self,
        inputs: TrialInputs,
        phase_rows: list[dict[str, Any]],
    ) -> dict[str, Any]:
        thresholds = self._config.get("thresholds", {})
        success_xy_mm = float(thresholds.get("success_xy_mm", 2.0))
        success_angle_deg = float(thresholds.get("success_angular_error_deg", 3.0))
        safe_offset_mm = float(thresholds.get("safe_offset_mm", 5.0))
        safe_angle_deg = float(thresholds.get("safe_angular_error_deg", 8.0))

        success = (
            abs(inputs.x_offset_mm) <= success_xy_mm
            and abs(inputs.y_offset_mm) <= success_xy_mm
            and abs(inputs.angular_error_deg) <= success_angle_deg
        )
        safety_violation_count = int(
            abs(inputs.x_offset_mm) > safe_offset_mm
            or abs(inputs.y_offset_mm) > safe_offset_mm
            or abs(inputs.angular_error_deg) > safe_angle_deg
        )
        cycle_time_sec = (
            float(phase_rows[-1]["elapsed_sec"]) if phase_rows else 0.0
        )
        max_force_n = self._synthetic_force(inputs)
        max_torque_nm = round(max_force_n * 0.015, 4)
        safe_success = bool(success and safety_violation_count == 0)
        return {
            "baseline_name": "research_baseline_v2_4_experiment_runner",
            "mode": "diagnostic_dry_run",
            "run_id": inputs.run_id,
            "task_completed": True,
            "success": bool(success),
            "safe_success": safe_success,
            "cycle_time_sec": cycle_time_sec,
            "phase_count": len(phase_rows),
            "safety_violation_count": safety_violation_count,
            "max_force_n": max_force_n,
            "max_torque_nm": max_torque_nm,
            "x_offset_mm": inputs.x_offset_mm,
            "y_offset_mm": inputs.y_offset_mm,
            "angular_error_deg": inputs.angular_error_deg,
            "hole_tolerance_mm": inputs.hole_tolerance_mm,
            "notes": (
                "Dry-run scaffold only: no MoveIt, /compute_ik, Gazebo motion, "
                "controller execution, or FollowJointTrajectory command."
            ),
        }

    def _synthetic_force(self, inputs: TrialInputs) -> float:
        lateral = (inputs.x_offset_mm**2 + inputs.y_offset_mm**2) ** 0.5
        return round(2.0 + 1.1 * lateral + 0.35 * abs(inputs.angular_error_deg), 4)

    def _run_batch_experiments(self) -> None:
        batch_rows = []
        for index in range(1, self._batch_size + 1):
            inputs = self._random_trial_inputs(index)
            phase_rows = [
                {"elapsed_sec": round(0.02 * phase_index, 4)}
                for phase_index, _phase in enumerate(PHASES, start=1)
            ]
            metrics = self._metrics_for(inputs, phase_rows)
            batch_rows.append(metrics)
        self._write_batch_results(batch_rows)
        self._write_batch_summary(batch_rows)

    def _random_trial_inputs(self, index: int) -> TrialInputs:
        randomization = self._config.get("randomization", {})
        x_range = randomization.get("x_offset_mm", [-4.0, 4.0])
        y_range = randomization.get("y_offset_mm", [-4.0, 4.0])
        angle_range = randomization.get("angular_error_deg", [-6.0, 6.0])
        tolerance_range = randomization.get("hole_tolerance_mm", [1.0, 3.0])
        return TrialInputs(
            run_id=f"v2_4_batch_{index:03d}",
            x_offset_mm=round(self._rng.uniform(float(x_range[0]), float(x_range[1])), 4),
            y_offset_mm=round(self._rng.uniform(float(y_range[0]), float(y_range[1])), 4),
            angular_error_deg=round(self._rng.uniform(float(angle_range[0]), float(angle_range[1])), 4),
            hole_tolerance_mm=round(
                self._rng.uniform(float(tolerance_range[0]), float(tolerance_range[1])),
                4,
            ),
        )

    def _write_phase_log(self, rows: list[dict[str, Any]]) -> None:
        path = self._output_dir / "phase_log.csv"
        fieldnames = [
            "run_id",
            "phase_index",
            "phase",
            "elapsed_sec",
            "x_offset_mm",
            "y_offset_mm",
            "angular_error_deg",
            "status",
        ]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    def _write_batch_results(self, rows: list[dict[str, Any]]) -> None:
        path = self._output_dir / "batch_results.csv"
        fieldnames = [
            "run_id",
            "success",
            "safe_success",
            "safety_violation_count",
            "cycle_time_sec",
            "phase_count",
            "max_force_n",
            "max_torque_nm",
            "x_offset_mm",
            "y_offset_mm",
            "angular_error_deg",
            "hole_tolerance_mm",
        ]
        with path.open("w", encoding="utf-8", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow({field: row[field] for field in fieldnames})

    def _write_batch_summary(self, rows: list[dict[str, Any]]) -> None:
        total = len(rows)
        successes = sum(1 for row in rows if row["success"])
        safe_successes = sum(1 for row in rows if row["safe_success"])
        violations = sum(int(row["safety_violation_count"]) for row in rows)
        summary = {
            "baseline_name": "research_baseline_v2_4_experiment_runner",
            "mode": "batch_diagnostic_dry_run",
            "batch_size": total,
            "seed": self._seed,
            "success_count": successes,
            "safe_success_count": safe_successes,
            "success_rate": round(successes / total, 4) if total else 0.0,
            "safe_success_rate": round(safe_successes / total, 4) if total else 0.0,
            "safety_violation_count": violations,
            "status": "complete",
        }
        self._write_json(self._output_dir / "batch_summary.json", summary)

    def _write_graph_snapshots(self) -> None:
        nodes = sorted(name for name in self.get_node_names() if name)
        topics = sorted(name for name, _types in self.get_topic_names_and_types())
        services = sorted(name for name, _types in self.get_service_names_and_types())
        self._write_lines(self._output_dir / "nodes.txt", nodes)
        self._write_lines(self._output_dir / "topics.txt", topics)
        self._write_lines(self._output_dir / "services.txt", services)

    def _write_summary(self, metrics: dict[str, Any]) -> None:
        path = self._output_dir / "summary.md"
        content = "\n".join(
            [
                "# Research Baseline v2.4 Experiment Runner",
                "",
                "This sprint preserves the v2.3 coordinate-based Cartesian insertion diagnostic baseline and adds a dry-run experiment runner.",
                "",
                "Motion status: no real robot, no controller execution, no FollowJointTrajectory, no MoveIt planning, and no /compute_ik call.",
                "",
                f"Single run: `{metrics['run_id']}`",
                f"Success: `{metrics['success']}`",
                f"Safe success: `{metrics['safe_success']}`",
                f"Safety violations: `{metrics['safety_violation_count']}`",
                "",
                "Batch results are written to `batch_results.csv` when batch mode is enabled.",
                "",
            ]
        )
        path.write_text(content, encoding="utf-8")

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def _write_lines(self, path: Path, lines: list[str]) -> None:
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _publish_text(self, publisher, text: str) -> None:
        message = String()
        message.data = text
        publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ResearchBaselineV24ExperimentRunner()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
