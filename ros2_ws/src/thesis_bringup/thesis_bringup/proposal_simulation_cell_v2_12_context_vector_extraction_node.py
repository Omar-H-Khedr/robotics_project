"""Context vector extraction from proposal simulation cell v2.11 logs."""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


class ProposalSimulationCellV212ContextVectorExtractionNode(Node):
    """Extract diagnostic-only context vectors from real v2.11 simulation logs."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_12_context_vector_extraction_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_12")

        self._config = self._load_config()
        extraction = self._config.get("context_vector_extraction", {})
        inputs = self._config.get("input_files", {})
        feature = self._config.get("feature_extraction", {})
        validation = self._config.get("validation", {})

        self._source_sprint = str(extraction.get("source_sprint", "v2.11"))
        self._task_type = str(extraction.get("task_type", "context_vector_extraction"))
        self._source_dir = Path(str(extraction.get("source_diagnostics_dir", "diagnostics/proposal_simulation_cell_v2_11")))
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or extraction.get("output_dir", "diagnostics/proposal_simulation_cell_v2_12")
        )
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._fake_dataset_created = bool(extraction.get("fake_dataset_created", False))
        self._fake_result_created = bool(extraction.get("fake_result_created", False))
        self._learning_executed = bool(extraction.get("learning_executed", False))
        self._policy_training_executed = bool(extraction.get("policy_training_executed", False))
        self._real_robot_used = bool(extraction.get("real_robot_used", False))
        self._peg_insertion_executed = bool(extraction.get("peg_insertion_executed", False))
        self._forceful_contact_executed = bool(extraction.get("forceful_contact_executed", False))

        self._input_names = {
            "observation": str(inputs.get("observation_log", "multimodal_observation_log.csv")),
            "contact_transition": str(inputs.get("contact_transition_log", "contact_transition_observation_log.csv")),
            "scenario_summary": str(inputs.get("scenario_summary", "scenario_execution_summary.csv")),
            "rgbd": str(inputs.get("rgbd_frame_count_report", "rgbd_frame_count_report.csv")),
            "channel": str(inputs.get("observation_channel_completeness_report", "observation_channel_completeness_report.csv")),
            "safety": str(inputs.get("scenario_safety_report", "scenario_safety_report.csv")),
        }
        self._max_offset = float(feature.get("max_expected_offset_m", 0.003))
        self._max_yaw = float(feature.get("max_expected_yaw_deg", 2.0))
        self._max_trigger_step = float(feature.get("max_expected_trigger_step", 12.0))
        self._force_norm = float(feature.get("force_normalization_n", 8.0))
        self._success_status = str(validation.get("status_success", "context_vector_extraction_validated"))
        self._missing_status = str(validation.get("status_missing_inputs", "context_vector_source_inputs_missing"))
        self._empty_status = str(validation.get("status_empty_inputs", "context_vector_source_inputs_empty"))

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/context_vector_extraction_status")),
            10,
        )
        self._context_pub = self.create_publisher(
            String,
            str(validation.get("context_vector_report_topic", "/proposal_simulation_cell/context_vector_report")),
            10,
        )
        self._transition_pub = self.create_publisher(
            String,
            str(
                validation.get(
                    "contact_transition_feature_report_topic",
                    "/proposal_simulation_cell/contact_transition_feature_report",
                )
            ),
            10,
        )

        self._started = False
        self._finished = False
        self._start_time = time.monotonic()
        self.create_timer(0.2, self._tick)

    def _load_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            return {}
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"proposal v2.12 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _tick(self) -> None:
        if self._finished or self._started:
            return
        self._started = True
        self._run()

    def _run(self) -> None:
        paths = {key: self._source_dir / name for key, name in self._input_names.items()}
        found = {key: path.is_file() for key, path in paths.items()}
        if not all(found.values()):
            self._write_outputs(paths, found, [], [], [], [], [], self._missing_status)
            return

        observations = self._read_csv(paths["observation"])
        transitions = self._read_csv(paths["contact_transition"])
        scenario_summary = self._read_csv(paths["scenario_summary"])
        rgbd_rows = self._read_csv(paths["rgbd"])
        channel_rows = self._read_csv(paths["channel"])
        safety_rows = self._read_csv(paths["safety"])
        if not observations or not transitions:
            self._write_outputs(paths, found, observations, transitions, [], [], [], self._empty_status)
            return

        vectors = self._scenario_context_vectors(observations, transitions, scenario_summary, rgbd_rows, safety_rows)
        transition_vectors = self._contact_transition_vectors(transitions, vectors)
        episodes = self._episode_summaries(vectors)
        channel_summary = self._channel_summary(channel_rows)
        safety_summary = self._safety_summary(vectors, safety_rows)
        status = self._success_status if self._valid_success(found, observations, transitions, vectors) else self._empty_status
        self._write_outputs(paths, found, observations, transitions, vectors, transition_vectors, episodes, status, channel_summary, safety_summary)

    def _scenario_context_vectors(
        self,
        observations: list[dict[str, str]],
        transitions: list[dict[str, str]],
        scenario_summary: list[dict[str, str]],
        rgbd_rows: list[dict[str, str]],
        safety_rows: list[dict[str, str]],
    ) -> list[dict[str, str]]:
        by_scenario: dict[str, list[dict[str, str]]] = {}
        for row in observations:
            by_scenario.setdefault(row.get("scenario_id", ""), []).append(row)
        transitions_by = {row.get("scenario", ""): row for row in transitions}
        summary_by = {row.get("scenario", ""): row for row in scenario_summary}
        rgbd_by = {row.get("scenario", ""): row for row in rgbd_rows}
        safety_by = {row.get("scenario", ""): row for row in safety_rows}
        vectors = []
        for scenario_id, rows in sorted(by_scenario.items()):
            if not scenario_id:
                continue
            first = rows[0]
            transition = transitions_by.get(scenario_id, {})
            summary = summary_by.get(scenario_id, {})
            rgbd = rgbd_by.get(scenario_id, {})
            safety = safety_by.get(scenario_id, {})
            forces = [self._float(row.get("contact_force_n")) for row in rows]
            torques = [self._float(row.get("contact_torque_nm")) for row in rows]
            trigger_force = self._float(transition.get("force_n"))
            initial_force = next((self._float(row.get("contact_force_n")) for row in rows if "verify_initial_no_contact" in row.get("phase_name", "")), 0.0)
            post_force = next((self._float(row.get("contact_force_n")) for row in rows if "verify_post_retreat_no_contact" in row.get("phase_name", "")), 0.0)
            pre_contact = [self._float(row.get("contact_force_n")) for row in rows if self._bool_text(row.get("contact_gate_triggered")) is False]
            contact_trigger_step = int(self._float(summary.get("contact_trigger_step_index", transition.get("step_index", "0"))))
            max_joint_delta = self._max_joint_delta(rows)
            safety_clear = int(self._int(safety.get("safety_violation_count", summary.get("safety_violation_count", "0"))) == 0)
            rgb_available = self._bool_text(rgbd.get("rgb_topic_available", first.get("rgb_topic_available", "false")))
            depth_available = self._bool_text(rgbd.get("depth_topic_available", first.get("depth_topic_available", "false")))
            stop_on_contact = self._bool_text(summary.get("stop_on_contact_executed", transition.get("stop_on_contact_executed", "false")))
            retreat_completed = self._bool_text(summary.get("retreat_completed", "false"))
            post_retreat_ok = self._bool_text(summary.get("post_retreat_no_contact_verified", "false"))
            contact_after_motion = self._bool_text(summary.get("contact_trigger_after_motion", "false"))
            max_force = max(forces + [self._float(summary.get("max_force_n")), self._float(safety.get("max_force_n"))])
            vectors.append(
                {
                    "scenario_id": scenario_id,
                    "x_offset_m": self._fmt(first.get("x_offset_m")),
                    "y_offset_m": self._fmt(first.get("y_offset_m")),
                    "yaw_offset_deg": self._fmt(first.get("yaw_offset_deg")),
                    "observation_count": str(len(rows)),
                    "rgb_available": self._bool(rgb_available),
                    "depth_available": self._bool(depth_available),
                    "rgb_frame_count_delta": str(self._int(rgbd.get("rgb_frame_count_delta", "0"))),
                    "depth_frame_count_delta": str(self._int(rgbd.get("depth_frame_count_delta", "0"))),
                    "initial_force_n": f"{initial_force:.9f}",
                    "max_force_n": f"{max_force:.9f}",
                    "max_torque_nm": f"{max(torques + [self._float(summary.get('max_torque_nm')), self._float(safety.get('max_torque_nm'))]):.9f}",
                    "contact_trigger_step_index": str(contact_trigger_step),
                    "contact_trigger_after_motion": self._bool(contact_after_motion),
                    "stop_on_contact_executed": self._bool(stop_on_contact),
                    "retreat_completed": self._bool(retreat_completed),
                    "post_retreat_no_contact_verified": self._bool(post_retreat_ok),
                    "safety_violation_count": str(self._int(safety.get("safety_violation_count", summary.get("safety_violation_count", "0")))),
                    "max_joint_delta_deg": f"{max_joint_delta:.9f}",
                    "contact_force_rise": f"{(trigger_force - initial_force):.9f}",
                    "contact_force_at_trigger": f"{trigger_force:.9f}",
                    "pre_contact_force_mean": f"{(sum(pre_contact) / len(pre_contact) if pre_contact else 0.0):.9f}",
                    "post_retreat_force_n": f"{post_force:.9f}",
                    "contact_binary_label": "1" if trigger_force > 0.0 else "0",
                    "safe_contact_label": "1" if trigger_force > 0.0 and safety_clear and stop_on_contact and retreat_completed else "0",
                    "offset_x_norm": f"{self._norm(self._float(first.get('x_offset_m')), self._max_offset):.9f}",
                    "offset_y_norm": f"{self._norm(self._float(first.get('y_offset_m')), self._max_offset):.9f}",
                    "yaw_norm": f"{self._norm(self._float(first.get('yaw_offset_deg')), self._max_yaw):.9f}",
                    "trigger_step_norm": f"{self._norm(float(contact_trigger_step), self._max_trigger_step):.9f}",
                    "force_norm": f"{self._norm(max_force, self._force_norm):.9f}",
                    "rgb_depth_available_flag": "1" if rgb_available and depth_available else "0",
                    "safety_clear_flag": str(safety_clear),
                }
            )
        return vectors

    def _contact_transition_vectors(self, transitions: list[dict[str, str]], vectors: list[dict[str, str]]) -> list[dict[str, str]]:
        vector_by = {row["scenario_id"]: row for row in vectors}
        rows = []
        for transition in transitions:
            scenario = transition.get("scenario", "")
            context = vector_by.get(scenario, {})
            rows.append(
                {
                    "scenario_id": scenario,
                    "step_index": transition.get("step_index", ""),
                    "force_n": self._fmt(transition.get("force_n")),
                    "torque_nm": self._fmt(transition.get("torque_nm")),
                    "contact_detection_force_threshold_n": self._fmt(transition.get("contact_detection_force_threshold_n")),
                    "contact_validation_min_force_n": self._fmt(transition.get("contact_validation_min_force_n")),
                    "contact_gate_triggered": self._bool(self._bool_text(transition.get("contact_gate_triggered"))),
                    "within_desired_contact_band": self._bool(self._bool_text(transition.get("within_desired_contact_band"))),
                    "emergency_stop_triggered": self._bool(self._bool_text(transition.get("emergency_stop_triggered"))),
                    "contact_binary_label": context.get("contact_binary_label", "0"),
                    "safe_contact_label": context.get("safe_contact_label", "0"),
                    "force_norm": context.get("force_norm", "0.000000000"),
                    "trigger_step_norm": context.get("trigger_step_norm", "0.000000000"),
                }
            )
        return rows

    def _episode_summaries(self, vectors: list[dict[str, str]]) -> list[dict[str, str]]:
        return [
            {
                "scenario_id": row["scenario_id"],
                "observation_count": row["observation_count"],
                "contact_trigger_step_index": row["contact_trigger_step_index"],
                "contact_force_at_trigger": row["contact_force_at_trigger"],
                "max_force_n": row["max_force_n"],
                "safe_contact_label": row["safe_contact_label"],
                "rgb_depth_available_flag": row["rgb_depth_available_flag"],
                "safety_clear_flag": row["safety_clear_flag"],
                "learning_executed": "false",
                "policy_training_executed": "false",
            }
            for row in vectors
        ]

    def _channel_summary(self, channel_rows: list[dict[str, str]]) -> list[dict[str, str]]:
        result = []
        for row in channel_rows:
            result.append(
                {
                    "channel": row.get("channel", ""),
                    "available": row.get("available", "false"),
                    "sample_count": row.get("sample_count", "0"),
                    "used_for_context": self._bool(row.get("channel", "") in {"rgb_image", "depth_image", "camera_info", "joint_states", "tf_tool_pose", "contact_wrench", "task_phase", "scenario_metadata", "observation_vector", "contact_transition_label"}),
                }
            )
        return result

    def _safety_summary(self, vectors: list[dict[str, str]], safety_rows: list[dict[str, str]]) -> list[dict[str, str]]:
        safety_by = {row.get("scenario", ""): row for row in safety_rows}
        return [
            {
                "scenario_id": row["scenario_id"],
                "safety_clear_flag": row["safety_clear_flag"],
                "safe_contact_label": row["safe_contact_label"],
                "safety_violation_count": row["safety_violation_count"],
                "peg_insertion_executed": safety_by.get(row["scenario_id"], {}).get("peg_insertion_executed", "false"),
                "forceful_contact_executed": safety_by.get(row["scenario_id"], {}).get("forceful_contact_executed", "false"),
                "real_robot_used": safety_by.get(row["scenario_id"], {}).get("real_robot_used", "false"),
            }
            for row in vectors
        ]

    def _write_outputs(
        self,
        paths: dict[str, Path],
        found: dict[str, bool],
        observations: list[dict[str, str]],
        transitions: list[dict[str, str]],
        vectors: list[dict[str, str]],
        transition_vectors: list[dict[str, str]],
        episodes: list[dict[str, str]],
        status: str,
        channel_summary: list[dict[str, str]] | None = None,
        safety_summary: list[dict[str, str]] | None = None,
    ) -> None:
        channel_summary = channel_summary or []
        safety_summary = safety_summary or []
        payload = self._status_payload(found, observations, transitions, vectors, transition_vectors, episodes, channel_summary, safety_summary, status)
        self._write_lines(self._output_dir / "nodes.txt", sorted(name for name in self.get_node_names() if name))
        self._write_lines(self._output_dir / "topics.txt", sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types()))
        self._write_lines(self._output_dir / "services.txt", sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types()))
        self._write_lines(self._output_dir / "parameters.txt", self._parameter_lines(paths))
        self._write_json(self._output_dir / "context_vector_extraction_status.json", payload)
        self._write_csv(self._output_dir / "scenario_context_vectors.csv", vectors)
        self._write_json(self._output_dir / "scenario_context_vectors.json", {"scenario_context_vectors": vectors})
        self._write_csv(self._output_dir / "contact_transition_feature_vectors.csv", transition_vectors)
        self._write_csv(self._output_dir / "episode_summary_table.csv", episodes)
        self._write_csv(self._output_dir / "observation_channel_context_summary.csv", channel_summary)
        self._write_csv(self._output_dir / "safety_gated_context_summary.csv", safety_summary)
        self._write_json(self._output_dir / "context_dataset_manifest.json", self._manifest(paths, payload))
        self._write_summary(payload)
        self._write_run_log(payload)
        self._publish_json(self._status_pub, payload)
        self._publish_json(self._context_pub, {"scenario_context_vectors": vectors})
        self._publish_json(self._transition_pub, {"contact_transition_feature_vectors": transition_vectors})
        self._finished = True
        self.get_logger().info("proposal_simulation_cell_v2_12 context vector extraction diagnostics written")
        rclpy.shutdown()

    def _status_payload(
        self,
        found: dict[str, bool],
        observations: list[dict[str, str]],
        transitions: list[dict[str, str]],
        vectors: list[dict[str, str]],
        transition_vectors: list[dict[str, str]],
        episodes: list[dict[str, str]],
        channel_summary: list[dict[str, str]],
        safety_summary: list[dict[str, str]],
        status: str,
    ) -> dict[str, Any]:
        return {
            "source_sprint": self._source_sprint,
            "source_observation_log_found": found.get("observation", False),
            "source_contact_transition_log_found": found.get("contact_transition", False),
            "source_scenario_summary_found": found.get("scenario_summary", False),
            "observation_row_count": len(observations),
            "contact_transition_row_count": len(transitions),
            "scenario_count": len({row.get("scenario_id", "") for row in vectors if row.get("scenario_id")}),
            "context_vectors_written": bool(vectors),
            "context_vector_count": len(vectors),
            "contact_transition_vectors_written": bool(transition_vectors),
            "episode_summary_written": bool(episodes),
            "channel_summary_written": bool(channel_summary),
            "safety_summary_written": bool(safety_summary),
            "fake_dataset_created": self._fake_dataset_created,
            "fake_result_created": self._fake_result_created,
            "learning_executed": self._learning_executed,
            "policy_training_executed": self._policy_training_executed,
            "real_robot_used": self._real_robot_used,
            "peg_insertion_executed": self._peg_insertion_executed,
            "forceful_contact_executed": self._forceful_contact_executed,
            "status": status,
        }

    def _manifest(self, paths: dict[str, Path], payload: dict[str, Any]) -> dict[str, Any]:
        return {
            "task_type": self._task_type,
            "source_sprint": self._source_sprint,
            "source_is_real_simulation_log": True,
            "source_files": {key: str(path) for key, path in paths.items()},
            "generated_files": [
                "context_vector_extraction_status.json",
                "scenario_context_vectors.csv",
                "scenario_context_vectors.json",
                "contact_transition_feature_vectors.csv",
                "episode_summary_table.csv",
                "observation_channel_context_summary.csv",
                "safety_gated_context_summary.csv",
                "context_dataset_manifest.json",
                "summary.md",
            ],
            "row_counts": {
                "observation_rows": payload["observation_row_count"],
                "contact_transition_rows": payload["contact_transition_row_count"],
                "context_vectors": payload["context_vector_count"],
            },
            "fake_dataset_created": False,
            "fake_result_created": False,
            "learning_executed": False,
            "policy_training_executed": False,
            "real_robot_used": False,
            "peg_insertion_executed": False,
            "forceful_contact_executed": False,
        }

    def _valid_success(
        self,
        found: dict[str, bool],
        observations: list[dict[str, str]],
        transitions: list[dict[str, str]],
        vectors: list[dict[str, str]],
    ) -> bool:
        return bool(
            found.get("observation")
            and found.get("contact_transition")
            and observations
            and transitions
            and len({row.get("scenario_id", "") for row in vectors if row.get("scenario_id")}) >= 5
            and len(vectors) >= 5
            and not self._fake_dataset_created
            and not self._learning_executed
        )

    def _max_joint_delta(self, rows: list[dict[str, str]]) -> float:
        parsed = [self._parse_joint_positions(row.get("joint_positions", "")) for row in rows]
        parsed = [row for row in parsed if row]
        if not parsed:
            return 0.0
        baseline = parsed[0]
        max_delta = 0.0
        for row in parsed[1:]:
            for joint, value in row.items():
                if joint in baseline:
                    max_delta = max(max_delta, abs(math.degrees(value - baseline[joint])))
        return max_delta

    def _parse_joint_positions(self, text: str) -> dict[str, float]:
        result = {}
        for item in text.split(";"):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            result[key] = self._float(value)
        return result

    def _parameter_lines(self, paths: dict[str, Path]) -> list[str]:
        lines = [
            "task_type=context_vector_extraction",
            f"source_sprint={self._source_sprint}",
            "source_is_real_simulation_log=true",
            "fake_dataset_created=false",
            "fake_result_created=false",
            "learning_executed=false",
            "policy_training_executed=false",
            "real_robot_used=false",
            "peg_insertion_executed=false",
            "forceful_contact_executed=false",
        ]
        lines.extend(f"input_{key}={path}" for key, path in paths.items())
        return lines

    def _write_summary(self, payload: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_12_context_vector_extraction",
            "",
            f"Status: `{payload['status']}`",
            "",
            "This diagnostic extracts compact context vectors from real v2.11 Gazebo simulation observation logs.",
            "",
            f"- source_sprint: {payload['source_sprint']}",
            f"- observation_row_count: {payload['observation_row_count']}",
            f"- contact_transition_row_count: {payload['contact_transition_row_count']}",
            f"- scenario_count: {payload['scenario_count']}",
            f"- context_vector_count: {payload['context_vector_count']}",
            "- fake_dataset_created: false",
            "- fake_result_created: false",
            "- learning_executed: false",
            "- policy_training_executed: false",
            "- real_robot_used: false",
            "- peg_insertion_executed: false",
            "- forceful_contact_executed: false",
        ]
        self._write_lines(self._output_dir / "summary.md", lines)

    def _write_run_log(self, payload: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_12_context_vector_extraction",
            f"status={payload['status']}",
            f"source_sprint={payload['source_sprint']}",
            f"observation_row_count={payload['observation_row_count']}",
            f"contact_transition_row_count={payload['contact_transition_row_count']}",
            f"context_vector_count={payload['context_vector_count']}",
            "fake_dataset_created=false",
            "fake_result_created=false",
            "learning_executed=false",
            "policy_training_executed=false",
            "real_robot_used=false",
        ]
        self._write_lines(self._output_dir / "run.log", lines)

    def _read_csv(self, path: Path) -> list[dict[str, str]]:
        with path.open("r", encoding="utf-8", newline="") as csv_file:
            return [dict(row) for row in csv.DictReader(csv_file)]

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

    def _float(self, value: Any) -> float:
        try:
            result = float(str(value))
        except (TypeError, ValueError):
            return 0.0
        if math.isnan(result) or math.isinf(result):
            return 0.0
        return result

    def _int(self, value: Any) -> int:
        return int(round(self._float(value)))

    def _fmt(self, value: Any) -> str:
        return f"{self._float(value):.9f}"

    def _norm(self, value: float, scale: float) -> float:
        if scale <= 0.0:
            return 0.0
        return max(-1.0, min(1.0, value / scale))

    def _bool_text(self, value: Any) -> bool:
        return str(value).strip().lower() in {"true", "1", "yes"}

    def _bool(self, value: Any) -> str:
        return str(bool(value)).lower()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV212ContextVectorExtractionNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
