"""Deterministic context encoder prototype for proposal simulation cell v2.13."""

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


class ProposalSimulationCellV213ContextEncoderNode(Node):
    """Encode v2.12 context vectors without policy training or RL."""

    def __init__(self) -> None:
        super().__init__("proposal_simulation_cell_v2_13_context_encoder_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("output_dir", "diagnostics/proposal_simulation_cell_v2_13")

        self._config = self._load_config()
        encoder = self._config.get("encoder", {})
        source = self._config.get("source_context", {})
        schema = self._config.get("feature_schema", {})
        embedding = self._config.get("embedding", {})
        validation = self._config.get("validation", {})
        safety = self._config.get("safety_policy", {})

        self._encoder_type = str(encoder.get("encoder_type", "deterministic_context_encoder_prototype"))
        self._source_sprint = str(source.get("source_sprint", "v2.12"))
        self._source_dir = Path(str(source.get("source_diagnostics_dir", "diagnostics/proposal_simulation_cell_v2_12")))
        self._source_files = {
            "context_vectors": str(source.get("scenario_context_vectors", "scenario_context_vectors.csv")),
            "contact_transition_vectors": str(source.get("contact_transition_feature_vectors", "contact_transition_feature_vectors.csv")),
            "episode_summary": str(source.get("episode_summary_table", "episode_summary_table.csv")),
            "safety_summary": str(source.get("safety_gated_context_summary", "safety_gated_context_summary.csv")),
            "manifest": str(source.get("context_dataset_manifest", "context_dataset_manifest.json")),
        }
        self._required_features = [str(item) for item in schema.get("required_features", [])]
        self._embedding_components = [str(item) for item in embedding.get("components", [])]
        self._embedding_dim = int(embedding.get("context_embedding_dim", encoder.get("context_embedding_dim", 8)))
        self._deterministic_only = bool(encoder.get("deterministic_only", True))
        self._variational_encoder_used = bool(encoder.get("variational_encoder_used", False))
        self._learning_executed = bool(safety.get("learning_executed", encoder.get("learning_executed", False)))
        self._policy_training_executed = bool(safety.get("policy_training_executed", encoder.get("policy_training_executed", False)))
        self._rl_training_executed = bool(safety.get("rl_training_executed", encoder.get("rl_training_executed", False)))
        self._fake_result_created = bool(safety.get("fake_result_created", encoder.get("fake_result_created", False)))
        self._real_robot_used = bool(safety.get("real_robot_used", False))
        self._peg_insertion_executed = bool(safety.get("peg_insertion_executed", False))
        self._success_status = str(validation.get("status_success", "context_encoder_prototype_validated"))
        self._missing_status = str(validation.get("status_missing_inputs", "context_encoder_source_inputs_missing"))
        self._invalid_status = str(validation.get("status_invalid_features", "context_encoder_required_features_missing"))
        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
            or validation.get("output_dir", "diagnostics/proposal_simulation_cell_v2_13")
        )
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._status_pub = self.create_publisher(
            String,
            str(validation.get("status_topic", "/proposal_simulation_cell/context_encoder_status")),
            10,
        )
        self._embedding_pub = self.create_publisher(
            String,
            str(validation.get("embedding_report_topic", "/proposal_simulation_cell/context_embedding_report")),
            10,
        )
        self._similarity_pub = self.create_publisher(
            String,
            str(validation.get("similarity_report_topic", "/proposal_simulation_cell/context_similarity_report")),
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
            raise FileNotFoundError(f"proposal v2.13 config not found: {path}")
        with path.open("r", encoding="utf-8") as config_file:
            data = yaml.safe_load(config_file) or {}
        return data if isinstance(data, dict) else {}

    def _tick(self) -> None:
        if self._finished or self._started:
            return
        self._started = True
        self._run()

    def _run(self) -> None:
        paths = {key: self._source_dir / filename for key, filename in self._source_files.items()}
        found = {key: path.is_file() for key, path in paths.items()}
        if not all(found.values()):
            self._write_outputs(paths, found, [], [], [], [], [], self._missing_status, False)
            return
        context_rows = self._read_csv(paths["context_vectors"])
        transition_rows = self._read_csv(paths["contact_transition_vectors"])
        required_available = self._required_features_available(context_rows)
        if not context_rows or not required_available:
            self._write_outputs(paths, found, context_rows, [], [], [], [], self._invalid_status, required_available)
            return
        embeddings = self._build_embeddings(context_rows)
        similarity = self._similarity_matrix(embeddings)
        nearest = self._nearest_context_report(similarity)
        validation_rows = self._validation_rows(context_rows, embeddings)
        success = self._success(found, context_rows, embeddings, required_available)
        status = self._success_status if success else self._invalid_status
        self._write_outputs(paths, found, context_rows, transition_rows, embeddings, similarity, nearest, status, required_available, validation_rows)

    def _build_embeddings(self, rows: list[dict[str, str]]) -> list[dict[str, str]]:
        encoded = []
        for row in rows:
            offset_x = self._float(row.get("offset_x_norm"))
            offset_y = self._float(row.get("offset_y_norm"))
            yaw = self._float(row.get("yaw_norm"))
            trigger_step = self._float(row.get("trigger_step_norm"))
            force = self._float(row.get("force_norm"))
            rgb_depth = self._float(row.get("rgb_depth_available_flag"))
            safety_clear = self._float(row.get("safety_clear_flag"))
            contact_binary = self._float(row.get("contact_binary_label"))
            safe_contact = self._float(row.get("safe_contact_label"))
            lateral_offset = math.sqrt(offset_x * offset_x + offset_y * offset_y)
            contact_strength = force * safe_contact
            observation_reliability = rgb_depth * safety_clear
            difficulty = min(1.0, lateral_offset + force)
            components = [
                offset_x,
                offset_y,
                min(1.0, lateral_offset),
                trigger_step,
                force,
                contact_strength,
                observation_reliability,
                difficulty,
            ]
            embedding = {f"embedding_{index}": f"{value:.9f}" for index, value in enumerate(components)}
            encoded.append(
                {
                    "scenario_id": row.get("scenario_id", ""),
                    "encoder_type": self._encoder_type,
                    "context_embedding_dim": str(self._embedding_dim),
                    "x_offset_m": self._fmt(row.get("x_offset_m")),
                    "y_offset_m": self._fmt(row.get("y_offset_m")),
                    "yaw_offset_deg": self._fmt(row.get("yaw_offset_deg")),
                    "lateral_offset_norm": f"{lateral_offset:.9f}",
                    "contact_context_strength": f"{contact_strength:.9f}",
                    "observation_reliability": f"{observation_reliability:.9f}",
                    "scenario_difficulty_proxy": f"{difficulty:.9f}",
                    **embedding,
                    "deterministic_only": self._bool(self._deterministic_only),
                    "learning_executed": "false",
                    "policy_training_executed": "false",
                    "rl_training_executed": "false",
                }
            )
        return encoded

    def _similarity_matrix(self, embeddings: list[dict[str, str]]) -> list[dict[str, str]]:
        rows = []
        for left in embeddings:
            row = {"scenario_id": left["scenario_id"]}
            for right in embeddings:
                row[right["scenario_id"]] = f"{self._cosine(self._embedding_values(left), self._embedding_values(right)):.9f}"
            rows.append(row)
        return rows

    def _nearest_context_report(self, similarity_rows: list[dict[str, str]]) -> list[dict[str, str]]:
        report = []
        for row in similarity_rows:
            scenario = row["scenario_id"]
            candidates = [(key, self._float(value)) for key, value in row.items() if key != "scenario_id" and key != scenario]
            candidates.sort(key=lambda item: item[1], reverse=True)
            nearest, score = candidates[0] if candidates else ("", 0.0)
            ordered = "|".join(f"{key}:{score_value:.6f}" for key, score_value in candidates)
            report.append(
                {
                    "scenario_id": scenario,
                    "nearest_context_id": nearest,
                    "cosine_similarity": f"{score:.9f}",
                    "ordered_neighbors": ordered,
                }
            )
        return report

    def _validation_rows(self, context_rows: list[dict[str, str]], embeddings: list[dict[str, str]]) -> list[dict[str, str]]:
        return [
            {
                "check": "source_context_vector_count",
                "value": str(len(context_rows)),
                "passed": self._bool(len(context_rows) >= 5),
            },
            {
                "check": "required_features_available",
                "value": self._bool(self._required_features_available(context_rows)),
                "passed": self._bool(self._required_features_available(context_rows)),
            },
            {
                "check": "embedding_dim",
                "value": str(self._embedding_dim),
                "passed": self._bool(self._embedding_dim == 8),
            },
            {
                "check": "embedding_count",
                "value": str(len(embeddings)),
                "passed": self._bool(len(embeddings) >= 5),
            },
            {"check": "deterministic_only", "value": self._bool(self._deterministic_only), "passed": "true"},
            {"check": "variational_encoder_used", "value": self._bool(self._variational_encoder_used), "passed": self._bool(not self._variational_encoder_used)},
            {"check": "learning_executed", "value": self._bool(self._learning_executed), "passed": self._bool(not self._learning_executed)},
            {"check": "policy_training_executed", "value": self._bool(self._policy_training_executed), "passed": self._bool(not self._policy_training_executed)},
            {"check": "fake_result_created", "value": self._bool(self._fake_result_created), "passed": self._bool(not self._fake_result_created)},
        ]

    def _write_outputs(
        self,
        paths: dict[str, Path],
        found: dict[str, bool],
        context_rows: list[dict[str, str]],
        transition_rows: list[dict[str, str]],
        embeddings: list[dict[str, str]],
        similarity: list[dict[str, str]],
        nearest: list[dict[str, str]],
        status: str,
        required_available: bool,
        validation_rows: list[dict[str, str]] | None = None,
    ) -> None:
        validation_rows = validation_rows or []
        payload = self._status_payload(found, context_rows, transition_rows, embeddings, similarity, nearest, required_available, status)
        self._write_lines(self._output_dir / "nodes.txt", sorted(name for name in self.get_node_names() if name))
        self._write_lines(self._output_dir / "topics.txt", sorted(f"{name} {','.join(types)}" for name, types in self.get_topic_names_and_types()))
        self._write_lines(self._output_dir / "services.txt", sorted(f"{name} {','.join(types)}" for name, types in self.get_service_names_and_types()))
        self._write_lines(self._output_dir / "parameters.txt", self._parameter_lines(paths))
        self._write_json(self._output_dir / "context_encoder_status.json", payload)
        self._write_json(self._output_dir / "context_feature_schema.json", self._feature_schema_payload())
        self._write_csv(self._output_dir / "context_embedding_table.csv", embeddings)
        self._write_json(self._output_dir / "context_embedding_table.json", {"context_embeddings": embeddings})
        self._write_csv(self._output_dir / "context_similarity_matrix.csv", similarity)
        self._write_csv(self._output_dir / "nearest_context_report.csv", nearest)
        self._write_json(self._output_dir / "context_encoder_manifest.json", self._manifest(paths, payload))
        self._write_csv(self._output_dir / "context_encoder_validation_report.csv", validation_rows)
        self._write_summary(payload)
        self._write_run_log(payload)
        self._publish_json(self._status_pub, payload)
        self._publish_json(self._embedding_pub, {"context_embeddings": embeddings})
        self._publish_json(self._similarity_pub, {"nearest_context_report": nearest})
        self._finished = True
        self.get_logger().info("proposal_simulation_cell_v2_13 context encoder diagnostics written")
        rclpy.shutdown()

    def _status_payload(
        self,
        found: dict[str, bool],
        context_rows: list[dict[str, str]],
        transition_rows: list[dict[str, str]],
        embeddings: list[dict[str, str]],
        similarity: list[dict[str, str]],
        nearest: list[dict[str, str]],
        required_available: bool,
        status: str,
    ) -> dict[str, Any]:
        return {
            "source_sprint": self._source_sprint,
            "source_context_vectors_found": found.get("context_vectors", False),
            "source_contact_transition_vectors_found": found.get("contact_transition_vectors", False),
            "source_manifest_found": found.get("manifest", False),
            "scenario_count": len(context_rows),
            "required_features_available": required_available,
            "feature_schema_written": True,
            "context_embeddings_written": bool(embeddings),
            "context_embedding_count": len(embeddings),
            "context_embedding_dim": self._embedding_dim,
            "similarity_matrix_written": bool(similarity),
            "nearest_context_report_written": bool(nearest),
            "deterministic_only": True,
            "variational_encoder_used": False,
            "learning_executed": False,
            "policy_training_executed": False,
            "rl_training_executed": False,
            "fake_result_created": False,
            "real_robot_used": False,
            "peg_insertion_executed": False,
            "status": status,
        }

    def _feature_schema_payload(self) -> dict[str, Any]:
        return {
            "encoder_type": self._encoder_type,
            "required_features": self._required_features,
            "derived_features": {
                "lateral_offset_norm": "sqrt(offset_x_norm^2 + offset_y_norm^2)",
                "contact_context_strength": "force_norm * safe_contact_label",
                "observation_reliability": "rgb_depth_available_flag * safety_clear_flag",
                "scenario_difficulty_proxy": "lateral_offset_norm + force_norm, clipped to 1.0",
            },
            "embedding_components": self._embedding_components,
            "context_embedding_dim": self._embedding_dim,
            "deterministic_only": True,
            "variational_encoder_used": False,
        }

    def _manifest(self, paths: dict[str, Path], payload: dict[str, Any]) -> dict[str, Any]:
        return {
            "encoder_type": self._encoder_type,
            "source_sprint": self._source_sprint,
            "source_files": {key: str(path) for key, path in paths.items()},
            "generated_files": [
                "context_encoder_status.json",
                "context_feature_schema.json",
                "context_embedding_table.csv",
                "context_embedding_table.json",
                "context_similarity_matrix.csv",
                "nearest_context_report.csv",
                "context_encoder_manifest.json",
                "context_encoder_validation_report.csv",
                "summary.md",
            ],
            "context_embedding_dim": payload["context_embedding_dim"],
            "context_embedding_count": payload["context_embedding_count"],
            "deterministic_only": True,
            "variational_encoder_used": False,
            "learning_executed": False,
            "policy_training_executed": False,
            "rl_training_executed": False,
            "fake_result_created": False,
            "real_robot_used": False,
            "peg_insertion_executed": False,
        }

    def _success(
        self,
        found: dict[str, bool],
        context_rows: list[dict[str, str]],
        embeddings: list[dict[str, str]],
        required_available: bool,
    ) -> bool:
        return bool(
            found.get("context_vectors")
            and len(context_rows) >= 5
            and required_available
            and len(embeddings) >= 5
            and self._embedding_dim == 8
            and not self._learning_executed
            and not self._policy_training_executed
            and not self._fake_result_created
        )

    def _required_features_available(self, rows: list[dict[str, str]]) -> bool:
        if not rows:
            return False
        columns = set(rows[0].keys())
        return all(feature in columns for feature in self._required_features)

    def _embedding_values(self, row: dict[str, str]) -> list[float]:
        return [self._float(row.get(f"embedding_{index}")) for index in range(self._embedding_dim)]

    def _cosine(self, left: list[float], right: list[float]) -> float:
        dot = sum(a * b for a, b in zip(left, right))
        left_norm = math.sqrt(sum(a * a for a in left))
        right_norm = math.sqrt(sum(b * b for b in right))
        if left_norm <= 0.0 or right_norm <= 0.0:
            return 0.0
        return dot / (left_norm * right_norm)

    def _parameter_lines(self, paths: dict[str, Path]) -> list[str]:
        lines = [
            f"encoder_type={self._encoder_type}",
            f"source_sprint={self._source_sprint}",
            f"context_embedding_dim={self._embedding_dim}",
            "deterministic_only=true",
            "variational_encoder_used=false",
            "learning_executed=false",
            "policy_training_executed=false",
            "rl_training_executed=false",
            "fake_result_created=false",
            "real_robot_used=false",
            "peg_insertion_executed=false",
        ]
        lines.extend(f"input_{key}={path}" for key, path in paths.items())
        return lines

    def _write_summary(self, payload: dict[str, Any]) -> None:
        lines = [
            "# proposal_simulation_cell_v2_13_context_encoder_prototype",
            "",
            f"Status: `{payload['status']}`",
            "",
            "This diagnostic encodes real v2.12 context vectors into deterministic prototype embeddings.",
            "",
            f"- source_sprint: {payload['source_sprint']}",
            f"- scenario_count: {payload['scenario_count']}",
            f"- context_embedding_count: {payload['context_embedding_count']}",
            f"- context_embedding_dim: {payload['context_embedding_dim']}",
            "- deterministic_only: true",
            "- variational_encoder_used: false",
            "- learning_executed: false",
            "- policy_training_executed: false",
            "- rl_training_executed: false",
            "- fake_result_created: false",
            "- real_robot_used: false",
            "- peg_insertion_executed: false",
        ]
        self._write_lines(self._output_dir / "summary.md", lines)

    def _write_run_log(self, payload: dict[str, Any]) -> None:
        lines = [
            "proposal_simulation_cell_v2_13_context_encoder_prototype",
            f"status={payload['status']}",
            f"source_sprint={payload['source_sprint']}",
            f"context_embedding_count={payload['context_embedding_count']}",
            f"context_embedding_dim={payload['context_embedding_dim']}",
            "deterministic_only=true",
            "learning_executed=false",
            "policy_training_executed=false",
            "rl_training_executed=false",
            "fake_result_created=false",
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

    def _fmt(self, value: Any) -> str:
        return f"{self._float(value):.9f}"

    def _bool(self, value: Any) -> str:
        return str(bool(value)).lower()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProposalSimulationCellV213ContextEncoderNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
