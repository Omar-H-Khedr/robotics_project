#!/usr/bin/env python3
"""Analyze INSERT tracking and RETREAT contact evidence for baseline runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

import numpy as np

from kuka_task_control.robot_kinematics import RobotKinematics


FINAL_INSERTION_POSE = np.array([0.520, -0.200, 0.790])
HOLE_TOP_Z = 0.810
COMMAND_TARGET_TOLERANCE_M = 0.035
CONTROLLER_STATE_TRACKING_FILE = "trajectory_controller_state_samples.csv"
JOINT_STATE_TRACKING_FILE = "trajectory_tracking_samples.csv"
CONTACT_FILE = "contact_state_samples.csv"
WRENCH_FILE = "wrench_state_samples.csv"


@dataclass(frozen=True)
class CommandRow:
    receipt_stamp_s: float
    joint_names: list[str]
    point_count: int
    final_time_from_start_s: float
    final_positions_rad: list[float]
    target_xyz_m: list[float]


@dataclass(frozen=True)
class TrackingSample:
    stamp_s: float
    max_abs_error_rad: float
    rms_error_rad: float
    reference: list[float]
    feedback: list[float]
    error: list[float]


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(
        len(ordered) - 1,
        max(0, int(round((percentile / 100.0) * (len(ordered) - 1)))),
    )
    return ordered[index]


def _float_list(text: str) -> list[float]:
    return [float(value) for value in text.split() if value.strip()]


def _read_commands(path: Path) -> list[CommandRow]:
    kin = RobotKinematics()
    rows: list[CommandRow] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            final_positions = _float_list(row["final_positions_rad"])
            target_xyz, _ = kin.pose(np.array(final_positions, dtype=float))
            rows.append(
                CommandRow(
                    receipt_stamp_s=float(row["receipt_stamp_s"]),
                    joint_names=row["joint_names"].split(),
                    point_count=int(row["point_count"]),
                    final_time_from_start_s=float(row["final_time_from_start_s"]),
                    final_positions_rad=final_positions,
                    target_xyz_m=[float(value) for value in target_xyz],
                )
            )
    return rows


def _read_tracking(path: Path, joint_names: list[str]) -> list[TrackingSample]:
    samples: list[TrackingSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            reference = [float(row[f"{name}_reference_rad"]) for name in joint_names]
            feedback = [float(row[f"{name}_feedback_rad"]) for name in joint_names]
            error = [float(row[f"{name}_error_rad"]) for name in joint_names]
            samples.append(
                TrackingSample(
                    stamp_s=float(row["stamp_s"]),
                    max_abs_error_rad=float(row["max_abs_position_error_rad"]),
                    rms_error_rad=float(row["rms_position_error_rad"]),
                    reference=reference,
                    feedback=feedback,
                    error=error,
                )
            )
    return samples


def _select_insert_command(commands: list[CommandRow]) -> int | None:
    best_index: int | None = None
    best_error = math.inf
    for index, command in enumerate(commands):
        target = np.array(command.target_xyz_m, dtype=float)
        error = float(np.linalg.norm(target - FINAL_INSERTION_POSE))
        if error < best_error:
            best_index = index
            best_error = error
    if best_index is None or best_error > COMMAND_TARGET_TOLERANCE_M:
        return None
    return best_index


def _contact_summary(path: Path) -> dict[str, object]:
    if not path.exists():
        return {"available": False}

    by_state: dict[str, dict[str, object]] = defaultdict(
        lambda: {
            "samples": 0,
            "positive_samples": 0,
            "max_contact_force_n": 0.0,
            "collision_pairs": defaultdict(lambda: {"samples": 0, "max_force_n": 0.0}),
        }
    )
    first_stamp_by_state: dict[str, float] = {}

    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            state = row["state"]
            force = float(row["max_contact_force_n"])
            count = int(row["contact_count"])
            stamp = float(row["stamp_s"])
            data = by_state[state]
            data["samples"] = int(data["samples"]) + 1
            if count > 0:
                data["positive_samples"] = int(data["positive_samples"]) + 1
            data["max_contact_force_n"] = max(float(data["max_contact_force_n"]), force)
            first_stamp_by_state.setdefault(state, stamp)
            for pair in row["collision_pairs"].split("; "):
                if not pair:
                    continue
                pairs = data["collision_pairs"]
                pair_data = pairs[pair]
                pair_data["samples"] += 1
                pair_data["max_force_n"] = max(pair_data["max_force_n"], force)

    states: dict[str, object] = {}
    for state, data in by_state.items():
        pairs = data["collision_pairs"]
        top_pairs = sorted(
            pairs.items(),
            key=lambda item: (item[1]["max_force_n"], item[1]["samples"]),
            reverse=True,
        )[:5]
        states[state] = {
            "first_stamp_s": first_stamp_by_state[state],
            "samples": data["samples"],
            "positive_samples": data["positive_samples"],
            "max_contact_force_n": data["max_contact_force_n"],
            "top_collision_pairs": [
                {
                    "collision_pair": pair,
                    "samples": pair_data["samples"],
                    "max_force_n": pair_data["max_force_n"],
                }
                for pair, pair_data in top_pairs
            ],
        }
    return {"available": True, "states": states}


def _wrench_summary(path: Path) -> dict[str, object]:
    if not path.exists():
        return {"available": False}
    states: dict[str, dict[str, list[float] | int]] = defaultdict(
        lambda: {"samples": 0, "fz": [], "force_norm": [], "xy_error": []}
    )
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            state = row["state"]
            data = states[state]
            data["samples"] = int(data["samples"]) + 1
            data["fz"].append(float(row["fz_n"]))
            data["force_norm"].append(float(row["force_norm_n"]))
            xy_error = float(row["xy_error_m"])
            if math.isfinite(xy_error):
                data["xy_error"].append(xy_error)

    result: dict[str, object] = {}
    for state, data in states.items():
        fz = data["fz"]
        norms = data["force_norm"]
        xy = data["xy_error"]
        result[state] = {
            "samples": data["samples"],
            "max_abs_fz_n": max((abs(value) for value in fz), default=0.0),
            "max_force_norm_n": max(norms, default=0.0),
            "mean_xy_error_m": mean(xy) if xy else 0.0,
        }
    return {"available": True, "states": result}


def _format_vec(values: np.ndarray | list[float], digits: int = 6) -> str:
    return ", ".join(f"{float(value):.{digits}f}" for value in values)


def analyze_directory(input_dir: Path, output_name: str, json_name: str, command_index: int) -> Path:
    commands_path = input_dir / "trajectory_commands.csv"
    controller_tracking_path = input_dir / CONTROLLER_STATE_TRACKING_FILE
    joint_tracking_path = input_dir / JOINT_STATE_TRACKING_FILE
    tracking_path = controller_tracking_path if controller_tracking_path.exists() else joint_tracking_path

    if not commands_path.exists():
        raise FileNotFoundError(f"missing {commands_path}")
    if not tracking_path.exists():
        raise FileNotFoundError(f"missing {tracking_path}")

    commands = _read_commands(commands_path)
    if not commands:
        raise RuntimeError(f"no commands in {commands_path}")
    if command_index < 0:
        selected = _select_insert_command(commands)
        if selected is None:
            raise RuntimeError(
                "no command target matched the canonical final insertion pose "
                f"within {COMMAND_TARGET_TOLERANCE_M} m"
            )
        command_index = selected
    if command_index < 0 or command_index >= len(commands):
        raise RuntimeError(f"command_index {command_index} out of range for {len(commands)} commands")

    insert_command = commands[command_index]
    next_command_index = command_index + 1
    next_stamp = commands[next_command_index].receipt_stamp_s if next_command_index < len(commands) else None
    samples = _read_tracking(tracking_path, insert_command.joint_names)
    insert_samples = [
        sample
        for sample in samples
        if sample.stamp_s >= insert_command.receipt_stamp_s
        and (next_stamp is None or sample.stamp_s < next_stamp)
    ]
    if not insert_samples:
        raise RuntimeError("no tracking samples in selected insert command window")

    kin = RobotKinematics()
    target_q = np.array(insert_command.final_positions_rad, dtype=float)
    target_pos, _ = kin.pose(target_q)
    final_sample = insert_samples[-1]
    feedback_q = np.array(final_sample.feedback, dtype=float)
    feedback_pos, _ = kin.pose(feedback_q)
    cart_error = target_pos - feedback_pos
    max_errors = [sample.max_abs_error_rad for sample in insert_samples]
    rms_errors = [sample.rms_error_rad for sample in insert_samples]
    feedback_positions = [kin.pose(np.array(sample.feedback, dtype=float))[0] for sample in insert_samples]
    min_feedback_z = min(float(pos[2]) for pos in feedback_positions)
    max_depth_m = max(0.0, HOLE_TOP_Z - min_feedback_z)
    final_depth_m = max(0.0, HOLE_TOP_Z - float(feedback_pos[2]))

    result = {
        "input_dir": str(input_dir),
        "tracking_source": tracking_path.name,
        "command_index": command_index,
        "command_receipt_stamp_s": insert_command.receipt_stamp_s,
        "next_command_stamp_s": next_stamp,
        "command_duration_s": insert_command.final_time_from_start_s,
        "observed_window_s": max(0.0, (next_stamp if next_stamp is not None else final_sample.stamp_s) - insert_command.receipt_stamp_s),
        "samples": len(insert_samples),
        "target_xyz_m": [float(value) for value in target_pos],
        "final_feedback_xyz_m": [float(value) for value in feedback_pos],
        "final_cartesian_error_xyz_m": [float(value) for value in cart_error],
        "final_cartesian_error_norm_m": float(np.linalg.norm(cart_error)),
        "missing_descent_to_target_m": float(feedback_pos[2] - target_pos[2]),
        "min_feedback_z_m": min_feedback_z,
        "final_feedback_z_m": float(feedback_pos[2]),
        "hole_top_z_m": HOLE_TOP_Z,
        "max_physical_depth_m": max_depth_m,
        "final_physical_depth_m": final_depth_m,
        "max_abs_position_error_rad": max(max_errors, default=0.0),
        "p95_max_abs_position_error_rad": _percentile(max_errors, 95.0),
        "mean_rms_position_error_rad": mean(rms_errors) if rms_errors else 0.0,
        "contact_summary": _contact_summary(input_dir / CONTACT_FILE),
        "wrench_summary": _wrench_summary(input_dir / WRENCH_FILE),
    }

    output_path = input_dir / output_name
    json_path = input_dir / json_name
    json_path.write_text(
        json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    contact_states = result["contact_summary"].get("states", {}) if result["contact_summary"].get("available") else {}
    wrench_states = result["wrench_summary"].get("states", {}) if result["wrench_summary"].get("available") else {}
    lines = [
        "# Insert / Retreat Contact Analysis",
        "",
        f"- input_dir: `{input_dir}`",
        f"- tracking_source: `{tracking_path.name}`",
        f"- command_index: `{command_index}`",
        f"- command_receipt_stamp_s: `{insert_command.receipt_stamp_s:.3f}`",
        f"- next_command_stamp_s: `{next_stamp:.3f}`" if next_stamp is not None else "- next_command_stamp_s: `none`",
        f"- insert_command_duration_s: `{insert_command.final_time_from_start_s:.3f}`",
        f"- observed_window_s: `{result['observed_window_s']:.3f}`",
        f"- samples: `{len(insert_samples)}`",
        f"- max_abs_position_error_rad: `{result['max_abs_position_error_rad']:.6f}`",
        f"- p95_max_abs_position_error_rad: `{result['p95_max_abs_position_error_rad']:.6f}`",
        f"- mean_rms_position_error_rad: `{result['mean_rms_position_error_rad']:.6f}`",
        "",
        "## Cartesian Peg Tip",
        "",
        f"- command_target_xyz_m: `{_format_vec(target_pos)}`",
        f"- final_feedback_xyz_m: `{_format_vec(feedback_pos)}`",
        f"- final_cartesian_error_xyz_m: `{_format_vec(cart_error)}`",
        f"- final_cartesian_error_norm_m: `{result['final_cartesian_error_norm_m']:.6f}`",
        f"- missing_descent_to_target_m: `{result['missing_descent_to_target_m']:.6f}`",
        f"- hole_top_z_m: `{HOLE_TOP_Z:.6f}`",
        f"- min_feedback_z_m: `{min_feedback_z:.6f}`",
        f"- max_physical_depth_m: `{max_depth_m:.6f}`",
        f"- final_physical_depth_m: `{final_depth_m:.6f}`",
        "",
        "## Contact By State",
        "",
    ]
    if not contact_states:
        lines.append("- contact_state_samples.csv unavailable or empty")
    else:
        lines.extend([
            "| state | first_stamp_s | samples | positive_samples | max_contact_force_n | top_collision_pair |",
            "| --- | ---: | ---: | ---: | ---: | --- |",
        ])
        for state in sorted(contact_states):
            data = contact_states[state]
            top_pair = data["top_collision_pairs"][0]["collision_pair"] if data["top_collision_pairs"] else ""
            lines.append(
                f"| {state} | {data['first_stamp_s']:.3f} | {data['samples']} | "
                f"{data['positive_samples']} | {data['max_contact_force_n']:.6f} | `{top_pair}` |"
            )
    lines.extend(["", "## Wrench By State", ""])
    if not wrench_states:
        lines.append("- wrench_state_samples.csv unavailable or empty")
    else:
        lines.extend([
            "| state | samples | max_abs_fz_n | max_force_norm_n | mean_xy_error_m |",
            "| --- | ---: | ---: | ---: | ---: |",
        ])
        for state in sorted(wrench_states):
            data = wrench_states[state]
            lines.append(
                f"| {state} | {data['samples']} | {data['max_abs_fz_n']:.6f} | "
                f"{data['max_force_norm_n']:.6f} | {data['mean_xy_error_m']:.6f} |"
            )
    lines.extend([
        "",
        "Interpretation: this is an offline diagnostic over passive observer CSVs. "
        "It does not publish commands or change task safety gates.",
    ])
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="Diagnostics directory containing trajectory/contact CSVs")
    parser.add_argument("--output-name", default="insert_retreat_contact_analysis.md")
    parser.add_argument("--json-name", default="insert_retreat_contact_analysis.json")
    parser.add_argument(
        "--command-index",
        type=int,
        default=-1,
        help=(
            "Zero-based command index to analyze. The default -1 selects the "
            "command whose FK target matches the canonical final insertion pose."
        ),
    )
    args = parser.parse_args()
    output_path = analyze_directory(args.input_dir, args.output_name, args.json_name, args.command_index)
    print(output_path)


if __name__ == "__main__":
    main()
