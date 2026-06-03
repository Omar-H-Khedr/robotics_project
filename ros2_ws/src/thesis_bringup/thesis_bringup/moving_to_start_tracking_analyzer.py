#!/usr/bin/env python3
"""Analyze MOVING_TO_START tracking by selecting the axis-align command."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

import numpy as np

from kuka_task_control.robot_kinematics import RobotKinematics


AXIS_ALIGN_POSE = np.array([0.520, -0.200, 0.885])
COMMAND_TARGET_TOLERANCE_M = 0.02
STRICT_XY_M = 0.002
FINAL_XY_WINDOW_S = 1.0
CONTROLLER_STATE_TRACKING_FILE = "trajectory_controller_state_samples.csv"
JOINT_STATE_TRACKING_FILE = "trajectory_tracking_samples.csv"


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
        reader = csv.DictReader(handle)
        for row in reader:
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
        reader = csv.DictReader(handle)
        for row in reader:
            feedback = [float(row[f"{name}_feedback_rad"]) for name in joint_names]
            error = [float(row[f"{name}_error_rad"]) for name in joint_names]
            samples.append(
                TrackingSample(
                    stamp_s=float(row["stamp_s"]),
                    max_abs_error_rad=float(row["max_abs_position_error_rad"]),
                    rms_error_rad=float(row["rms_position_error_rad"]),
                    feedback=feedback,
                    error=error,
                )
            )
    return samples


def _select_axis_align_command(commands: list[CommandRow]) -> int | None:
    best_index: int | None = None
    best_error = math.inf
    for index, command in enumerate(commands):
        target = np.array(command.target_xyz_m, dtype=float)
        error = float(np.linalg.norm(target - AXIS_ALIGN_POSE))
        if error < best_error:
            best_error = error
            best_index = index
    if best_index is None or best_error > COMMAND_TARGET_TOLERANCE_M:
        return None
    return best_index


def analyze(input_dir: Path) -> dict[str, object]:
    commands_path = input_dir / "trajectory_commands.csv"
    controller_tracking_path = input_dir / CONTROLLER_STATE_TRACKING_FILE
    joint_tracking_path = input_dir / JOINT_STATE_TRACKING_FILE
    tracking_path = (
        controller_tracking_path
        if controller_tracking_path.exists()
        else joint_tracking_path
    )
    if not commands_path.exists():
        raise FileNotFoundError(f"missing {commands_path}")
    if not tracking_path.exists():
        raise FileNotFoundError(f"missing {tracking_path}")

    commands = _read_commands(commands_path)
    command_index = _select_axis_align_command(commands)
    if command_index is None:
        return {
            "input_dir": str(input_dir),
            "result": "NO_AXIS_ALIGN_COMMAND_CAPTURED",
            "commands": len(commands),
            "axis_align_pose_m": AXIS_ALIGN_POSE.tolist(),
            "command_target_tolerance_m": COMMAND_TARGET_TOLERANCE_M,
            "command_targets_m": [command.target_xyz_m for command in commands],
        }

    command = commands[command_index]
    next_stamp = (
        commands[command_index + 1].receipt_stamp_s
        if command_index + 1 < len(commands)
        else None
    )
    samples = _read_tracking(tracking_path, command.joint_names)
    window = [
        sample
        for sample in samples
        if sample.stamp_s >= command.receipt_stamp_s
        and (next_stamp is None or sample.stamp_s < next_stamp)
    ]
    if not window:
        return {
            "input_dir": str(input_dir),
            "result": "NO_TRACKING_SAMPLES_FOR_AXIS_ALIGN_COMMAND",
            "command_index": command_index,
            "command_receipt_stamp_s": command.receipt_stamp_s,
            "tracking_source": tracking_path.name,
        }

    kin = RobotKinematics()
    target_q = np.array(command.final_positions_rad, dtype=float)
    target_pos = np.array(command.target_xyz_m, dtype=float)
    final = window[-1]
    feedback_q = np.array(final.feedback, dtype=float)
    feedback_pos, _ = kin.pose(feedback_q)
    cart_error = target_pos - feedback_pos
    xy_errors: list[float] = []
    for sample in window:
        sample_pos, _ = kin.pose(np.array(sample.feedback, dtype=float))
        xy_errors.append(float(np.linalg.norm(sample_pos[:2] - AXIS_ALIGN_POSE[:2])))
    final_window_start_s = max(command.receipt_stamp_s, final.stamp_s - FINAL_XY_WINDOW_S)
    final_xy_errors = [
        xy_error
        for sample, xy_error in zip(window, xy_errors)
        if sample.stamp_s >= final_window_start_s
    ]
    joint_rows: list[dict[str, object]] = []
    worst_p95 = ("", 0.0)
    worst_final = ("", 0.0)
    for index, name in enumerate(command.joint_names):
        abs_errors = [abs(sample.error[index]) for sample in window]
        p95_abs = _percentile(abs_errors, 95.0)
        final_error = float(final.error[index])
        if p95_abs > worst_p95[1]:
            worst_p95 = (name, p95_abs)
        if abs(final_error) > worst_final[1]:
            worst_final = (name, abs(final_error))
        joint_rows.append(
            {
                "joint": name,
                "max_abs_rad": max(abs_errors, default=0.0),
                "p95_abs_rad": p95_abs,
                "mean_abs_rad": mean(abs_errors) if abs_errors else 0.0,
                "final_error_rad": final_error,
                "target_minus_feedback_rad": float(target_q[index] - feedback_q[index]),
                "target_rad": float(target_q[index]),
                "final_feedback_rad": float(feedback_q[index]),
            }
        )

    command_end_s = command.receipt_stamp_s + command.final_time_from_start_s
    max_errors = [sample.max_abs_error_rad for sample in window]
    rms_errors = [sample.rms_error_rad for sample in window]
    return {
        "input_dir": str(input_dir),
        "result": "OK",
        "tracking_source": tracking_path.name,
        "command_index": command_index,
        "command_receipt_stamp_s": command.receipt_stamp_s,
        "next_command_stamp_s": next_stamp,
        "command_duration_s": command.final_time_from_start_s,
        "post_command_hold_before_next_command_s": max(0.0, final.stamp_s - command_end_s),
        "samples": len(window),
        "max_abs_position_error_rad": max(max_errors, default=0.0),
        "p95_max_abs_position_error_rad": _percentile(max_errors, 95.0),
        "mean_rms_position_error_rad": mean(rms_errors) if rms_errors else 0.0,
        "worst_joint_by_p95_error": {
            "joint": worst_p95[0],
            "p95_abs_rad": worst_p95[1],
        },
        "worst_joint_by_final_error": {
            "joint": worst_final[0],
            "final_abs_rad": worst_final[1],
        },
        "command_target_xyz_m": target_pos.tolist(),
        "final_feedback_xyz_m": [float(value) for value in feedback_pos],
        "final_cartesian_error_xyz_m": [float(value) for value in cart_error],
        "final_cartesian_error_norm_m": float(np.linalg.norm(cart_error)),
        "final_xy_error_m": float(np.linalg.norm(feedback_pos[:2] - AXIS_ALIGN_POSE[:2])),
        "xy_error_min_m": min(xy_errors, default=0.0),
        "xy_error_mean_m": mean(xy_errors) if xy_errors else 0.0,
        "xy_error_p95_m": _percentile(xy_errors, 95.0),
        "xy_error_max_m": max(xy_errors, default=0.0),
        "strict_xy_m": STRICT_XY_M,
        "strict_xy_sample_count": sum(1 for value in xy_errors if value <= STRICT_XY_M),
        "strict_xy_sample_fraction": (
            sum(1 for value in xy_errors if value <= STRICT_XY_M) / len(xy_errors)
            if xy_errors
            else 0.0
        ),
        "final_xy_window_s": FINAL_XY_WINDOW_S,
        "final_xy_window_min_m": min(final_xy_errors, default=0.0),
        "final_xy_window_mean_m": mean(final_xy_errors) if final_xy_errors else 0.0,
        "final_xy_window_max_m": max(final_xy_errors, default=0.0),
        "joint_errors": joint_rows,
    }


def _fmt(value: object, digits: int = 6) -> str:
    if value is None:
        return "none"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    if isinstance(value, list):
        return ", ".join(_fmt(item, digits) for item in value)
    return str(value)


def write_outputs(input_dir: Path, result: dict[str, object]) -> None:
    json_path = input_dir / "moving_to_start_tracking_analysis.json"
    md_path = input_dir / "moving_to_start_tracking_analysis.md"
    json_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# MOVING_TO_START Tracking Analysis",
        "",
        f"- input_dir: `{result['input_dir']}`",
        f"- result: `{result.get('result')}`",
    ]
    if result.get("result") != "OK":
        lines.extend(
            [
                f"- commands: `{result.get('commands', 0)}`",
                f"- command_target_tolerance_m: `{_fmt(result.get('command_target_tolerance_m'))}`",
                "",
                "Interpretation: no captured trajectory command had a peg-tip target "
                "near the canonical axis-align pose, so this diagnostic cannot "
                "attribute MOVING_TO_START joint tracking for this run.",
            ]
        )
        md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return

    worst_p95 = result["worst_joint_by_p95_error"]
    worst_final = result["worst_joint_by_final_error"]
    assert isinstance(worst_p95, dict)
    assert isinstance(worst_final, dict)
    lines.extend(
        [
            f"- command_index: `{result.get('command_index')}`",
            f"- tracking_source: `{result.get('tracking_source')}`",
            f"- command_duration_s: `{_fmt(result.get('command_duration_s'), 3)}`",
            f"- post_command_hold_before_next_command_s: `{_fmt(result.get('post_command_hold_before_next_command_s'), 3)}`",
            f"- samples: `{result.get('samples')}`",
            f"- p95_max_abs_position_error_rad: `{_fmt(result.get('p95_max_abs_position_error_rad'))}`",
            f"- worst_joint_by_p95_error: `{worst_p95.get('joint')}` `{_fmt(worst_p95.get('p95_abs_rad'))}` rad",
            f"- worst_joint_by_final_error: `{worst_final.get('joint')}` `{_fmt(worst_final.get('final_abs_rad'))}` rad",
            f"- command_target_xyz_m: `{_fmt(result.get('command_target_xyz_m'))}`",
            f"- final_feedback_xyz_m: `{_fmt(result.get('final_feedback_xyz_m'))}`",
            f"- final_cartesian_error_xyz_m: `{_fmt(result.get('final_cartesian_error_xyz_m'))}`",
            f"- final_cartesian_error_norm_m: `{_fmt(result.get('final_cartesian_error_norm_m'))}`",
            f"- final_xy_error_m: `{_fmt(result.get('final_xy_error_m'))}`",
            f"- xy_error_min_m: `{_fmt(result.get('xy_error_min_m'))}`",
            f"- xy_error_mean_m: `{_fmt(result.get('xy_error_mean_m'))}`",
            f"- xy_error_p95_m: `{_fmt(result.get('xy_error_p95_m'))}`",
            f"- xy_error_max_m: `{_fmt(result.get('xy_error_max_m'))}`",
            f"- strict_xy_sample_count: `{result.get('strict_xy_sample_count')}`",
            f"- strict_xy_sample_fraction: `{_fmt(result.get('strict_xy_sample_fraction'))}`",
            f"- final_xy_window_s: `{_fmt(result.get('final_xy_window_s'), 3)}`",
            f"- final_xy_window_min_m: `{_fmt(result.get('final_xy_window_min_m'))}`",
            f"- final_xy_window_mean_m: `{_fmt(result.get('final_xy_window_mean_m'))}`",
            f"- final_xy_window_max_m: `{_fmt(result.get('final_xy_window_max_m'))}`",
            "",
            "## Per Joint Error",
            "",
            "| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    joint_errors = result["joint_errors"]
    assert isinstance(joint_errors, list)
    for row in joint_errors:
        assert isinstance(row, dict)
        lines.append(
            "| "
            + " | ".join(
                [
                    str(row["joint"]),
                    _fmt(row["max_abs_rad"]),
                    _fmt(row["p95_abs_rad"]),
                    _fmt(row["mean_abs_rad"]),
                    _fmt(row["final_error_rad"]),
                    _fmt(row["target_minus_feedback_rad"]),
                    _fmt(row["target_rad"]),
                    _fmt(row["final_feedback_rad"]),
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "Interpretation: this is an offline diagnostic over passive trajectory "
            "observer CSVs. It selects the MOVING_TO_START command by peg-tip "
            "target pose, not by command index, and does not alter controller "
            "behavior or safety gates.",
        ]
    )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path)
    args = parser.parse_args()
    result = analyze(args.input_dir)
    write_outputs(args.input_dir, result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
