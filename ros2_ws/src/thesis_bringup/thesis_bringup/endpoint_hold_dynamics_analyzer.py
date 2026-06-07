#!/usr/bin/env python3
"""Analyze above-hole endpoint hold dynamics after the axis-align command."""

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
TRIAL_OUTCOME_FILE = "trial_outcome.json"
DEFAULT_STATE_LOOP_HZ = 10.0
CONTROLLER_STATE_TRACKING_FILE = "trajectory_controller_state_samples.csv"
JOINT_STATE_TRACKING_FILE = "trajectory_tracking_samples.csv"


@dataclass(frozen=True)
class CommandRow:
    receipt_stamp_s: float
    joint_names: list[str]
    final_time_from_start_s: float
    final_positions_rad: list[float]
    target_xyz_m: list[float]


@dataclass(frozen=True)
class TrackingSample:
    stamp_s: float
    max_abs_error_rad: float
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
                    feedback=feedback,
                    error=error,
                )
            )
    return samples


def _select_axis_align_command(commands: list[CommandRow]) -> int | None:
    best_index: int | None = None
    best_error = math.inf
    for index, command in enumerate(commands):
        error = float(np.linalg.norm(np.array(command.target_xyz_m) - AXIS_ALIGN_POSE))
        if error < best_error:
            best_error = error
            best_index = index
    if best_index is None or best_error > COMMAND_TARGET_TOLERANCE_M:
        return None
    return best_index


def _infer_state_loop_hz(input_dir: Path, override_hz: float | None = None) -> float:
    if override_hz is not None and override_hz > 0.0:
        return override_hz
    outcome_path = input_dir / TRIAL_OUTCOME_FILE
    if outcome_path.exists():
        try:
            outcome = json.loads(outcome_path.read_text(encoding="utf-8"))
            metrics = outcome.get("metrics", {})
            if isinstance(metrics, dict):
                control_rate_hz = float(metrics.get("control_rate_hz", 0.0))
                if control_rate_hz > 0.0:
                    return control_rate_hz
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            pass
    return DEFAULT_STATE_LOOP_HZ


def _strict_bins(
    samples: list[TrackingSample],
    xy_errors: list[float],
    state_loop_hz: float,
) -> dict[str, object]:
    bins: dict[int, list[float]] = {}
    if not samples:
        return {
            "strict_bin_count": 0,
            "max_consecutive_strict_bins": 0,
            "estimated_state_loop_hz": state_loop_hz,
        }
    start_s = samples[0].stamp_s
    for sample, xy_error in zip(samples, xy_errors):
        index = int(math.floor((sample.stamp_s - start_s) * state_loop_hz))
        bins.setdefault(index, []).append(xy_error)
    strict_indexes = sorted(
        index for index, values in bins.items() if values and max(values) <= STRICT_XY_M
    )
    best = 0
    current = 0
    previous: int | None = None
    for index in strict_indexes:
        current = current + 1 if previous is not None and index == previous + 1 else 1
        best = max(best, current)
        previous = index
    return {
        "strict_bin_count": len(strict_indexes),
        "max_consecutive_strict_bins": best,
        "estimated_state_loop_hz": state_loop_hz,
    }


def analyze(input_dir: Path, state_loop_hz: float | None = None) -> dict[str, object]:
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

    inferred_state_loop_hz = _infer_state_loop_hz(input_dir, state_loop_hz)
    commands = _read_commands(commands_path)
    command_index = _select_axis_align_command(commands)
    if command_index is None:
        return {
            "input_dir": str(input_dir),
            "result": "NO_AXIS_ALIGN_COMMAND_CAPTURED",
            "commands": len(commands),
        }

    command = commands[command_index]
    command_end_s = command.receipt_stamp_s + command.final_time_from_start_s
    next_stamp = (
        commands[command_index + 1].receipt_stamp_s
        if command_index + 1 < len(commands)
        else None
    )
    samples = _read_tracking(tracking_path, command.joint_names)
    hold = [
        sample
        for sample in samples
        if sample.stamp_s >= command_end_s
        and (next_stamp is None or sample.stamp_s < next_stamp)
    ]
    if not hold:
        return {
            "input_dir": str(input_dir),
            "result": "NO_ENDPOINT_HOLD_SAMPLES",
            "tracking_source": tracking_path.name,
            "command_index": command_index,
            "command_end_s": command_end_s,
            "next_command_stamp_s": next_stamp,
        }

    kin = RobotKinematics()
    target_q = np.array(command.final_positions_rad, dtype=float)
    target_pos = np.array(command.target_xyz_m, dtype=float)
    xy_errors: list[float] = []
    cart_errors: list[float] = []
    positions: list[np.ndarray] = []
    for sample in hold:
        pos, _ = kin.pose(np.array(sample.feedback, dtype=float))
        positions.append(pos)
        xy_errors.append(float(np.linalg.norm(pos[:2] - AXIS_ALIGN_POSE[:2])))
        cart_errors.append(float(np.linalg.norm(target_pos - pos)))

    joint_rows: list[dict[str, object]] = []
    largest_range = ("", 0.0)
    for index, name in enumerate(command.joint_names):
        feedback_values = [sample.feedback[index] for sample in hold]
        error_values = [sample.error[index] for sample in hold]
        feedback_range = max(feedback_values) - min(feedback_values)
        if feedback_range > largest_range[1]:
            largest_range = (name, feedback_range)
        joint_rows.append(
            {
                "joint": name,
                "target_rad": float(target_q[index]),
                "feedback_min_rad": min(feedback_values),
                "feedback_max_rad": max(feedback_values),
                "feedback_range_rad": feedback_range,
                "mean_abs_error_rad": mean(abs(value) for value in error_values),
                "p95_abs_error_rad": _percentile([abs(value) for value in error_values], 95.0),
                "final_error_rad": float(error_values[-1]),
            }
        )

    position_array = np.array(positions, dtype=float)
    bins = _strict_bins(hold, xy_errors, inferred_state_loop_hz)
    return {
        "input_dir": str(input_dir),
        "result": "OK",
        "tracking_source": tracking_path.name,
        "command_index": command_index,
        "samples": len(hold),
        "hold_start_s": command_end_s,
        "hold_end_s": hold[-1].stamp_s,
        "hold_duration_s": hold[-1].stamp_s - hold[0].stamp_s,
        "next_command_stamp_s": next_stamp,
        "strict_xy_m": STRICT_XY_M,
        "xy_error_min_m": min(xy_errors),
        "xy_error_mean_m": mean(xy_errors),
        "xy_error_p95_m": _percentile(xy_errors, 95.0),
        "xy_error_max_m": max(xy_errors),
        "cartesian_error_min_m": min(cart_errors),
        "cartesian_error_mean_m": mean(cart_errors),
        "cartesian_error_p95_m": _percentile(cart_errors, 95.0),
        "cartesian_error_max_m": max(cart_errors),
        "x_range_m": float(position_array[:, 0].max() - position_array[:, 0].min()),
        "y_range_m": float(position_array[:, 1].max() - position_array[:, 1].min()),
        "z_range_m": float(position_array[:, 2].max() - position_array[:, 2].min()),
        "strict_xy_sample_count": sum(1 for value in xy_errors if value <= STRICT_XY_M),
        "strict_xy_sample_fraction": (
            sum(1 for value in xy_errors if value <= STRICT_XY_M) / len(xy_errors)
        ),
        "strict_bin_count": bins["strict_bin_count"],
        "max_consecutive_strict_bins": bins["max_consecutive_strict_bins"],
        "estimated_state_loop_hz": bins["estimated_state_loop_hz"],
        "largest_feedback_range_joint": {
            "joint": largest_range[0],
            "feedback_range_rad": largest_range[1],
        },
        "joint_hold_dynamics": joint_rows,
    }


def _fmt(value: object, digits: int = 6) -> str:
    if value is None:
        return "none"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def write_outputs(input_dir: Path, result: dict[str, object]) -> None:
    json_path = input_dir / "endpoint_hold_dynamics_analysis.json"
    md_path = input_dir / "endpoint_hold_dynamics_analysis.md"
    json_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# Endpoint Hold Dynamics Analysis",
        "",
        f"- input_dir: `{result['input_dir']}`",
        f"- result: `{result.get('result')}`",
    ]
    if result.get("result") != "OK":
        md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return

    largest = result["largest_feedback_range_joint"]
    assert isinstance(largest, dict)
    lines.extend(
        [
            f"- tracking_source: `{result.get('tracking_source')}`",
            f"- command_index: `{result.get('command_index')}`",
            f"- samples: `{result.get('samples')}`",
            f"- hold_duration_s: `{_fmt(result.get('hold_duration_s'), 3)}`",
            f"- xy_error_min_m: `{_fmt(result.get('xy_error_min_m'))}`",
            f"- xy_error_mean_m: `{_fmt(result.get('xy_error_mean_m'))}`",
            f"- xy_error_p95_m: `{_fmt(result.get('xy_error_p95_m'))}`",
            f"- xy_error_max_m: `{_fmt(result.get('xy_error_max_m'))}`",
            f"- cartesian_error_p95_m: `{_fmt(result.get('cartesian_error_p95_m'))}`",
            f"- x_range_m: `{_fmt(result.get('x_range_m'))}`",
            f"- y_range_m: `{_fmt(result.get('y_range_m'))}`",
            f"- z_range_m: `{_fmt(result.get('z_range_m'))}`",
            f"- strict_xy_sample_count: `{result.get('strict_xy_sample_count')}`",
            f"- strict_xy_sample_fraction: `{_fmt(result.get('strict_xy_sample_fraction'))}`",
            f"- strict_bin_count: `{result.get('strict_bin_count')}`",
            f"- max_consecutive_strict_bins: `{result.get('max_consecutive_strict_bins')}`",
            f"- estimated_state_loop_hz: `{_fmt(result.get('estimated_state_loop_hz'), 1)}`",
            f"- largest_feedback_range_joint: `{largest.get('joint')}` `{_fmt(largest.get('feedback_range_rad'))}` rad",
            "",
            "## Per Joint Hold Dynamics",
            "",
            "| joint | target_rad | feedback_min_rad | feedback_max_rad | feedback_range_rad | mean_abs_error_rad | p95_abs_error_rad | final_error_rad |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    joint_rows = result["joint_hold_dynamics"]
    assert isinstance(joint_rows, list)
    for row in joint_rows:
        assert isinstance(row, dict)
        lines.append(
            "| "
            + " | ".join(
                [
                    str(row["joint"]),
                    _fmt(row["target_rad"]),
                    _fmt(row["feedback_min_rad"]),
                    _fmt(row["feedback_max_rad"]),
                    _fmt(row["feedback_range_rad"]),
                    _fmt(row["mean_abs_error_rad"]),
                    _fmt(row["p95_abs_error_rad"]),
                    _fmt(row["final_error_rad"]),
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "Interpretation: this diagnostic uses passive trajectory observer data "
            "after the axis-align command duration has elapsed and before the next "
            "trajectory command. It does not alter controller behavior or safety gates.",
        ]
    )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path)
    parser.add_argument(
        "--state-loop-hz",
        type=float,
        default=None,
        help=(
            "Task state-machine cadence in Hz. Defaults to metrics.control_rate_hz "
            "from trial_outcome.json in input_dir, falling back to 10 Hz."
        ),
    )
    args = parser.parse_args()
    result = analyze(args.input_dir, args.state_loop_hz)
    write_outputs(args.input_dir, result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
