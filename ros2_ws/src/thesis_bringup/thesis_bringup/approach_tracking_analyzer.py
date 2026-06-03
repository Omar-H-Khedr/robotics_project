#!/usr/bin/env python3
"""Analyze approach tracking CSVs from the research baseline launch."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

import numpy as np

from kuka_task_control.robot_kinematics import RobotKinematics


TOUCH_POSE = np.array([0.520, -0.200, 0.830])
COMMAND_TARGET_TOLERANCE_M = 0.03
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
    reference: list[float]
    feedback: list[float]
    error: list[float]


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((percentile / 100.0) * (len(ordered) - 1)))))
    return ordered[index]


def _float_list(text: str) -> list[float]:
    return [float(value) for value in text.split() if value.strip()]


def _read_commands(path: Path) -> list[CommandRow]:
    kin = RobotKinematics()
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows: list[CommandRow] = []
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
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        samples: list[TrackingSample] = []
        for row in reader:
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


def _format_vec(values: np.ndarray | list[float], digits: int = 6) -> str:
    return ", ".join(f"{float(value):.{digits}f}" for value in values)


def _write_summary(
    output_path: Path,
    input_dir: Path,
    tracking_source: str,
    command_index: int,
    approach_command: CommandRow,
    next_command_stamp_s: float | None,
    approach_samples: list[TrackingSample],
) -> None:
    kin = RobotKinematics()
    joint_names = approach_command.joint_names
    target_q = np.array(approach_command.final_positions_rad, dtype=float)
    target_pos, _target_rot = kin.pose(target_q)

    final_sample = approach_samples[-1] if approach_samples else None
    if final_sample is None:
        output_path.write_text(
            "# Approach Tracking Analysis\n\n"
            f"- input_dir: `{input_dir}`\n"
            "- result: `NO_SAMPLES`\n",
            encoding="utf-8",
        )
        return

    feedback_q = np.array(final_sample.feedback, dtype=float)
    feedback_pos, _feedback_rot = kin.pose(feedback_q)
    cart_error = target_pos - feedback_pos
    cart_error_norm = float(np.linalg.norm(cart_error))
    missing_descent_m = float(feedback_pos[2] - target_pos[2])

    per_joint_lines: list[str] = []
    worst_joint_by_p95 = ("", 0.0)
    worst_joint_by_final = ("", 0.0)
    for index, name in enumerate(joint_names):
        abs_errors = [abs(sample.error[index]) for sample in approach_samples]
        max_abs = max(abs_errors, default=0.0)
        p95_abs = _percentile(abs_errors, 95.0)
        mean_abs = mean(abs_errors) if abs_errors else 0.0
        final_abs = abs(final_sample.error[index])
        target_error = float(target_q[index] - feedback_q[index])
        if p95_abs > worst_joint_by_p95[1]:
            worst_joint_by_p95 = (name, p95_abs)
        if final_abs > worst_joint_by_final[1]:
            worst_joint_by_final = (name, final_abs)
        per_joint_lines.append(
            "| "
            + " | ".join(
                [
                    name,
                    f"{max_abs:.6f}",
                    f"{p95_abs:.6f}",
                    f"{mean_abs:.6f}",
                    f"{final_sample.error[index]:.6f}",
                    f"{target_error:.6f}",
                    f"{target_q[index]:.6f}",
                    f"{feedback_q[index]:.6f}",
                ]
            )
            + " |"
        )

    start_s = approach_command.receipt_stamp_s
    end_s = next_command_stamp_s if next_command_stamp_s is not None else final_sample.stamp_s
    observed_duration_s = max(0.0, end_s - start_s)
    command_end_s = start_s + approach_command.final_time_from_start_s
    post_command_hold_s = max(0.0, final_sample.stamp_s - command_end_s)
    max_errors = [sample.max_abs_error_rad for sample in approach_samples]
    rms_errors = [sample.rms_error_rad for sample in approach_samples]

    lines = [
        "# Approach Tracking Analysis",
        "",
        f"- input_dir: `{input_dir}`",
        f"- tracking_source: `{tracking_source}`",
        f"- command_index: `{command_index}`",
        f"- command_receipt_stamp_s: `{start_s:.3f}`",
        f"- next_command_stamp_s: `{next_command_stamp_s:.3f}`" if next_command_stamp_s is not None else "- next_command_stamp_s: `none`",
        f"- approach_command_duration_s: `{approach_command.final_time_from_start_s:.3f}`",
        f"- observed_approach_window_s: `{observed_duration_s:.3f}`",
        f"- post_command_hold_before_next_command_s: `{post_command_hold_s:.3f}`",
        f"- samples: `{len(approach_samples)}`",
        f"- max_abs_position_error_rad: `{max(max_errors, default=0.0):.6f}`",
        f"- p95_max_abs_position_error_rad: `{_percentile(max_errors, 95.0):.6f}`",
        f"- mean_rms_position_error_rad: `{mean(rms_errors) if rms_errors else 0.0:.6f}`",
        f"- worst_joint_by_p95_error: `{worst_joint_by_p95[0]}` `{worst_joint_by_p95[1]:.6f}` rad",
        f"- worst_joint_by_final_error: `{worst_joint_by_final[0]}` `{worst_joint_by_final[1]:.6f}` rad",
        "",
        "## Cartesian Peg Tip Error",
        "",
        f"- command_target_xyz_m: `{_format_vec(target_pos)}`",
        f"- final_feedback_xyz_m: `{_format_vec(feedback_pos)}`",
        f"- final_cartesian_error_xyz_m: `{_format_vec(cart_error)}`",
        f"- final_cartesian_error_norm_m: `{cart_error_norm:.6f}`",
        f"- missing_descent_m: `{missing_descent_m:.6f}`",
        "",
        "## Per Joint Error",
        "",
        "| joint | max_abs_rad | p95_abs_rad | mean_abs_rad | final_sample_error_rad | target_minus_feedback_rad | target_rad | final_feedback_rad |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        *per_joint_lines,
        "",
        "Interpretation: this is an offline diagnostic over the passive tracking observer CSVs. "
        "It does not alter controller behavior or task safety gates.",
    ]
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _select_approach_command(commands: list[CommandRow]) -> int | None:
    best_index: int | None = None
    best_error = math.inf
    for index, command in enumerate(commands):
        target = np.array(command.target_xyz_m, dtype=float)
        error = float(np.linalg.norm(target - TOUCH_POSE))
        if error < best_error:
            best_error = error
            best_index = index
    if best_index is None or best_error > COMMAND_TARGET_TOLERANCE_M:
        return None
    return best_index


def analyze_directory(input_dir: Path, output_name: str, command_index: int) -> Path:
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
    if not commands:
        raise RuntimeError(f"no commands in {commands_path}")
    if command_index < 0:
        selected = _select_approach_command(commands)
        if selected is None:
            raise RuntimeError(
                "no command target matched the canonical approach/touch pose "
                f"within {COMMAND_TARGET_TOLERANCE_M} m"
            )
        command_index = selected
    if command_index < 0 or command_index >= len(commands):
        raise RuntimeError(
            f"command_index {command_index} out of range for {len(commands)} commands"
        )
    approach_command = commands[command_index]
    next_command_index = command_index + 1
    next_stamp = (
        commands[next_command_index].receipt_stamp_s
        if next_command_index < len(commands)
        else None
    )
    samples = _read_tracking(tracking_path, approach_command.joint_names)
    approach_samples = [
        sample
        for sample in samples
        if sample.stamp_s >= approach_command.receipt_stamp_s
        and (next_stamp is None or sample.stamp_s < next_stamp)
    ]
    output_path = input_dir / output_name
    _write_summary(
        output_path,
        input_dir,
        tracking_path.name,
        command_index,
        approach_command,
        next_stamp,
        approach_samples,
    )
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="Diagnostics directory containing trajectory CSVs")
    parser.add_argument(
        "--output-name",
        default="approach_tracking_analysis.md",
        help="Markdown file to write inside input_dir",
    )
    parser.add_argument(
        "--command-index",
        type=int,
        default=-1,
        help=(
            "Zero-based command index to analyze. The default -1 selects the "
            "command whose FK target matches the canonical approach/touch pose."
        ),
    )
    args = parser.parse_args()
    output_path = analyze_directory(args.input_dir, args.output_name, args.command_index)
    print(output_path)


if __name__ == "__main__":
    main()
