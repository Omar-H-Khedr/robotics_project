#!/usr/bin/env python3
"""Analyze peg-tip XY drift during the INSERT command window."""

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


FINAL_INSERTION_POSE = np.array([0.520, -0.200, 0.790])
HOLE_CENTRE_XY = np.array([0.520, -0.200])
HOLE_TOP_Z = 0.810
PHYSICAL_CLEARANCE_M = 0.001
MEANINGFUL_DEPTH_M = 0.001
COMMAND_TARGET_TOLERANCE_M = 0.035
PRE_COMMAND_WINDOW_S = 1.0
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


@dataclass(frozen=True)
class CartesianSample:
    stamp_s: float
    elapsed_s: float
    xy_error_m: float
    depth_m: float
    z_m: float
    max_abs_joint_error_rad: float
    rms_joint_error_rad: float
    worst_joint_name: str
    worst_joint_error_rad: float
    feedback_xyz_m: list[float]


def _float_list(text: str) -> list[float]:
    return [float(value) for value in text.split() if value.strip()]


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(
        len(ordered) - 1,
        max(0, int(round((percentile / 100.0) * (len(ordered) - 1)))),
    )
    return ordered[index]


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
    rows: list[TrackingSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            reference = [float(row[f"{name}_reference_rad"]) for name in joint_names]
            feedback = [float(row[f"{name}_feedback_rad"]) for name in joint_names]
            error = [float(row[f"{name}_error_rad"]) for name in joint_names]
            rows.append(
                TrackingSample(
                    stamp_s=float(row["stamp_s"]),
                    max_abs_error_rad=float(row["max_abs_position_error_rad"]),
                    rms_error_rad=float(row["rms_position_error_rad"]),
                    reference=reference,
                    feedback=feedback,
                    error=error,
                )
            )
    return rows


def _select_insert_command(commands: list[CommandRow]) -> int | None:
    best_index: int | None = None
    best_error = math.inf
    for index, command in enumerate(commands):
        error = float(np.linalg.norm(np.array(command.target_xyz_m) - FINAL_INSERTION_POSE))
        if error < best_error:
            best_index = index
            best_error = error
    if best_index is None or best_error > COMMAND_TARGET_TOLERANCE_M:
        return None
    return best_index


def _nearest_state_value(path: Path, stamp_s: float, column: str, default: float = 0.0) -> float:
    if not path.exists():
        return default
    nearest_distance = math.inf
    nearest_value = default
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            distance = abs(float(row["stamp_s"]) - stamp_s)
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_value = float(row[column])
    return nearest_value


def _first(samples: list[CartesianSample], predicate) -> CartesianSample | None:
    for sample in samples:
        if predicate(sample):
            return sample
    return None


def _as_summary(sample: CartesianSample | None) -> dict[str, object] | None:
    return sample.__dict__ if sample is not None else None


def _format_float(value: float | None, digits: int = 6) -> str:
    if value is None:
        return "none"
    return f"{value:.{digits}f}"


def _format_vec(values: list[float] | np.ndarray, digits: int = 6) -> str:
    return ", ".join(f"{float(value):.{digits}f}" for value in values)


def analyze_directory(
    input_dir: Path,
    output_name: str,
    json_name: str,
    command_index: int,
) -> Path:
    commands_path = input_dir / "trajectory_commands.csv"
    controller_tracking_path = input_dir / CONTROLLER_STATE_TRACKING_FILE
    joint_tracking_path = input_dir / JOINT_STATE_TRACKING_FILE
    tracking_path = controller_tracking_path if controller_tracking_path.exists() else joint_tracking_path

    if not commands_path.exists():
        raise FileNotFoundError(f"missing {commands_path}")
    if not tracking_path.exists():
        raise FileNotFoundError(f"missing {tracking_path}")

    commands = _read_commands(commands_path)
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
    tracking_samples = _read_tracking(tracking_path, insert_command.joint_names)
    insert_tracking = [
        sample
        for sample in tracking_samples
        if sample.stamp_s >= insert_command.receipt_stamp_s
        and (next_stamp is None or sample.stamp_s < next_stamp)
    ]
    pre_command_tracking = [
        sample
        for sample in tracking_samples
        if insert_command.receipt_stamp_s - PRE_COMMAND_WINDOW_S
        <= sample.stamp_s
        < insert_command.receipt_stamp_s
    ]
    if not insert_tracking:
        raise RuntimeError("no tracking samples in selected insert command window")

    kin = RobotKinematics()
    def to_cartesian(sample: TrackingSample) -> CartesianSample:
        feedback_xyz, _ = kin.pose(np.array(sample.feedback, dtype=float))
        abs_joint_errors = [abs(value) for value in sample.error]
        worst_index = max(range(len(abs_joint_errors)), key=lambda index: abs_joint_errors[index])
        return CartesianSample(
            stamp_s=sample.stamp_s,
            elapsed_s=sample.stamp_s - insert_command.receipt_stamp_s,
            xy_error_m=float(np.linalg.norm(feedback_xyz[:2] - HOLE_CENTRE_XY)),
            depth_m=max(0.0, HOLE_TOP_Z - float(feedback_xyz[2])),
            z_m=float(feedback_xyz[2]),
            max_abs_joint_error_rad=sample.max_abs_error_rad,
            rms_joint_error_rad=sample.rms_error_rad,
            worst_joint_name=insert_command.joint_names[worst_index],
            worst_joint_error_rad=sample.error[worst_index],
            feedback_xyz_m=[float(value) for value in feedback_xyz],
        )

    cartesian_samples = [to_cartesian(sample) for sample in insert_tracking]
    pre_command_samples = [to_cartesian(sample) for sample in pre_command_tracking]
    first_clearance_violation = _first(
        cartesian_samples,
        lambda sample: sample.xy_error_m > PHYSICAL_CLEARANCE_M,
    )
    first_depth = _first(
        cartesian_samples,
        lambda sample: sample.depth_m >= MEANINGFUL_DEPTH_M,
    )
    first_sideload = _first(
        cartesian_samples,
        lambda sample: sample.depth_m >= MEANINGFUL_DEPTH_M
        and sample.xy_error_m > PHYSICAL_CLEARANCE_M,
    )
    max_xy_sample = max(cartesian_samples, key=lambda sample: sample.xy_error_m)
    max_depth_sample = max(cartesian_samples, key=lambda sample: sample.depth_m)
    final_sample = cartesian_samples[-1]
    pre_final_sample = pre_command_samples[-1] if pre_command_samples else None
    pre_max_xy_sample = max(pre_command_samples, key=lambda sample: sample.xy_error_m) if pre_command_samples else None

    xy_errors = [sample.xy_error_m for sample in cartesian_samples]
    depths = [sample.depth_m for sample in cartesian_samples]
    joint_errors = [sample.max_abs_joint_error_rad for sample in cartesian_samples]
    target_xyz, _ = kin.pose(np.array(insert_command.final_positions_rad, dtype=float))
    nearest_fz_at_violation = (
        _nearest_state_value(input_dir / WRENCH_FILE, first_clearance_violation.stamp_s, "fz_n")
        if first_clearance_violation is not None else None
    )
    nearest_contact_at_violation = (
        _nearest_state_value(
            input_dir / CONTACT_FILE,
            first_clearance_violation.stamp_s,
            "max_contact_force_n",
        )
        if first_clearance_violation is not None else None
    )

    result = {
        "input_dir": str(input_dir),
        "tracking_source": tracking_path.name,
        "command_index": command_index,
        "command_point_count": insert_command.point_count,
        "command_receipt_stamp_s": insert_command.receipt_stamp_s,
        "next_command_stamp_s": next_stamp,
        "command_duration_s": insert_command.final_time_from_start_s,
        "observed_window_s": max(0.0, (next_stamp if next_stamp is not None else final_sample.stamp_s) - insert_command.receipt_stamp_s),
        "samples": len(cartesian_samples),
        "hole_centre_xy_m": [float(value) for value in HOLE_CENTRE_XY],
        "hole_top_z_m": HOLE_TOP_Z,
        "physical_clearance_m": PHYSICAL_CLEARANCE_M,
        "meaningful_depth_m": MEANINGFUL_DEPTH_M,
        "pre_command_window_s": PRE_COMMAND_WINDOW_S,
        "pre_command_samples": len(pre_command_samples),
        "target_xyz_m": [float(value) for value in target_xyz],
        "pre_command_final_sample": _as_summary(pre_final_sample),
        "pre_command_max_xy_sample": _as_summary(pre_max_xy_sample),
        "initial_xy_error_m": cartesian_samples[0].xy_error_m,
        "final_xy_error_m": final_sample.xy_error_m,
        "max_xy_error_m": max_xy_sample.xy_error_m,
        "mean_xy_error_m": mean(xy_errors),
        "p95_xy_error_m": _percentile(xy_errors, 95.0),
        "final_depth_m": final_sample.depth_m,
        "max_depth_m": max_depth_sample.depth_m,
        "mean_depth_m": mean(depths),
        "max_abs_joint_error_rad": max(joint_errors),
        "p95_max_abs_joint_error_rad": _percentile(joint_errors, 95.0),
        "first_clearance_violation": _as_summary(first_clearance_violation),
        "first_depth": _as_summary(first_depth),
        "first_sideload": _as_summary(first_sideload),
        "nearest_fz_at_first_clearance_violation_n": nearest_fz_at_violation,
        "nearest_contact_at_first_clearance_violation_n": nearest_contact_at_violation,
    }

    json_path = input_dir / json_name
    json_path.write_text(
        json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    lines = [
        "# Insert XY Drift Analysis",
        "",
        f"- input_dir: `{input_dir}`",
        f"- tracking_source: `{tracking_path.name}`",
        f"- command_index: `{command_index}`",
        f"- command_point_count: `{insert_command.point_count}`",
        f"- command_receipt_stamp_s: `{insert_command.receipt_stamp_s:.3f}`",
        f"- next_command_stamp_s: `{next_stamp:.3f}`" if next_stamp is not None else "- next_command_stamp_s: `none`",
        f"- observed_window_s: `{result['observed_window_s']:.3f}`",
        f"- samples: `{len(cartesian_samples)}`",
        f"- hole_centre_xy_m: `{_format_vec(HOLE_CENTRE_XY)}`",
        f"- target_xyz_m: `{_format_vec(target_xyz)}`",
        f"- physical_clearance_m: `{PHYSICAL_CLEARANCE_M:.6f}`",
        f"- meaningful_depth_m: `{MEANINGFUL_DEPTH_M:.6f}`",
        f"- pre_command_window_s: `{PRE_COMMAND_WINDOW_S:.3f}`",
        f"- pre_command_samples: `{len(pre_command_samples)}`",
        "",
        "## Pre-Command Boundary",
        "",
        "- pre_command_final_xy_error_m: "
        f"`{_format_float(pre_final_sample.xy_error_m if pre_final_sample else None)}`",
        "- pre_command_final_depth_m: "
        f"`{_format_float(pre_final_sample.depth_m if pre_final_sample else None)}`",
        "- pre_command_max_xy_error_m: "
        f"`{_format_float(pre_max_xy_sample.xy_error_m if pre_max_xy_sample else None)}`",
        "",
        "## Drift Summary",
        "",
        f"- initial_xy_error_m: `{result['initial_xy_error_m']:.6f}`",
        f"- final_xy_error_m: `{result['final_xy_error_m']:.6f}`",
        f"- max_xy_error_m: `{result['max_xy_error_m']:.6f}`",
        f"- p95_xy_error_m: `{result['p95_xy_error_m']:.6f}`",
        f"- final_depth_m: `{result['final_depth_m']:.6f}`",
        f"- max_depth_m: `{result['max_depth_m']:.6f}`",
        f"- max_abs_joint_error_rad: `{result['max_abs_joint_error_rad']:.6f}`",
        f"- p95_max_abs_joint_error_rad: `{result['p95_max_abs_joint_error_rad']:.6f}`",
        "",
        "## First Events",
        "",
    ]
    event_rows = [
        ("clearance_violation", first_clearance_violation),
        ("meaningful_depth", first_depth),
        ("sideload", first_sideload),
    ]
    lines.extend([
        "| event | elapsed_s | stamp_s | xy_error_m | depth_m | z_m | worst_joint | worst_joint_error_rad |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |",
    ])
    for name, sample in event_rows:
        if sample is None:
            lines.append(f"| {name} | none | none | none | none | none | none | none |")
        else:
            lines.append(
                f"| {name} | {sample.elapsed_s:.3f} | {sample.stamp_s:.3f} | "
                f"{sample.xy_error_m:.6f} | {sample.depth_m:.6f} | {sample.z_m:.6f} | "
                f"{sample.worst_joint_name} | {sample.worst_joint_error_rad:.6f} |"
            )
    lines.extend([
        "",
        "## Correlated Sensor Values",
        "",
        "- nearest_fz_at_first_clearance_violation_n: "
        f"`{_format_float(nearest_fz_at_violation)}`",
        "- nearest_contact_at_first_clearance_violation_n: "
        f"`{_format_float(nearest_contact_at_violation)}`",
        "",
        "Interpretation: this is an offline diagnostic over passive observer CSVs. "
        "It does not publish commands or alter safety gates.",
    ])
    output_path = input_dir / output_name
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="Diagnostics directory containing trajectory CSVs")
    parser.add_argument("--output-name", default="insert_xy_drift_analysis.md")
    parser.add_argument("--json-name", default="insert_xy_drift_analysis.json")
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
