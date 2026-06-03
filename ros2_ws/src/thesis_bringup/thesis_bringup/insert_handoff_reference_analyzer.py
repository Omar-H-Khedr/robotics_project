#!/usr/bin/env python3
"""Compare Cartesian JTC reference and feedback at the INSERT handoff."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import mean

import numpy as np

from kuka_task_control.robot_kinematics import RobotKinematics


FINAL_INSERTION_POSE = np.array([0.520, -0.200, 0.790])
HOLE_CENTRE_XY = np.array([0.520, -0.200])
HOLE_TOP_Z = 0.810
PHYSICAL_CLEARANCE_M = 0.001
COMMAND_TARGET_TOLERANCE_M = 0.035
PRE_COMMAND_WINDOW_S = 1.0
POST_COMMAND_WINDOW_S = 0.5
CONTROLLER_STATE_TRACKING_FILE = "trajectory_controller_state_samples.csv"


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
class CartesianPairSample:
    stamp_s: float
    elapsed_s: float
    reference_xy_error_m: float
    feedback_xy_error_m: float
    reference_z_m: float
    feedback_z_m: float
    reference_depth_m: float
    feedback_depth_m: float
    cartesian_reference_feedback_error_m: float
    max_abs_joint_error_rad: float
    rms_joint_error_rad: float
    worst_joint_name: str
    worst_joint_error_rad: float
    reference_xyz_m: list[float]
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


def _first(samples: list[CartesianPairSample], predicate) -> CartesianPairSample | None:
    for sample in samples:
        if predicate(sample):
            return sample
    return None


def _sample_dict(sample: CartesianPairSample | None) -> dict[str, object] | None:
    return asdict(sample) if sample is not None else None


def _fmt(value: float | None, digits: int = 6) -> str:
    if value is None:
        return "none"
    return f"{value:.{digits}f}"


def _fmt_vec(values: list[float] | np.ndarray, digits: int = 6) -> str:
    return ", ".join(f"{float(value):.{digits}f}" for value in values)


def _to_cartesian(sample: TrackingSample, joint_names: list[str], kin: RobotKinematics, receipt: float) -> CartesianPairSample:
    reference_xyz, _ = kin.pose(np.array(sample.reference, dtype=float))
    feedback_xyz, _ = kin.pose(np.array(sample.feedback, dtype=float))
    abs_joint_errors = [abs(value) for value in sample.error]
    worst_index = max(range(len(abs_joint_errors)), key=lambda index: abs_joint_errors[index])
    return CartesianPairSample(
        stamp_s=sample.stamp_s,
        elapsed_s=sample.stamp_s - receipt,
        reference_xy_error_m=float(np.linalg.norm(reference_xyz[:2] - HOLE_CENTRE_XY)),
        feedback_xy_error_m=float(np.linalg.norm(feedback_xyz[:2] - HOLE_CENTRE_XY)),
        reference_z_m=float(reference_xyz[2]),
        feedback_z_m=float(feedback_xyz[2]),
        reference_depth_m=max(0.0, HOLE_TOP_Z - float(reference_xyz[2])),
        feedback_depth_m=max(0.0, HOLE_TOP_Z - float(feedback_xyz[2])),
        cartesian_reference_feedback_error_m=float(np.linalg.norm(reference_xyz - feedback_xyz)),
        max_abs_joint_error_rad=sample.max_abs_error_rad,
        rms_joint_error_rad=sample.rms_error_rad,
        worst_joint_name=joint_names[worst_index],
        worst_joint_error_rad=sample.error[worst_index],
        reference_xyz_m=[float(value) for value in reference_xyz],
        feedback_xyz_m=[float(value) for value in feedback_xyz],
    )


def analyze_directory(
    input_dir: Path,
    output_name: str,
    json_name: str,
    command_index: int,
) -> Path:
    commands_path = input_dir / "trajectory_commands.csv"
    tracking_path = input_dir / CONTROLLER_STATE_TRACKING_FILE
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
    next_stamp = commands[command_index + 1].receipt_stamp_s if command_index + 1 < len(commands) else None
    tracking_samples = _read_tracking(tracking_path, insert_command.joint_names)
    pre_samples = [
        sample
        for sample in tracking_samples
        if insert_command.receipt_stamp_s - PRE_COMMAND_WINDOW_S <= sample.stamp_s < insert_command.receipt_stamp_s
    ]
    insert_samples = [
        sample
        for sample in tracking_samples
        if sample.stamp_s >= insert_command.receipt_stamp_s
        and (next_stamp is None or sample.stamp_s < next_stamp)
        and sample.stamp_s <= insert_command.receipt_stamp_s + POST_COMMAND_WINDOW_S
    ]
    if not insert_samples:
        raise RuntimeError("no controller-state samples found in INSERT handoff window")

    kin = RobotKinematics()
    pre = [_to_cartesian(sample, insert_command.joint_names, kin, insert_command.receipt_stamp_s) for sample in pre_samples]
    insert = [_to_cartesian(sample, insert_command.joint_names, kin, insert_command.receipt_stamp_s) for sample in insert_samples]

    first_reference_clearance = _first(insert, lambda sample: sample.reference_xy_error_m > PHYSICAL_CLEARANCE_M)
    first_feedback_clearance = _first(insert, lambda sample: sample.feedback_xy_error_m > PHYSICAL_CLEARANCE_M)
    max_reference_xy = max(insert, key=lambda sample: sample.reference_xy_error_m)
    max_feedback_xy = max(insert, key=lambda sample: sample.feedback_xy_error_m)
    max_cartesian_error = max(insert, key=lambda sample: sample.cartesian_reference_feedback_error_m)
    final_sample = insert[-1]
    pre_final = pre[-1] if pre else None

    reference_xy_errors = [sample.reference_xy_error_m for sample in insert]
    feedback_xy_errors = [sample.feedback_xy_error_m for sample in insert]
    cartesian_errors = [sample.cartesian_reference_feedback_error_m for sample in insert]
    joint_errors = [sample.max_abs_joint_error_rad for sample in insert]

    result = {
        "input_dir": str(input_dir),
        "tracking_source": tracking_path.name,
        "command_index": command_index,
        "command_point_count": insert_command.point_count,
        "command_receipt_stamp_s": insert_command.receipt_stamp_s,
        "next_command_stamp_s": next_stamp,
        "command_duration_s": insert_command.final_time_from_start_s,
        "samples": len(insert),
        "pre_command_samples": len(pre),
        "pre_command_window_s": PRE_COMMAND_WINDOW_S,
        "post_command_window_s": POST_COMMAND_WINDOW_S,
        "physical_clearance_m": PHYSICAL_CLEARANCE_M,
        "hole_centre_xy_m": [float(value) for value in HOLE_CENTRE_XY],
        "hole_top_z_m": HOLE_TOP_Z,
        "command_target_xyz_m": insert_command.target_xyz_m,
        "pre_command_final_sample": _sample_dict(pre_final),
        "initial_sample": _sample_dict(insert[0]),
        "final_sample": _sample_dict(final_sample),
        "first_reference_clearance_violation": _sample_dict(first_reference_clearance),
        "first_feedback_clearance_violation": _sample_dict(first_feedback_clearance),
        "max_reference_xy_sample": _sample_dict(max_reference_xy),
        "max_feedback_xy_sample": _sample_dict(max_feedback_xy),
        "max_cartesian_reference_feedback_error_sample": _sample_dict(max_cartesian_error),
        "mean_reference_xy_error_m": mean(reference_xy_errors),
        "p95_reference_xy_error_m": _percentile(reference_xy_errors, 95.0),
        "mean_feedback_xy_error_m": mean(feedback_xy_errors),
        "p95_feedback_xy_error_m": _percentile(feedback_xy_errors, 95.0),
        "mean_cartesian_reference_feedback_error_m": mean(cartesian_errors),
        "p95_cartesian_reference_feedback_error_m": _percentile(cartesian_errors, 95.0),
        "max_abs_joint_error_rad": max(joint_errors),
        "p95_max_abs_joint_error_rad": _percentile(joint_errors, 95.0),
    }

    json_path = input_dir / json_name
    json_path.write_text(json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    lines = [
        "# Insert Handoff Reference Analysis",
        "",
        f"- input_dir: `{input_dir}`",
        f"- tracking_source: `{tracking_path.name}`",
        f"- command_index: `{command_index}`",
        f"- command_point_count: `{insert_command.point_count}`",
        f"- command_receipt_stamp_s: `{insert_command.receipt_stamp_s:.3f}`",
        f"- command_duration_s: `{insert_command.final_time_from_start_s:.3f}`",
        f"- post_command_window_s: `{POST_COMMAND_WINDOW_S:.3f}`",
        f"- samples: `{len(insert)}`",
        f"- physical_clearance_m: `{PHYSICAL_CLEARANCE_M:.6f}`",
        f"- command_target_xyz_m: `{_fmt_vec(insert_command.target_xyz_m)}`",
        "",
        "## Boundary",
        "",
        "- pre_command_final_reference_xy_error_m: "
        f"`{_fmt(pre_final.reference_xy_error_m if pre_final else None)}`",
        "- pre_command_final_feedback_xy_error_m: "
        f"`{_fmt(pre_final.feedback_xy_error_m if pre_final else None)}`",
        f"- initial_reference_xy_error_m: `{insert[0].reference_xy_error_m:.6f}`",
        f"- initial_feedback_xy_error_m: `{insert[0].feedback_xy_error_m:.6f}`",
        f"- final_reference_xy_error_m: `{final_sample.reference_xy_error_m:.6f}`",
        f"- final_feedback_xy_error_m: `{final_sample.feedback_xy_error_m:.6f}`",
        "",
        "## Reference vs Feedback",
        "",
        f"- max_reference_xy_error_m: `{max_reference_xy.reference_xy_error_m:.6f}`",
        f"- max_feedback_xy_error_m: `{max_feedback_xy.feedback_xy_error_m:.6f}`",
        f"- p95_reference_xy_error_m: `{result['p95_reference_xy_error_m']:.6f}`",
        f"- p95_feedback_xy_error_m: `{result['p95_feedback_xy_error_m']:.6f}`",
        f"- max_cartesian_reference_feedback_error_m: `{max_cartesian_error.cartesian_reference_feedback_error_m:.6f}`",
        f"- p95_cartesian_reference_feedback_error_m: `{result['p95_cartesian_reference_feedback_error_m']:.6f}`",
        f"- max_abs_joint_error_rad: `{result['max_abs_joint_error_rad']:.6f}`",
        f"- p95_max_abs_joint_error_rad: `{result['p95_max_abs_joint_error_rad']:.6f}`",
        "",
        "## First Clearance Violations",
        "",
        "| signal | elapsed_s | stamp_s | xy_error_m | z_m | depth_m | worst_joint | worst_joint_error_rad |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |",
    ]
    for name, sample, xy_attr, z_attr, depth_attr in [
        ("reference", first_reference_clearance, "reference_xy_error_m", "reference_z_m", "reference_depth_m"),
        ("feedback", first_feedback_clearance, "feedback_xy_error_m", "feedback_z_m", "feedback_depth_m"),
    ]:
        if sample is None:
            lines.append(f"| {name} | none | none | none | none | none | none | none |")
        else:
            lines.append(
                f"| {name} | {sample.elapsed_s:.3f} | {sample.stamp_s:.3f} | "
                f"{getattr(sample, xy_attr):.6f} | {getattr(sample, z_attr):.6f} | "
                f"{getattr(sample, depth_attr):.6f} | {sample.worst_joint_name} | "
                f"{sample.worst_joint_error_rad:.6f} |"
            )
    lines.extend(
        [
            "",
            "Interpretation: this offline diagnostic compares the JTC reference and feedback "
            "at the INSERT handoff. It does not publish commands or alter safety gates.",
        ]
    )
    output_path = input_dir / output_name
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="Diagnostics directory containing trajectory CSVs")
    parser.add_argument("--output-name", default="insert_handoff_reference_analysis.md")
    parser.add_argument("--json-name", default="insert_handoff_reference_analysis.json")
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
