#!/usr/bin/env python3
"""Analyze Cartesian reference/feedback stability for trajectory command windows."""

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


COMMANDS_FILE = "trajectory_commands.csv"
CONTROLLER_STATE_TRACKING_FILE = "trajectory_controller_state_samples.csv"
HOLE_CENTRE_XY = np.array([0.520, -0.200])
HOLE_TOP_Z_M = 0.810
PHYSICAL_CLEARANCE_M = 0.001
STATE_LOOP_HZ = 10.0
HOLD_MIN_DURATION_S = 1.0


@dataclass(frozen=True)
class CommandRow:
    index: int
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


def _float_list(text: str) -> list[float]:
    return [float(value) for value in text.split() if value.strip()]


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((percentile / 100.0) * (len(ordered) - 1)))
    return ordered[min(len(ordered) - 1, max(0, index))]


def _mean(values: list[float]) -> float | None:
    return mean(values) if values else None


def _fmt(value: object, digits: int = 6) -> str:
    if value is None:
        return "none"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def _read_commands(path: Path) -> list[CommandRow]:
    kin = RobotKinematics()
    rows: list[CommandRow] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for index, row in enumerate(csv.DictReader(handle)):
            final_positions = _float_list(row["final_positions_rad"])
            target_xyz, _ = kin.pose(np.array(final_positions, dtype=float))
            rows.append(
                CommandRow(
                    index=index,
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
            try:
                reference = [float(row[f"{name}_reference_rad"]) for name in joint_names]
                feedback = [float(row[f"{name}_feedback_rad"]) for name in joint_names]
                error = [float(row[f"{name}_error_rad"]) for name in joint_names]
            except KeyError:
                continue
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


def _to_cartesian(sample: TrackingSample, command: CommandRow, kin: RobotKinematics) -> CartesianSample:
    reference_xyz, _ = kin.pose(np.array(sample.reference, dtype=float))
    feedback_xyz, _ = kin.pose(np.array(sample.feedback, dtype=float))
    abs_joint_errors = [abs(value) for value in sample.error]
    worst_index = max(range(len(abs_joint_errors)), key=lambda index: abs_joint_errors[index])
    return CartesianSample(
        stamp_s=sample.stamp_s,
        elapsed_s=sample.stamp_s - command.receipt_stamp_s,
        reference_xy_error_m=float(np.linalg.norm(reference_xyz[:2] - HOLE_CENTRE_XY)),
        feedback_xy_error_m=float(np.linalg.norm(feedback_xyz[:2] - HOLE_CENTRE_XY)),
        reference_z_m=float(reference_xyz[2]),
        feedback_z_m=float(feedback_xyz[2]),
        reference_depth_m=max(0.0, HOLE_TOP_Z_M - float(reference_xyz[2])),
        feedback_depth_m=max(0.0, HOLE_TOP_Z_M - float(feedback_xyz[2])),
        cartesian_reference_feedback_error_m=float(np.linalg.norm(reference_xyz - feedback_xyz)),
        max_abs_joint_error_rad=sample.max_abs_error_rad,
        rms_joint_error_rad=sample.rms_error_rad,
        worst_joint_name=command.joint_names[worst_index],
        worst_joint_error_rad=sample.error[worst_index],
    )


def _best_estimated_window(samples: list[CartesianSample], threshold_m: float, attr: str) -> dict[str, object]:
    if not samples:
        return {"ticks": 0, "duration_s": 0.0, "start_s": None, "end_s": None}

    tick_period_s = 1.0 / STATE_LOOP_HZ
    next_tick_s = samples[0].stamp_s
    index = 0
    current_ticks = 0
    current_start_s: float | None = None
    best_ticks = 0
    best_start_s: float | None = None
    best_end_s: float | None = None

    while next_tick_s <= samples[-1].stamp_s and index < len(samples):
        while index + 1 < len(samples) and samples[index].stamp_s < next_tick_s:
            index += 1
        sample = samples[index]
        if float(getattr(sample, attr)) <= threshold_m:
            if current_ticks == 0:
                current_start_s = next_tick_s
            current_ticks += 1
            if current_ticks > best_ticks:
                best_ticks = current_ticks
                best_start_s = current_start_s
                best_end_s = next_tick_s
        else:
            current_ticks = 0
            current_start_s = None
        next_tick_s += tick_period_s

    return {
        "ticks": best_ticks,
        "duration_s": best_ticks * tick_period_s,
        "start_s": best_start_s,
        "end_s": best_end_s,
    }


def _first_violation(samples: list[CartesianSample], attr: str) -> dict[str, object] | None:
    for sample in samples:
        if float(getattr(sample, attr)) > PHYSICAL_CLEARANCE_M:
            return asdict(sample)
    return None


def _summarize_command(command: CommandRow, samples: list[CartesianSample], next_stamp: float | None) -> dict[str, object]:
    ref_xy = [sample.reference_xy_error_m for sample in samples]
    fb_xy = [sample.feedback_xy_error_m for sample in samples]
    cart_errors = [sample.cartesian_reference_feedback_error_m for sample in samples]
    joint_errors = [sample.max_abs_joint_error_rad for sample in samples]
    final_sample = samples[-1] if samples else None
    target_xy_error = float(np.linalg.norm(np.array(command.target_xyz_m[:2]) - HOLE_CENTRE_XY))
    hold_like = command.point_count == 1 and command.final_time_from_start_s >= HOLD_MIN_DURATION_S

    return {
        "command_index": command.index,
        "receipt_stamp_s": command.receipt_stamp_s,
        "next_command_stamp_s": next_stamp,
        "point_count": command.point_count,
        "duration_s": command.final_time_from_start_s,
        "hold_like": hold_like,
        "samples": len(samples),
        "target_xyz_m": command.target_xyz_m,
        "target_xy_error_m": target_xy_error,
        "target_depth_m": max(0.0, HOLE_TOP_Z_M - command.target_xyz_m[2]),
        "mean_reference_xy_error_m": _mean(ref_xy),
        "p95_reference_xy_error_m": _percentile(ref_xy, 95.0),
        "max_reference_xy_error_m": max(ref_xy) if ref_xy else None,
        "final_reference_xy_error_m": final_sample.reference_xy_error_m if final_sample else None,
        "mean_feedback_xy_error_m": _mean(fb_xy),
        "p95_feedback_xy_error_m": _percentile(fb_xy, 95.0),
        "max_feedback_xy_error_m": max(fb_xy) if fb_xy else None,
        "final_feedback_xy_error_m": final_sample.feedback_xy_error_m if final_sample else None,
        "mean_cartesian_reference_feedback_error_m": _mean(cart_errors),
        "p95_cartesian_reference_feedback_error_m": _percentile(cart_errors, 95.0),
        "max_cartesian_reference_feedback_error_m": max(cart_errors) if cart_errors else None,
        "max_abs_joint_error_rad": max(joint_errors) if joint_errors else None,
        "p95_max_abs_joint_error_rad": _percentile(joint_errors, 95.0),
        "reference_best_1mm_window": _best_estimated_window(
            samples, PHYSICAL_CLEARANCE_M, "reference_xy_error_m"
        ),
        "feedback_best_1mm_window": _best_estimated_window(
            samples, PHYSICAL_CLEARANCE_M, "feedback_xy_error_m"
        ),
        "first_reference_clearance_violation": _first_violation(samples, "reference_xy_error_m"),
        "first_feedback_clearance_violation": _first_violation(samples, "feedback_xy_error_m"),
    }


def analyze(input_dir: Path) -> dict[str, object]:
    commands_path = input_dir / COMMANDS_FILE
    tracking_path = input_dir / CONTROLLER_STATE_TRACKING_FILE
    if not commands_path.exists():
        raise FileNotFoundError(f"missing {commands_path}")
    if not tracking_path.exists():
        raise FileNotFoundError(f"missing {tracking_path}")

    commands = _read_commands(commands_path)
    if not commands:
        raise RuntimeError(f"no commands in {commands_path}")
    tracking = _read_tracking(tracking_path, commands[0].joint_names)
    kin = RobotKinematics()
    summaries: list[dict[str, object]] = []
    for index, command in enumerate(commands):
        next_stamp = commands[index + 1].receipt_stamp_s if index + 1 < len(commands) else None
        raw_window = [
            sample
            for sample in tracking
            if sample.stamp_s >= command.receipt_stamp_s
            and (next_stamp is None or sample.stamp_s < next_stamp)
        ]
        cartesian_window = [_to_cartesian(sample, command, kin) for sample in raw_window]
        summaries.append(_summarize_command(command, cartesian_window, next_stamp))

    hold_summaries = [summary for summary in summaries if summary["hold_like"]]
    return {
        "input_dir": str(input_dir),
        "command_source": COMMANDS_FILE,
        "tracking_source": CONTROLLER_STATE_TRACKING_FILE,
        "physical_clearance_m": PHYSICAL_CLEARANCE_M,
        "state_loop_hz": STATE_LOOP_HZ,
        "commands": summaries,
        "hold_like_command_count": len(hold_summaries),
        "hold_like_best_feedback_1mm_ticks": max(
            (
                int(summary["feedback_best_1mm_window"]["ticks"])
                for summary in hold_summaries
                if isinstance(summary.get("feedback_best_1mm_window"), dict)
            ),
            default=0,
        ),
    }


def write_outputs(input_dir: Path, result: dict[str, object]) -> tuple[Path, Path]:
    json_path = input_dir / "hold_window_reference_analysis.json"
    md_path = input_dir / "hold_window_reference_analysis.md"
    json_path.write_text(json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    lines = [
        "# Hold Window Reference Analysis",
        "",
        f"- input_dir: `{result['input_dir']}`",
        f"- command_source: `{COMMANDS_FILE}`",
        f"- tracking_source: `{CONTROLLER_STATE_TRACKING_FILE}`",
        f"- physical_clearance_m: `{PHYSICAL_CLEARANCE_M:.6f}`",
        f"- state_loop_hz: `{STATE_LOOP_HZ:.1f}`",
        f"- hold_like_command_count: `{result['hold_like_command_count']}`",
        f"- hold_like_best_feedback_1mm_ticks: `{result['hold_like_best_feedback_1mm_ticks']}`",
        "",
        "| Cmd | Hold | Samples | Duration s | Target XY m | Target depth m | Ref mean XY m | Ref max XY m | Feedback mean XY m | Feedback max XY m | Feedback final XY m | Ref best 1mm ticks | Feedback best 1mm ticks | P95 JTC error rad |",
        "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for summary in result.get("commands", []):
        if not isinstance(summary, dict):
            continue
        ref_window = summary.get("reference_best_1mm_window", {})
        fb_window = summary.get("feedback_best_1mm_window", {})
        ref_ticks = ref_window.get("ticks", 0) if isinstance(ref_window, dict) else 0
        fb_ticks = fb_window.get("ticks", 0) if isinstance(fb_window, dict) else 0
        lines.append(
            "| "
            + " | ".join(
                [
                    str(summary.get("command_index")),
                    "yes" if summary.get("hold_like") else "no",
                    str(summary.get("samples")),
                    _fmt(summary.get("duration_s"), 3),
                    _fmt(summary.get("target_xy_error_m")),
                    _fmt(summary.get("target_depth_m")),
                    _fmt(summary.get("mean_reference_xy_error_m")),
                    _fmt(summary.get("max_reference_xy_error_m")),
                    _fmt(summary.get("mean_feedback_xy_error_m")),
                    _fmt(summary.get("max_feedback_xy_error_m")),
                    _fmt(summary.get("final_feedback_xy_error_m")),
                    str(ref_ticks),
                    str(fb_ticks),
                    _fmt(summary.get("p95_max_abs_joint_error_rad")),
                ]
            )
            + " |"
        )
    lines.extend(
        [
            "",
            "Interpretation: this is an offline passive-log diagnostic. A hold-like command "
            "is a single-point trajectory lasting at least 1 s. The analyzer compares the "
            "controller reference and feedback in each command window; it does not publish "
            "commands or alter the task safety gates.",
        ]
    )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return md_path, json_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path)
    args = parser.parse_args()
    result = analyze(args.input_dir)
    md_path, _ = write_outputs(args.input_dir, result)
    print(md_path)


if __name__ == "__main__":
    main()
