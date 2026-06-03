#!/usr/bin/env python3
"""Attribute peg-tip XY drift to controller tracking error in passive logs."""

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


COMMANDS_FILE = "trajectory_commands.csv"
CONTROLLER_STATE_TRACKING_FILE = "trajectory_controller_state_samples.csv"
HOLE_CENTRE_XY = np.array([0.520, -0.200])
PHYSICAL_CLEARANCE_M = 0.001
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
    reference: np.ndarray
    feedback: np.ndarray
    error_reference_minus_feedback: np.ndarray


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
                reference = np.array([float(row[f"{name}_reference_rad"]) for name in joint_names])
                feedback = np.array([float(row[f"{name}_feedback_rad"]) for name in joint_names])
                error = np.array([float(row[f"{name}_error_rad"]) for name in joint_names])
            except KeyError:
                continue
            rows.append(
                TrackingSample(
                    stamp_s=float(row["stamp_s"]),
                    max_abs_error_rad=float(row["max_abs_position_error_rad"]),
                    rms_error_rad=float(row["rms_position_error_rad"]),
                    reference=reference,
                    feedback=feedback,
                    error_reference_minus_feedback=error,
                )
            )
    return rows


def _joint_stats(joint_names: list[str], values_by_joint: list[list[float]]) -> list[dict[str, object]]:
    stats: list[dict[str, object]] = []
    for name, values in zip(joint_names, values_by_joint):
        stats.append(
            {
                "joint": name,
                "mean_abs_xy_contribution_m": _mean(values),
                "p95_abs_xy_contribution_m": _percentile(values, 95.0),
                "max_abs_xy_contribution_m": max(values) if values else None,
            }
        )
    return sorted(
        stats,
        key=lambda row: float(row["p95_abs_xy_contribution_m"] or 0.0),
        reverse=True,
    )


def _summarize_command(
    command: CommandRow,
    samples: list[TrackingSample],
    next_stamp_s: float | None,
    kin: RobotKinematics,
) -> dict[str, object]:
    target_xy_error = float(np.linalg.norm(np.array(command.target_xyz_m[:2]) - HOLE_CENTRE_XY))
    hold_like = command.point_count == 1 and command.final_time_from_start_s >= HOLD_MIN_DURATION_S

    ref_xy_errors: list[float] = []
    fb_xy_errors: list[float] = []
    actual_xy_drifts: list[float] = []
    predicted_xy_drifts: list[float] = []
    prediction_residuals: list[float] = []
    max_abs_joint_errors: list[float] = []
    rms_joint_errors: list[float] = []
    max_xy_sensitivity_by_sample: list[float] = []
    condition_numbers: list[float] = []
    contribution_by_joint: list[list[float]] = [[] for _ in command.joint_names]

    final_reference_xy_error = None
    final_feedback_xy_error = None
    final_actual_xy_drift = None
    final_predicted_xy_drift = None

    for sample in samples:
        reference_xyz, _ = kin.pose(sample.reference)
        feedback_xyz, _ = kin.pose(sample.feedback)
        actual_xy = feedback_xyz[:2] - reference_xyz[:2]
        delta_q = sample.feedback - sample.reference
        jac_xy = kin.jacobian(sample.reference)[:2, :]
        predicted_xy = jac_xy @ delta_q
        residual_xy = actual_xy - predicted_xy
        singular_values = np.linalg.svd(jac_xy, compute_uv=False)
        if len(singular_values) >= 2 and singular_values[-1] > 1e-12:
            condition_numbers.append(float(singular_values[0] / singular_values[-1]))
        column_norms = np.linalg.norm(jac_xy, axis=0)
        max_xy_sensitivity_by_sample.append(float(np.max(column_norms)))

        ref_xy_error = float(np.linalg.norm(reference_xyz[:2] - HOLE_CENTRE_XY))
        fb_xy_error = float(np.linalg.norm(feedback_xyz[:2] - HOLE_CENTRE_XY))
        actual_norm = float(np.linalg.norm(actual_xy))
        predicted_norm = float(np.linalg.norm(predicted_xy))
        residual_norm = float(np.linalg.norm(residual_xy))

        ref_xy_errors.append(ref_xy_error)
        fb_xy_errors.append(fb_xy_error)
        actual_xy_drifts.append(actual_norm)
        predicted_xy_drifts.append(predicted_norm)
        prediction_residuals.append(residual_norm)
        max_abs_joint_errors.append(sample.max_abs_error_rad)
        rms_joint_errors.append(sample.rms_error_rad)
        for index, delta in enumerate(delta_q):
            contribution_by_joint[index].append(float(np.linalg.norm(jac_xy[:, index] * delta)))

        final_reference_xy_error = ref_xy_error
        final_feedback_xy_error = fb_xy_error
        final_actual_xy_drift = actual_norm
        final_predicted_xy_drift = predicted_norm

    return {
        "command_index": command.index,
        "receipt_stamp_s": command.receipt_stamp_s,
        "next_command_stamp_s": next_stamp_s,
        "point_count": command.point_count,
        "duration_s": command.final_time_from_start_s,
        "hold_like": hold_like,
        "samples": len(samples),
        "target_xy_error_m": target_xy_error,
        "target_xyz_m": command.target_xyz_m,
        "target_within_physical_clearance": target_xy_error <= PHYSICAL_CLEARANCE_M,
        "mean_reference_xy_error_m": _mean(ref_xy_errors),
        "mean_feedback_xy_error_m": _mean(fb_xy_errors),
        "final_reference_xy_error_m": final_reference_xy_error,
        "final_feedback_xy_error_m": final_feedback_xy_error,
        "mean_actual_reference_to_feedback_xy_drift_m": _mean(actual_xy_drifts),
        "p95_actual_reference_to_feedback_xy_drift_m": _percentile(actual_xy_drifts, 95.0),
        "max_actual_reference_to_feedback_xy_drift_m": max(actual_xy_drifts) if actual_xy_drifts else None,
        "final_actual_reference_to_feedback_xy_drift_m": final_actual_xy_drift,
        "mean_linearized_xy_drift_m": _mean(predicted_xy_drifts),
        "p95_linearized_xy_drift_m": _percentile(predicted_xy_drifts, 95.0),
        "max_linearized_xy_drift_m": max(predicted_xy_drifts) if predicted_xy_drifts else None,
        "final_linearized_xy_drift_m": final_predicted_xy_drift,
        "p95_linearization_residual_m": _percentile(prediction_residuals, 95.0),
        "max_linearization_residual_m": max(prediction_residuals) if prediction_residuals else None,
        "p95_max_abs_joint_error_rad": _percentile(max_abs_joint_errors, 95.0),
        "max_abs_joint_error_rad": max(max_abs_joint_errors) if max_abs_joint_errors else None,
        "mean_rms_joint_error_rad": _mean(rms_joint_errors),
        "mean_max_xy_sensitivity_m_per_rad": _mean(max_xy_sensitivity_by_sample),
        "p95_max_xy_sensitivity_m_per_rad": _percentile(max_xy_sensitivity_by_sample, 95.0),
        "mean_xy_jacobian_condition_number": _mean(condition_numbers),
        "p95_xy_jacobian_condition_number": _percentile(condition_numbers, 95.0),
        "dominant_joints_by_p95_xy_contribution": _joint_stats(command.joint_names, contribution_by_joint),
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
    command_summaries: list[dict[str, object]] = []
    for index, command in enumerate(commands):
        next_stamp = commands[index + 1].receipt_stamp_s if index + 1 < len(commands) else None
        window = [
            sample
            for sample in tracking
            if sample.stamp_s >= command.receipt_stamp_s
            and (next_stamp is None or sample.stamp_s < next_stamp)
        ]
        command_summaries.append(_summarize_command(command, window, next_stamp, kin))

    hold_like = [summary for summary in command_summaries if summary["hold_like"]]
    centered_holds = [
        summary
        for summary in hold_like
        if bool(summary["target_within_physical_clearance"]) and int(summary["samples"]) > 0
    ]
    max_hold_p95_xy_drift = max(
        (
            float(summary["p95_actual_reference_to_feedback_xy_drift_m"] or 0.0)
            for summary in centered_holds
        ),
        default=0.0,
    )
    max_hold_p95_joint_error = max(
        (float(summary["p95_max_abs_joint_error_rad"] or 0.0) for summary in centered_holds),
        default=0.0,
    )

    return {
        "input_dir": str(input_dir),
        "command_source": COMMANDS_FILE,
        "tracking_source": CONTROLLER_STATE_TRACKING_FILE,
        "physical_clearance_m": PHYSICAL_CLEARANCE_M,
        "commands": command_summaries,
        "hold_like_command_count": len(hold_like),
        "centered_hold_like_command_count": len(centered_holds),
        "max_centered_hold_p95_actual_xy_drift_m": max_hold_p95_xy_drift,
        "max_centered_hold_p95_joint_error_rad": max_hold_p95_joint_error,
    }


def write_outputs(input_dir: Path, result: dict[str, object]) -> tuple[Path, Path]:
    json_path = input_dir / "search_tracking_sensitivity_analysis.json"
    md_path = input_dir / "search_tracking_sensitivity_analysis.md"
    json_path.write_text(json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    lines = [
        "# Search Tracking Sensitivity Analysis",
        "",
        f"- input_dir: `{result['input_dir']}`",
        f"- command_source: `{COMMANDS_FILE}`",
        f"- tracking_source: `{CONTROLLER_STATE_TRACKING_FILE}`",
        f"- physical_clearance_m: `{PHYSICAL_CLEARANCE_M:.6f}`",
        f"- hold_like_command_count: `{result['hold_like_command_count']}`",
        f"- centered_hold_like_command_count: `{result['centered_hold_like_command_count']}`",
        f"- max_centered_hold_p95_actual_xy_drift_m: `{result['max_centered_hold_p95_actual_xy_drift_m']:.6f}`",
        f"- max_centered_hold_p95_joint_error_rad: `{result['max_centered_hold_p95_joint_error_rad']:.6f}`",
        "",
        "| Cmd | Hold | Centered target | Samples | Target XY m | Feedback mean XY m | Actual XY drift p95 m | Linearized XY drift p95 m | Residual p95 m | JTC p95 rad | Max sens p95 m/rad | Cond p95 | Dominant joint p95 m |",
        "| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for summary in result.get("commands", []):
        if not isinstance(summary, dict):
            continue
        dominant = summary.get("dominant_joints_by_p95_xy_contribution", [])
        dominant_label = "none"
        if isinstance(dominant, list) and dominant:
            first = dominant[0]
            if isinstance(first, dict):
                dominant_label = (
                    f"{first.get('joint')} {_fmt(first.get('p95_abs_xy_contribution_m'))}"
                )
        lines.append(
            "| "
            + " | ".join(
                [
                    str(summary.get("command_index")),
                    "yes" if summary.get("hold_like") else "no",
                    "yes" if summary.get("target_within_physical_clearance") else "no",
                    str(summary.get("samples")),
                    _fmt(summary.get("target_xy_error_m")),
                    _fmt(summary.get("mean_feedback_xy_error_m")),
                    _fmt(summary.get("p95_actual_reference_to_feedback_xy_drift_m")),
                    _fmt(summary.get("p95_linearized_xy_drift_m")),
                    _fmt(summary.get("p95_linearization_residual_m")),
                    _fmt(summary.get("p95_max_abs_joint_error_rad")),
                    _fmt(summary.get("p95_max_xy_sensitivity_m_per_rad")),
                    _fmt(summary.get("p95_xy_jacobian_condition_number"), 3),
                    dominant_label,
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "Interpretation: this passive diagnostic uses the current finite-difference "
            "peg-tip Jacobian at each controller reference. The linearized XY drift is "
            "`J_xy * (feedback - reference)`. Close agreement between actual and "
            "linearized drift means controller tracking error is sufficient to explain "
            "the Cartesian offset; a large residual points to model, frame, or log "
            "consistency issues.",
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
