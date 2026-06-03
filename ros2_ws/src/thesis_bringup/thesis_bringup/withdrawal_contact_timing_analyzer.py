#!/usr/bin/env python3
"""Correlate RETREAT contacts with command timing and peg-tip feedback pose."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

import numpy as np

from kuka_task_control.robot_kinematics import RobotKinematics


HOLE_CENTER_XY_M = np.array([0.520, -0.200], dtype=float)
HOLE_TOP_Z_M = 0.810
FINAL_INSERTION_POSE_M = np.array([0.520, -0.200, 0.790], dtype=float)
COMMAND_TARGET_TOLERANCE_M = 0.035
COMMANDS_FILE = "trajectory_commands.csv"
CONTACT_FILE = "contact_state_samples.csv"
CONTROLLER_TRACKING_FILE = "trajectory_controller_state_samples.csv"
JOINT_TRACKING_FILE = "trajectory_tracking_samples.csv"


@dataclass(frozen=True)
class CommandRow:
    index: int
    receipt_stamp_s: float
    joint_names: list[str]
    point_count: int
    duration_s: float
    final_positions_rad: list[float]
    target_xyz_m: list[float]


@dataclass(frozen=True)
class TrackingSample:
    stamp_s: float
    feedback: list[float]


@dataclass(frozen=True)
class ContactSample:
    stamp_s: float
    state: str
    source: str
    contact_count: int
    force_n: float
    collision_pairs: str


def _float_list(text: str) -> list[float]:
    return [float(value) for value in text.split() if value.strip()]


def _read_commands(path: Path) -> list[CommandRow]:
    kin = RobotKinematics()
    commands: list[CommandRow] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for index, row in enumerate(csv.DictReader(handle)):
            final_positions = _float_list(row["final_positions_rad"])
            target_xyz, _ = kin.pose(np.array(final_positions, dtype=float))
            commands.append(
                CommandRow(
                    index=index,
                    receipt_stamp_s=float(row["receipt_stamp_s"]),
                    joint_names=row["joint_names"].split(),
                    point_count=int(row["point_count"]),
                    duration_s=float(row["final_time_from_start_s"]),
                    final_positions_rad=final_positions,
                    target_xyz_m=[float(value) for value in target_xyz],
                )
            )
    return commands


def _read_tracking(path: Path, joint_names: list[str]) -> list[TrackingSample]:
    samples: list[TrackingSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            feedback = [float(row[f"{name}_feedback_rad"]) for name in joint_names]
            samples.append(TrackingSample(stamp_s=float(row["stamp_s"]), feedback=feedback))
    return samples


def _read_contacts(path: Path) -> list[ContactSample]:
    contacts: list[ContactSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            contacts.append(
                ContactSample(
                    stamp_s=float(row["stamp_s"]),
                    state=row["state"],
                    source=row["source"],
                    contact_count=int(row["contact_count"]),
                    force_n=float(row["max_contact_force_n"]),
                    collision_pairs=row["collision_pairs"],
                )
            )
    return contacts


def _select_insert_command(commands: list[CommandRow]) -> int:
    best_index = -1
    best_error = math.inf
    for command in commands:
        error = float(np.linalg.norm(np.array(command.target_xyz_m, dtype=float) - FINAL_INSERTION_POSE_M))
        if error < best_error:
            best_index = command.index
            best_error = error
    if best_index < 0 or best_error > COMMAND_TARGET_TOLERANCE_M:
        raise RuntimeError(
            "no command target matched the canonical final insertion pose "
            f"within {COMMAND_TARGET_TOLERANCE_M} m"
        )
    return best_index


def _label_command(command: CommandRow, insert_index: int) -> str:
    if command.index < insert_index:
        return "MOVING_TO_START" if command.index == 0 else "APPROACH"
    if command.index == insert_index:
        return "INSERT"
    retreat_number = command.index - insert_index
    if retreat_number == 1:
        return "RETREAT_1"
    return f"RETREAT_{retreat_number}"


def _active_command(commands: list[CommandRow], stamp_s: float) -> CommandRow | None:
    starts = [command.receipt_stamp_s for command in commands]
    index = bisect.bisect_right(starts, stamp_s) - 1
    if index < 0:
        return None
    return commands[index]


def _nearest_tracking(samples: list[TrackingSample], stamps: list[float], stamp_s: float) -> TrackingSample | None:
    if not samples:
        return None
    index = bisect.bisect_left(stamps, stamp_s)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda sample: abs(sample.stamp_s - stamp_s)) if candidates else None


def _format_vec(values: list[float] | np.ndarray) -> str:
    return ", ".join(f"{float(value):.6f}" for value in values)


def analyze_directory(input_dir: Path, output_name: str, json_name: str) -> Path:
    commands_path = input_dir / COMMANDS_FILE
    contact_path = input_dir / CONTACT_FILE
    tracking_path = input_dir / CONTROLLER_TRACKING_FILE
    if not tracking_path.exists():
        tracking_path = input_dir / JOINT_TRACKING_FILE

    if not commands_path.exists():
        raise FileNotFoundError(f"missing {commands_path}")
    if not contact_path.exists():
        raise FileNotFoundError(f"missing {contact_path}")
    if not tracking_path.exists():
        raise FileNotFoundError(f"missing {tracking_path}")

    commands = _read_commands(commands_path)
    if not commands:
        raise RuntimeError(f"no commands in {commands_path}")
    insert_index = _select_insert_command(commands)
    tracking = _read_tracking(tracking_path, commands[0].joint_names)
    tracking_stamps = [sample.stamp_s for sample in tracking]
    contacts = [sample for sample in _read_contacts(contact_path) if sample.contact_count > 0]
    kin = RobotKinematics()

    by_label: dict[str, dict[str, object]] = defaultdict(
        lambda: {
            "samples": 0,
            "sources": defaultdict(int),
            "max_force_n": 0.0,
            "first_stamp_s": None,
            "last_stamp_s": None,
            "depths_m": [],
            "xy_errors_m": [],
            "z_m": [],
            "pairs": defaultdict(lambda: {"samples": 0, "max_force_n": 0.0}),
        }
    )
    events: list[dict[str, object]] = []

    for contact in contacts:
        command = _active_command(commands, contact.stamp_s)
        label = _label_command(command, insert_index) if command is not None else "PRE_COMMAND"
        tracking_sample = _nearest_tracking(tracking, tracking_stamps, contact.stamp_s)
        pose = None
        tracking_delay = None
        depth = 0.0
        xy_error = 0.0
        if tracking_sample is not None:
            tracking_delay = contact.stamp_s - tracking_sample.stamp_s
            pose_array, _ = kin.pose(np.array(tracking_sample.feedback, dtype=float))
            pose = [float(value) for value in pose_array]
            depth = max(0.0, HOLE_TOP_Z_M - float(pose_array[2]))
            xy_error = float(np.linalg.norm(pose_array[:2] - HOLE_CENTER_XY_M))

        data = by_label[label]
        data["samples"] = int(data["samples"]) + 1
        data["sources"][contact.source] += 1
        data["max_force_n"] = max(float(data["max_force_n"]), contact.force_n)
        data["first_stamp_s"] = contact.stamp_s if data["first_stamp_s"] is None else min(float(data["first_stamp_s"]), contact.stamp_s)
        data["last_stamp_s"] = contact.stamp_s if data["last_stamp_s"] is None else max(float(data["last_stamp_s"]), contact.stamp_s)
        data["depths_m"].append(depth)
        data["xy_errors_m"].append(xy_error)
        if pose is not None:
            data["z_m"].append(pose[2])
        for pair in contact.collision_pairs.split("; "):
            if not pair:
                continue
            pair_data = data["pairs"][pair]
            pair_data["samples"] += 1
            pair_data["max_force_n"] = max(pair_data["max_force_n"], contact.force_n)

        events.append(
            {
                "stamp_s": contact.stamp_s,
                "state": contact.state,
                "source": contact.source,
                "command_label": label,
                "command_index": command.index if command is not None else None,
                "force_n": contact.force_n,
                "depth_m": depth,
                "xy_error_m": xy_error,
                "peg_tip_xyz_m": pose,
                "tracking_stamp_delay_s": tracking_delay,
                "collision_pairs": contact.collision_pairs,
            }
        )

    command_windows = []
    for index, command in enumerate(commands):
        next_stamp = commands[index + 1].receipt_stamp_s if index + 1 < len(commands) else None
        command_windows.append(
            {
                "index": command.index,
                "label": _label_command(command, insert_index),
                "receipt_stamp_s": command.receipt_stamp_s,
                "next_command_stamp_s": next_stamp,
                "duration_s": command.duration_s,
                "point_count": command.point_count,
                "target_xyz_m": command.target_xyz_m,
            }
        )

    summaries: dict[str, object] = {}
    for label, data in by_label.items():
        depths = data["depths_m"]
        xy_errors = data["xy_errors_m"]
        z_values = data["z_m"]
        top_pairs = sorted(
            data["pairs"].items(),
            key=lambda item: (item[1]["max_force_n"], item[1]["samples"]),
            reverse=True,
        )[:5]
        summaries[label] = {
            "samples": data["samples"],
            "sources": dict(data["sources"]),
            "first_stamp_s": data["first_stamp_s"],
            "last_stamp_s": data["last_stamp_s"],
            "max_force_n": data["max_force_n"],
            "max_depth_m": max(depths, default=0.0),
            "mean_depth_m": mean(depths) if depths else 0.0,
            "max_xy_error_m": max(xy_errors, default=0.0),
            "mean_xy_error_m": mean(xy_errors) if xy_errors else 0.0,
            "min_peg_tip_z_m": min(z_values, default=0.0),
            "max_peg_tip_z_m": max(z_values, default=0.0),
            "top_collision_pairs": [
                {
                    "collision_pair": pair,
                    "samples": pair_data["samples"],
                    "max_force_n": pair_data["max_force_n"],
                }
                for pair, pair_data in top_pairs
            ],
        }

    result = {
        "input_dir": str(input_dir),
        "tracking_source": tracking_path.name,
        "insert_command_index": insert_index,
        "positive_contact_samples": len(contacts),
        "command_windows": command_windows,
        "contact_by_command": summaries,
        "first_20_events": events[:20],
        "top_20_force_events": sorted(events, key=lambda event: float(event["force_n"]), reverse=True)[:20],
    }

    json_path = input_dir / json_name
    json_path.write_text(json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    lines = [
        "# Withdrawal Contact Timing Analysis",
        "",
        f"- input_dir: `{input_dir}`",
        f"- tracking_source: `{tracking_path.name}`",
        f"- insert_command_index: `{insert_index}`",
        f"- positive_contact_samples: `{len(contacts)}`",
        "",
        "## Command Windows",
        "",
        "| index | label | receipt_stamp_s | next_command_stamp_s | duration_s | point_count | target_xyz_m |",
        "| ---: | --- | ---: | ---: | ---: | ---: | --- |",
    ]
    for window in command_windows:
        next_stamp = window["next_command_stamp_s"]
        next_text = f"{next_stamp:.3f}" if next_stamp is not None else "none"
        lines.append(
            f"| {window['index']} | {window['label']} | {window['receipt_stamp_s']:.3f} | "
            f"{next_text} | {window['duration_s']:.3f} | {window['point_count']} | "
            f"`{_format_vec(window['target_xyz_m'])}` |"
        )

    lines.extend([
        "",
        "## Positive Contact By Active Command",
        "",
        "| command | samples | first_stamp_s | last_stamp_s | max_force_n | max_depth_m | max_xy_error_m | z_range_m | top_collision_pair |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |",
    ])
    for label in sorted(summaries):
        data = summaries[label]
        top_pair = data["top_collision_pairs"][0]["collision_pair"] if data["top_collision_pairs"] else ""
        lines.append(
            f"| {label} | {data['samples']} | {data['first_stamp_s']:.3f} | {data['last_stamp_s']:.3f} | "
            f"{data['max_force_n']:.6f} | {data['max_depth_m']:.6f} | {data['max_xy_error_m']:.6f} | "
            f"{data['min_peg_tip_z_m']:.6f}..{data['max_peg_tip_z_m']:.6f} | `{top_pair}` |"
        )

    lines.extend([
        "",
        "## Highest Force Events",
        "",
        "| stamp_s | command | source | force_n | depth_m | xy_error_m | peg_tip_xyz_m | collision_pairs |",
        "| ---: | --- | --- | ---: | ---: | ---: | --- | --- |",
    ])
    for event in result["top_20_force_events"][:10]:
        pose = event["peg_tip_xyz_m"]
        pose_text = _format_vec(pose) if pose is not None else "none"
        lines.append(
            f"| {event['stamp_s']:.3f} | {event['command_label']} | {event['source']} | "
            f"{event['force_n']:.6f} | {event['depth_m']:.6f} | {event['xy_error_m']:.6f} | "
            f"`{pose_text}` | `{event['collision_pairs']}` |"
        )

    lines.extend([
        "",
        "Interpretation: this is an offline diagnostic. It uses recorded command, "
        "contact, and tracking CSVs only; it does not publish robot commands or "
        "change safety gates.",
    ])
    output_path = input_dir / output_name
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_dir", type=Path, help="Diagnostics directory containing contact and trajectory CSVs")
    parser.add_argument("--output-name", default="withdrawal_contact_timing_analysis.md")
    parser.add_argument("--json-name", default="withdrawal_contact_timing_analysis.json")
    args = parser.parse_args()
    print(analyze_directory(args.input_dir, args.output_name, args.json_name))


if __name__ == "__main__":
    main()
