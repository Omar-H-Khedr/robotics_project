#!/usr/bin/env python3
"""Offline clearance diagnostic for the iisy6 MOVING_TO_START path."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

from kuka_task_control.robot_kinematics import RobotKinematics, _make_transform


JOINT_NAMES = [f"joint_{index}" for index in range(1, 7)]
SAFE_HOME = np.array([0.0, -0.8, 1.2, 0.0, 0.8, 0.0])
AXIS_ALIGN_POSE = np.array([0.520, -0.200, 0.885])
PEG_AXIS_WORLD = np.array([0.0, 0.0, 1.0])

LINK5_COLLISION_ORIGIN = np.array([0.04, 0.0, -0.03])
LINK5_COLLISION_SIZE = np.array([0.17, 0.12, 0.12])

TARGET_PLATE_BOXES = {
    "target_plate_collision": (
        np.array([0.520, -0.200 + 0.054, 0.790 + 0.010]),
        np.array([0.180, 0.072, 0.020]),
    ),
    "plate_rear_collision": (
        np.array([0.520, -0.200 - 0.054, 0.790 + 0.010]),
        np.array([0.180, 0.072, 0.020]),
    ),
    "plate_left_collision": (
        np.array([0.520 - 0.054, -0.200, 0.790 + 0.010]),
        np.array([0.072, 0.036, 0.020]),
    ),
    "plate_right_collision": (
        np.array([0.520 + 0.054, -0.200, 0.790 + 0.010]),
        np.array([0.072, 0.036, 0.020]),
    ),
}


@dataclass
class SampleResult:
    label: str
    stamp_s: float | None
    joints: np.ndarray
    peg_xyz: np.ndarray
    link5_center: np.ndarray
    intersections: list[str]
    min_aabb_distance_m: float


def _box_intersects(
    center_a: np.ndarray,
    axes_a: np.ndarray,
    half_a: np.ndarray,
    center_b: np.ndarray,
    axes_b: np.ndarray,
    half_b: np.ndarray,
    eps: float = 1e-9,
) -> bool:
    """Separating-axis test for two oriented boxes."""
    rot = axes_a.T @ axes_b
    abs_rot = np.abs(rot) + eps
    translation = axes_a.T @ (center_b - center_a)

    for axis in range(3):
        radius_a = half_a[axis]
        radius_b = float(np.dot(half_b, abs_rot[axis, :]))
        if abs(translation[axis]) > radius_a + radius_b:
            return False

    for axis in range(3):
        radius_a = float(np.dot(half_a, abs_rot[:, axis]))
        radius_b = half_b[axis]
        if abs(float(np.dot(translation, rot[:, axis]))) > radius_a + radius_b:
            return False

    for axis_a in range(3):
        for axis_b in range(3):
            radius_a = (
                half_a[(axis_a + 1) % 3] * abs_rot[(axis_a + 2) % 3, axis_b]
                + half_a[(axis_a + 2) % 3] * abs_rot[(axis_a + 1) % 3, axis_b]
            )
            radius_b = (
                half_b[(axis_b + 1) % 3] * abs_rot[axis_a, (axis_b + 2) % 3]
                + half_b[(axis_b + 2) % 3] * abs_rot[axis_a, (axis_b + 1) % 3]
            )
            distance = abs(
                translation[(axis_a + 2) % 3] * rot[(axis_a + 1) % 3, axis_b]
                - translation[(axis_a + 1) % 3] * rot[(axis_a + 2) % 3, axis_b]
            )
            if distance > radius_a + radius_b:
                return False
    return True


def _obb_corners(center: np.ndarray, axes: np.ndarray, half: np.ndarray) -> list[np.ndarray]:
    corners: list[np.ndarray] = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                corners.append(
                    center
                    + sx * half[0] * axes[:, 0]
                    + sy * half[1] * axes[:, 1]
                    + sz * half[2] * axes[:, 2]
                )
    return corners


def _point_aabb_distance(point: np.ndarray, center: np.ndarray, half: np.ndarray) -> float:
    delta = np.maximum(np.abs(point - center) - half, 0.0)
    return float(np.linalg.norm(delta))


def _min_corner_aabb_distance(
    center: np.ndarray,
    axes: np.ndarray,
    half: np.ndarray,
    aabb_center: np.ndarray,
    aabb_half: np.ndarray,
) -> float:
    return min(
        _point_aabb_distance(corner, aabb_center, aabb_half)
        for corner in _obb_corners(center, axes, half)
    )


def _read_tracking_samples(path: Path, stride: int) -> list[tuple[str, float, np.ndarray]]:
    rows: list[tuple[str, float, np.ndarray]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader):
            if index % stride != 0:
                continue
            joints = np.array(
                [float(row[f"{name}_feedback_rad"]) for name in JOINT_NAMES],
                dtype=float,
            )
            rows.append((f"tracking_{index}", float(row["stamp_s"]), joints))
    return rows


def _planned_samples(kin: RobotKinematics, count: int) -> list[tuple[str, float | None, np.ndarray]]:
    target, converged, err = kin.inverse_position_axis(
        AXIS_ALIGN_POSE,
        PEG_AXIS_WORLD,
        SAFE_HOME,
        max_iter=100,
    )
    if not converged:
        raise RuntimeError(f"Axis-aligned IK did not converge for planned path: err={err:.6f}")
    rows: list[tuple[str, float | None, np.ndarray]] = []
    for index, alpha in enumerate(np.linspace(0.0, 1.0, count)):
        q = SAFE_HOME + alpha * (target - SAFE_HOME)
        rows.append((f"planned_{index:03d}", None, q))
    return rows


def _analyze_samples(
    kin: RobotKinematics,
    samples: Iterable[tuple[str, float | None, np.ndarray]],
) -> list[SampleResult]:
    results: list[SampleResult] = []
    local_box = _make_transform(tuple(LINK5_COLLISION_ORIGIN), (0.0, 0.0, 0.0))
    identity_axes = np.eye(3)

    for label, stamp_s, joints in samples:
        peg_xyz, _peg_rot = kin.pose(joints)
        link5_box = kin.link_transform(joints, "link_5") @ local_box
        center = link5_box[:3, 3]
        axes = link5_box[:3, :3]
        half = LINK5_COLLISION_SIZE / 2.0
        intersections: list[str] = []
        min_distance = float("inf")
        for name, (box_center, box_size) in TARGET_PLATE_BOXES.items():
            box_half = box_size / 2.0
            if _box_intersects(center, axes, half, box_center, identity_axes, box_half):
                intersections.append(name)
                min_distance = 0.0
            elif not intersections:
                min_distance = min(
                    min_distance,
                    _min_corner_aabb_distance(center, axes, half, box_center, box_half),
                )
        results.append(
            SampleResult(
                label=label,
                stamp_s=stamp_s,
                joints=joints,
                peg_xyz=peg_xyz,
                link5_center=center,
                intersections=intersections,
                min_aabb_distance_m=min_distance,
            )
        )
    return results


def _summarize(results: list[SampleResult], title: str) -> dict[str, object]:
    intersecting = [result for result in results if result.intersections]
    first = intersecting[0] if intersecting else None
    min_distance_row = min(results, key=lambda result: result.min_aabb_distance_m)
    return {
        "title": title,
        "samples": len(results),
        "intersection_samples": len(intersecting),
        "first_intersection": _row_to_dict(first) if first else None,
        "min_distance_sample": _row_to_dict(min_distance_row),
    }


def _row_to_dict(row: SampleResult | None) -> dict[str, object] | None:
    if row is None:
        return None
    return {
        "label": row.label,
        "stamp_s": row.stamp_s,
        "peg_xyz": [round(float(value), 6) for value in row.peg_xyz],
        "link5_center_xyz": [round(float(value), 6) for value in row.link5_center],
        "intersections": row.intersections,
        "min_aabb_distance_m": round(float(row.min_aabb_distance_m), 6),
        "joints_rad": [round(float(value), 6) for value in row.joints],
    }


def _markdown(summary: dict[str, object]) -> str:
    lines = [
        "# Clearance Path Analysis",
        "",
        f"- title: `{summary['title']}`",
        f"- samples: `{summary['samples']}`",
        f"- intersection_samples: `{summary['intersection_samples']}`",
        "",
    ]
    first = summary["first_intersection"]
    if isinstance(first, dict):
        lines.extend([
            "## First Intersection",
            "",
            f"- label: `{first['label']}`",
            f"- stamp_s: `{first['stamp_s']}`",
            f"- peg_xyz: `{first['peg_xyz']}`",
            f"- link5_center_xyz: `{first['link5_center_xyz']}`",
            f"- intersections: `{first['intersections']}`",
            f"- joints_rad: `{first['joints_rad']}`",
            "",
        ])
    else:
        lines.extend(["## First Intersection", "", "None.", ""])

    closest = summary["min_distance_sample"]
    if isinstance(closest, dict):
        lines.extend([
            "## Closest Sample",
            "",
            f"- label: `{closest['label']}`",
            f"- stamp_s: `{closest['stamp_s']}`",
            f"- min_aabb_distance_m: `{closest['min_aabb_distance_m']}`",
            f"- peg_xyz: `{closest['peg_xyz']}`",
            f"- link5_center_xyz: `{closest['link5_center_xyz']}`",
            f"- intersections: `{closest['intersections']}`",
            f"- joints_rad: `{closest['joints_rad']}`",
            "",
        ])
    lines.append("This analyzer is offline only and does not publish robot commands.")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("diagnostic_dir", nargs="?", type=Path)
    parser.add_argument("--tracking-csv", type=Path)
    parser.add_argument("--stride", type=int, default=20)
    parser.add_argument("--planned-samples", type=int, default=201)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    tracking_csv = args.tracking_csv
    if tracking_csv is None and args.diagnostic_dir is not None:
        tracking_csv = args.diagnostic_dir / "trajectory_tracking_samples.csv"

    kin = RobotKinematics()
    planned = _analyze_samples(kin, _planned_samples(kin, args.planned_samples))
    summaries = [_summarize(planned, "planned_SAFE_HOME_to_AXIS_ALIGN")]

    if tracking_csv is not None and tracking_csv.exists():
        tracking = _read_tracking_samples(tracking_csv, max(1, args.stride))
        summaries.append(_summarize(_analyze_samples(kin, tracking), "tracking_feedback_samples"))

    output_dir = args.output_dir or args.diagnostic_dir
    payload = {"analyses": summaries}
    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "clearance_path_analysis.json").write_text(
            json.dumps(payload, indent=2) + "\n",
            encoding="utf-8",
        )
        markdown = []
        for summary in summaries:
            markdown.append(_markdown(summary))
        (output_dir / "clearance_path_analysis.md").write_text(
            "\n".join(markdown),
            encoding="utf-8",
        )
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
