#!/usr/bin/env python3
"""Print final segmented Cartesian endpoint diagnostics from the latest trial."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any


FIELD_NAMES = (
    "target_joint_positions",
    "reached_joint_positions",
    "end_effector_position_xyz",
    "end_effector_orientation_xyzw",
)


def main() -> int:
    results_root = _default_results_root()
    task_events_path = _latest_task_events_path(results_root)
    if task_events_path is None:
        print(f"No task_events.csv found under {results_root}")
        return 1

    row = _find_final_endpoint_row(task_events_path)
    print(f"trial_path: {task_events_path.parent}")
    print(f"task_events_csv: {task_events_path}")
    if row is None:
        print("final_segment_endpoint row not found")
        return 1

    print(f"final_segment_endpoint_row: {json.dumps(row, sort_keys=True)}")
    for field_name in FIELD_NAMES:
        print(f"{field_name}: {_format_csv_json(row.get(field_name, ''))}")
    return 0


def _default_results_root() -> Path:
    return Path(__file__).resolve().parents[3] / "results" / "baseline_trials"


def _latest_task_events_path(results_root: Path) -> Path | None:
    candidates = [
        path
        for path in results_root.glob("trial_*/task_events.csv")
        if path.is_file()
    ]
    if not candidates:
        candidates = [
            path
            for path in results_root.glob("*/task_events.csv")
            if path.is_file() and path.parent.name != "latest"
        ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: (path.parent.stat().st_mtime, path.parent.name))


def _find_final_endpoint_row(task_events_path: Path) -> dict[str, str] | None:
    final_row: dict[str, str] | None = None
    with task_events_path.open("r", encoding="utf-8", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if row.get("event_type") == "final_segment_endpoint":
                final_row = row
    return final_row


def _format_csv_json(value: str) -> str:
    if not value:
        return "unavailable"
    try:
        parsed: Any = json.loads(value)
    except json.JSONDecodeError:
        return value
    return json.dumps(parsed)


if __name__ == "__main__":
    raise SystemExit(main())
