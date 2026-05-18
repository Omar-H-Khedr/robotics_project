#!/usr/bin/env python3
"""Print final segmented endpoint diagnostics from the latest trial CSV."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Extract final_segment_endpoint diagnostics from task_events.csv."
    )
    parser.add_argument(
        "--results-root",
        type=Path,
        default=_default_results_root(),
        help="Directory containing baseline trial folders.",
    )
    args = parser.parse_args()

    task_events_path = _latest_task_events_path(args.results_root)
    if task_events_path is None:
        print(f"No task_events.csv found under {args.results_root}")
        return 1

    event = _find_final_endpoint_event(task_events_path)
    print(f"trial_path: {task_events_path.parent}")
    if event is None:
        print("final_segment_endpoint event not found")
        return 1

    print(f"target_joint_positions: {_format_csv_json(event, 'target_joint_positions')}")
    print(f"reached_joint_positions: {_format_csv_json(event, 'reached_joint_positions')}")
    return 0


def _default_results_root() -> Path:
    return Path(__file__).resolve().parents[3] / "results" / "baseline_trials"


def _latest_task_events_path(results_root: Path) -> Path | None:
    latest = results_root / "latest" / "task_events.csv"
    if latest.is_file():
        return latest

    candidates = [
        path
        for path in results_root.glob("trial_*/task_events.csv")
        if path.is_file()
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda path: (path.parent.stat().st_mtime, path.parent.name))


def _find_final_endpoint_event(task_events_path: Path) -> dict[str, str] | None:
    final_event: dict[str, str] | None = None
    with task_events_path.open("r", encoding="utf-8", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if row.get("event_type") == "final_segment_endpoint":
                final_event = row
    return final_event


def _format_csv_json(row: dict[str, str], field_name: str) -> str:
    value = row.get(field_name, "")
    if not value:
        return "unavailable"
    try:
        parsed: Any = json.loads(value)
    except json.JSONDecodeError:
        return value
    return json.dumps(parsed)


if __name__ == "__main__":
    raise SystemExit(main())
