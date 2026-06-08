#!/usr/bin/env python3
"""Analyze online SEARCH gate decisions from admittance_insertion_node traces."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


TRACE_FILE = "search_gate_trace.csv"
JSON_OUTPUT = "search_gate_trace_analysis.json"
MD_OUTPUT = "search_gate_trace_analysis.md"
PHYSICAL_CLEARANCE_M = 0.001
REQUIRED_TICKS = 4


@dataclass(frozen=True)
class TraceRow:
    stamp_s: float
    search_tick: int
    state_entry_ticks: int
    elapsed_s: float
    settle_elapsed_s: float
    ready_to_count: bool
    settling_window_active: bool
    command_still_running: bool
    search_step: int
    recenter_attempts: int
    convergence_ticks_before: int
    convergence_ticks_after: int
    xy_error_m: float
    peg_z_m: float
    stability_ready_s: float
    decision: str


def _as_bool(text: str) -> bool:
    return str(text).strip() in {"1", "true", "True"}


def _read_rows(path: Path) -> list[TraceRow]:
    rows: list[TraceRow] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            try:
                xy_error_m = float(raw["xy_error_m"])
            except (KeyError, ValueError):
                continue
            if math.isnan(xy_error_m):
                continue
            rows.append(
                TraceRow(
                    stamp_s=float(raw["stamp_s"]),
                    search_tick=int(raw["search_tick"]),
                    state_entry_ticks=int(raw["state_entry_ticks"]),
                    elapsed_s=float(raw["elapsed_s"]),
                    settle_elapsed_s=float(raw["settle_elapsed_s"]),
                    ready_to_count=_as_bool(raw["ready_to_count"]),
                    settling_window_active=_as_bool(raw["settling_window_active"]),
                    command_still_running=_as_bool(raw["command_still_running"]),
                    search_step=int(raw["search_step"]),
                    recenter_attempts=int(raw["recenter_attempts"]),
                    convergence_ticks_before=int(raw["convergence_ticks_before"]),
                    convergence_ticks_after=int(raw["convergence_ticks_after"]),
                    xy_error_m=xy_error_m,
                    peg_z_m=float(raw["peg_z_m"]),
                    stability_ready_s=float(raw["stability_ready_s"]),
                    decision=raw.get("decision", "") or "",
                )
            )
    return rows


def _best_streak(rows: list[TraceRow], *, require_ready: bool) -> dict[str, object]:
    best: list[TraceRow] = []
    current: list[TraceRow] = []
    for row in rows:
        inside = row.xy_error_m <= PHYSICAL_CLEARANCE_M
        ready = row.ready_to_count or not require_ready
        if inside and ready:
            current.append(row)
        else:
            if len(current) > len(best):
                best = current
            current = []
    if len(current) > len(best):
        best = current

    return {
        "ticks": len(best),
        "start_search_tick": best[0].search_tick if best else None,
        "end_search_tick": best[-1].search_tick if best else None,
        "start_elapsed_s": best[0].elapsed_s if best else None,
        "end_elapsed_s": best[-1].elapsed_s if best else None,
        "min_xy_error_m": min((row.xy_error_m for row in best), default=None),
        "max_xy_error_m": max((row.xy_error_m for row in best), default=None),
    }


def _fmt(value: object, digits: int = 6) -> str:
    if value is None:
        return "none"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def analyze(path_or_dir: Path) -> dict[str, object]:
    trace_path = path_or_dir / TRACE_FILE if path_or_dir.is_dir() else path_or_dir
    if not trace_path.exists():
        raise FileNotFoundError(f"missing {trace_path}")

    rows = _read_rows(trace_path)
    decision_counts = Counter(row.decision for row in rows)
    max_convergence_after = max(
        (row.convergence_ticks_after for row in rows),
        default=0,
    )
    max_convergence_before = max(
        (row.convergence_ticks_before for row in rows),
        default=0,
    )
    converged_rows = [
        row for row in rows if "converged" in row.decision
    ]
    final_row = rows[-1] if rows else None

    return {
        "input": str(trace_path),
        "physical_clearance_m": PHYSICAL_CLEARANCE_M,
        "required_ticks": REQUIRED_TICKS,
        "rows": len(rows),
        "decision_counts": dict(sorted(decision_counts.items())),
        "first_elapsed_s": rows[0].elapsed_s if rows else None,
        "last_elapsed_s": rows[-1].elapsed_s if rows else None,
        "final_decision": final_row.decision if final_row else None,
        "final_xy_error_m": final_row.xy_error_m if final_row else None,
        "max_convergence_ticks_before": max_convergence_before,
        "max_convergence_ticks_after": max_convergence_after,
        "passed_online_gate": bool(
            max_convergence_after >= REQUIRED_TICKS or converged_rows
        ),
        "converged_decisions": [row.decision for row in converged_rows],
        "best_ready_inside_clearance_streak": _best_streak(
            rows,
            require_ready=True,
        ),
        "best_all_trace_inside_clearance_streak": _best_streak(
            rows,
            require_ready=False,
        ),
    }


def write_outputs(result: dict[str, object], output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / JSON_OUTPUT
    md_path = output_dir / MD_OUTPUT
    json_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    decisions = result["decision_counts"]
    assert isinstance(decisions, dict)
    ready = result["best_ready_inside_clearance_streak"]
    all_trace = result["best_all_trace_inside_clearance_streak"]
    assert isinstance(ready, dict)
    assert isinstance(all_trace, dict)

    lines = [
        "# SEARCH Gate Trace Analysis",
        "",
        f"- input: `{result['input']}`",
        f"- physical_clearance_m: `{_fmt(result['physical_clearance_m'])}`",
        f"- required_ticks: `{result['required_ticks']}`",
        f"- rows: `{result['rows']}`",
        f"- final_decision: `{result['final_decision']}`",
        f"- final_xy_error_m: `{_fmt(result['final_xy_error_m'])}`",
        f"- max_convergence_ticks_after: `{result['max_convergence_ticks_after']}`",
        f"- passed_online_gate: `{result['passed_online_gate']}`",
        "",
        "| Decision | Rows |",
        "| --- | ---: |",
    ]
    for decision, count in decisions.items():
        lines.append(f"| {decision or 'none'} | {count} |")
    lines.extend([
        "",
        "| Window | Ticks | Start tick | End tick | Min XY m | Max XY m |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
        (
            "| ready inside clearance | "
            f"{ready['ticks']} | {ready['start_search_tick']} | "
            f"{ready['end_search_tick']} | {_fmt(ready['min_xy_error_m'])} | "
            f"{_fmt(ready['max_xy_error_m'])} |"
        ),
        (
            "| all trace inside clearance | "
            f"{all_trace['ticks']} | {all_trace['start_search_tick']} | "
            f"{all_trace['end_search_tick']} | {_fmt(all_trace['min_xy_error_m'])} | "
            f"{_fmt(all_trace['max_xy_error_m'])} |"
        ),
        "",
        (
            "Interpretation: `max_convergence_ticks_after` is the online "
            "state-machine counter recorded by `admittance_insertion_node`. "
            "It is more authoritative than passive observer replay for "
            "deciding whether SEARCH was allowed to enter INSERT."
        ),
    ])
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return json_path, md_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze search_gate_trace.csv online SEARCH gate evidence."
    )
    parser.add_argument(
        "path",
        type=Path,
        help="Diagnostics directory containing search_gate_trace.csv, or the CSV path.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for search_gate_trace_analysis.{json,md}; defaults to input dir.",
    )
    args = parser.parse_args()

    result = analyze(args.path)
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = args.path if args.path.is_dir() else args.path.parent
    _, md_path = write_outputs(result, output_dir)
    print(md_path)


if __name__ == "__main__":
    main()
