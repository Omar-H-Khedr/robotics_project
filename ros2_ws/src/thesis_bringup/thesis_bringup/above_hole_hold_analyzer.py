#!/usr/bin/env python3
"""Analyze above-hole strict-gate hold windows from passive observer CSVs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean


STRICT_XY_M = 0.002
TRIAL_OUTCOME_FILE = "trial_outcome.json"
DEFAULT_STATE_LOOP_HZ = 10.0
REQUIRED_STABLE_TICKS = 5
STATE_NAME = "MOVING_TO_START"


@dataclass(frozen=True)
class HoldSample:
    stamp_s: float
    state: str
    xy_error_m: float
    peg_z_m: float
    force_norm_n: float


def _read_samples(path: Path) -> list[HoldSample]:
    samples: list[HoldSample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                xy_error = float(row["xy_error_m"])
                peg_z = float(row["peg_z_m"])
                force_norm = float(row["force_norm_n"])
            except (KeyError, ValueError):
                continue
            if math.isnan(xy_error):
                continue
            samples.append(
                HoldSample(
                    stamp_s=float(row["stamp_s"]),
                    state=row.get("state", ""),
                    xy_error_m=xy_error,
                    peg_z_m=peg_z,
                    force_norm_n=force_norm,
                )
            )
    return samples


def _continuous_strict_windows(samples: list[HoldSample]) -> list[list[HoldSample]]:
    windows: list[list[HoldSample]] = []
    current: list[HoldSample] = []
    for sample in samples:
        if sample.state == STATE_NAME and sample.xy_error_m <= STRICT_XY_M:
            current.append(sample)
        elif current:
            windows.append(current)
            current = []
    if current:
        windows.append(current)
    return windows


def _infer_state_loop_hz(input_dir: Path, override_hz: float | None = None) -> float:
    if override_hz is not None and override_hz > 0.0:
        return override_hz
    outcome_path = input_dir / TRIAL_OUTCOME_FILE
    if outcome_path.exists():
        try:
            outcome = json.loads(outcome_path.read_text(encoding="utf-8"))
            metrics = outcome.get("metrics", {})
            if isinstance(metrics, dict):
                control_rate_hz = float(metrics.get("control_rate_hz", 0.0))
                if control_rate_hz > 0.0:
                    return control_rate_hz
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            pass
    return DEFAULT_STATE_LOOP_HZ


def _estimated_state_loop_ticks(
    samples: list[HoldSample],
    state_loop_hz: float,
) -> tuple[int, float | None, float | None]:
    """Replay strict samples at the task node's configured cadence.

    Observer data is higher-rate than the task state machine. This samples the
    recorded state/XY stream by nearest sample at or after each tick. It is an
    approximation, but it is stricter than counting adjacent high-rate observer
    rows.
    """
    moving = [sample for sample in samples if sample.state == STATE_NAME]
    if not moving:
        return 0, None, None

    tick_period = 1.0 / state_loop_hz
    next_tick = moving[0].stamp_s
    index = 0
    current = 0
    best = 0
    best_start: float | None = None
    best_end: float | None = None
    current_start: float | None = None

    while next_tick <= moving[-1].stamp_s and index < len(moving):
        while index + 1 < len(moving) and moving[index].stamp_s < next_tick:
            index += 1
        sample = moving[index]
        if sample.xy_error_m <= STRICT_XY_M:
            if current == 0:
                current_start = next_tick
            current += 1
            if current > best:
                best = current
                best_start = current_start
                best_end = next_tick
        else:
            current = 0
            current_start = None
        next_tick += tick_period

    return best, best_start, best_end


def analyze(input_dir: Path, state_loop_hz: float | None = None) -> dict[str, object]:
    samples_path = input_dir / "wrench_state_samples.csv"
    if not samples_path.exists():
        raise FileNotFoundError(f"missing {samples_path}")

    inferred_state_loop_hz = _infer_state_loop_hz(input_dir, state_loop_hz)
    samples = _read_samples(samples_path)
    moving = [sample for sample in samples if sample.state == STATE_NAME]
    if not moving:
        result: dict[str, object] = {
            "input_dir": str(input_dir),
            "state": STATE_NAME,
            "samples": len(samples),
            "moving_to_start_samples": 0,
            "result": "NO_MOVING_TO_START_SAMPLES",
        }
        return result

    strict_windows = _continuous_strict_windows(samples)
    best_window = max(strict_windows, key=len) if strict_windows else []
    best_duration = (
        best_window[-1].stamp_s - best_window[0].stamp_s
        if len(best_window) >= 2
        else 0.0
    )
    best_ticks, best_tick_start, best_tick_end = _estimated_state_loop_ticks(
        samples,
        inferred_state_loop_hz,
    )
    min_sample = min(moving, key=lambda sample: sample.xy_error_m)
    final_sample = moving[-1]
    strict_count = sum(1 for sample in moving if sample.xy_error_m <= STRICT_XY_M)
    xy_values = [sample.xy_error_m for sample in moving]

    return {
        "input_dir": str(input_dir),
        "state": STATE_NAME,
        "strict_xy_m": STRICT_XY_M,
        "required_stable_ticks": REQUIRED_STABLE_TICKS,
        "state_loop_hz": inferred_state_loop_hz,
        "samples": len(samples),
        "moving_to_start_samples": len(moving),
        "strict_samples": strict_count,
        "strict_sample_fraction": strict_count / len(moving),
        "best_continuous_strict_samples": len(best_window),
        "best_continuous_strict_duration_s": best_duration,
        "best_continuous_strict_start_s": best_window[0].stamp_s if best_window else None,
        "best_continuous_strict_end_s": best_window[-1].stamp_s if best_window else None,
        "estimated_state_loop_best_stable_ticks": best_ticks,
        "estimated_state_loop_best_start_s": best_tick_start,
        "estimated_state_loop_best_end_s": best_tick_end,
        "estimated_state_loop_gate_passed": best_ticks >= REQUIRED_STABLE_TICKS,
        "min_xy_error_m": min_sample.xy_error_m,
        "min_xy_stamp_s": min_sample.stamp_s,
        "min_xy_peg_z_m": min_sample.peg_z_m,
        "final_moving_xy_error_m": final_sample.xy_error_m,
        "final_moving_stamp_s": final_sample.stamp_s,
        "mean_moving_xy_error_m": mean(xy_values),
    }


def _fmt(value: object, digits: int = 6) -> str:
    if value is None:
        return "none"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def write_outputs(input_dir: Path, result: dict[str, object]) -> None:
    json_path = input_dir / "above_hole_hold_analysis.json"
    md_path = input_dir / "above_hole_hold_analysis.md"
    json_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# Above-Hole Hold Analysis",
        "",
        f"- input_dir: `{result['input_dir']}`",
        f"- state: `{STATE_NAME}`",
        f"- strict_xy_m: `{_fmt(result.get('strict_xy_m'))}`",
        f"- required_stable_ticks: `{REQUIRED_STABLE_TICKS}`",
        f"- state_loop_hz: `{float(result.get('state_loop_hz', DEFAULT_STATE_LOOP_HZ)):.1f}`",
        f"- moving_to_start_samples: `{result.get('moving_to_start_samples', 0)}`",
        f"- strict_samples: `{result.get('strict_samples', 0)}`",
        f"- best_continuous_strict_samples: `{result.get('best_continuous_strict_samples', 0)}`",
        f"- best_continuous_strict_duration_s: `{_fmt(result.get('best_continuous_strict_duration_s'))}`",
        f"- estimated_state_loop_best_stable_ticks: `{result.get('estimated_state_loop_best_stable_ticks', 0)}`",
        f"- estimated_state_loop_gate_passed: `{result.get('estimated_state_loop_gate_passed', False)}`",
        f"- min_xy_error_m: `{_fmt(result.get('min_xy_error_m'))}`",
        f"- min_xy_stamp_s: `{_fmt(result.get('min_xy_stamp_s'), 3)}`",
        f"- final_moving_xy_error_m: `{_fmt(result.get('final_moving_xy_error_m'))}`",
        f"- mean_moving_xy_error_m: `{_fmt(result.get('mean_moving_xy_error_m'))}`",
        "",
        "Interpretation: this is an offline diagnostic over passive observer CSVs. "
        "It estimates whether the recorded above-hole hold would satisfy the "
        "task controller's strict 2 mm XY gate for five configured-cadence "
        "state-loop ticks. "
        "It does not publish commands or alter task safety gates.",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze above-hole hold stability from wrench_state_samples.csv"
    )
    parser.add_argument("input_dir", type=Path)
    parser.add_argument(
        "--state-loop-hz",
        type=float,
        default=None,
        help=(
            "Task state-machine cadence in Hz. Defaults to metrics.control_rate_hz "
            "from trial_outcome.json in input_dir, falling back to 10 Hz."
        ),
    )
    args = parser.parse_args()

    result = analyze(args.input_dir, args.state_loop_hz)
    write_outputs(args.input_dir, result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
