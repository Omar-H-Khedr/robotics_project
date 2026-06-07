#!/usr/bin/env python3
"""Analyze per-state XY clearance stability from passive wrench logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean


WRENCH_FILE = "wrench_state_samples.csv"
TRIAL_OUTCOME_FILE = "trial_outcome.json"
DEFAULT_STATE_LOOP_HZ = 10.0
CLEARANCE_THRESHOLDS_M = (0.001, 0.002)


@dataclass(frozen=True)
class Sample:
    stamp_s: float
    state: str
    xy_error_m: float
    peg_z_m: float
    force_norm_n: float


def _read_samples(path: Path) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                stamp_s = float(row["stamp_s"])
                xy_error_m = float(row["xy_error_m"])
                peg_z_m = float(row["peg_z_m"])
                force_norm_n = float(row["force_norm_n"])
            except (KeyError, ValueError):
                continue
            if math.isnan(xy_error_m):
                continue
            samples.append(
                Sample(
                    stamp_s=stamp_s,
                    state=row.get("state", "UNKNOWN") or "UNKNOWN",
                    xy_error_m=xy_error_m,
                    peg_z_m=peg_z_m,
                    force_norm_n=force_norm_n,
                )
            )
    return samples


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((percentile / 100.0) * (len(ordered) - 1)))
    return ordered[min(len(ordered) - 1, max(0, index))]


def _best_contiguous_window(samples: list[Sample], threshold_m: float) -> dict[str, object]:
    best: list[Sample] = []
    current: list[Sample] = []
    for sample in samples:
        if sample.xy_error_m <= threshold_m:
            current.append(sample)
        elif current:
            if len(current) > len(best):
                best = current
            current = []
    if len(current) > len(best):
        best = current

    if len(best) >= 2:
        duration_s = best[-1].stamp_s - best[0].stamp_s
    else:
        duration_s = 0.0

    return {
        "samples": len(best),
        "duration_s": duration_s,
        "start_s": best[0].stamp_s if best else None,
        "end_s": best[-1].stamp_s if best else None,
        "start_z_m": best[0].peg_z_m if best else None,
        "end_z_m": best[-1].peg_z_m if best else None,
    }


def _estimated_state_loop_window(
    samples: list[Sample],
    threshold_m: float,
    state_loop_hz: float,
) -> dict[str, object]:
    if not samples:
        return {
            "ticks": 0,
            "duration_s": 0.0,
            "start_s": None,
            "end_s": None,
            "start_z_m": None,
            "end_z_m": None,
        }

    tick_period_s = 1.0 / state_loop_hz
    next_tick_s = samples[0].stamp_s
    index = 0
    current_ticks = 0
    current_start_s: float | None = None
    current_start_z: float | None = None
    best_ticks = 0
    best_start_s: float | None = None
    best_end_s: float | None = None
    best_start_z: float | None = None
    best_end_z: float | None = None

    while next_tick_s <= samples[-1].stamp_s and index < len(samples):
        while index + 1 < len(samples) and samples[index].stamp_s < next_tick_s:
            index += 1
        sample = samples[index]
        if sample.xy_error_m <= threshold_m:
            if current_ticks == 0:
                current_start_s = next_tick_s
                current_start_z = sample.peg_z_m
            current_ticks += 1
            if current_ticks > best_ticks:
                best_ticks = current_ticks
                best_start_s = current_start_s
                best_end_s = next_tick_s
                best_start_z = current_start_z
                best_end_z = sample.peg_z_m
        else:
            current_ticks = 0
            current_start_s = None
            current_start_z = None
        next_tick_s += tick_period_s

    return {
        "ticks": best_ticks,
        "duration_s": best_ticks * tick_period_s,
        "start_s": best_start_s,
        "end_s": best_end_s,
        "start_z_m": best_start_z,
        "end_z_m": best_end_z,
    }


def _state_summary(
    state: str,
    samples: list[Sample],
    state_loop_hz: float,
) -> dict[str, object]:
    xy_values = [sample.xy_error_m for sample in samples]
    z_values = [sample.peg_z_m for sample in samples]
    force_values = [sample.force_norm_n for sample in samples]
    thresholds: dict[str, object] = {}
    for threshold in CLEARANCE_THRESHOLDS_M:
        key = f"{threshold:.3f}m"
        below = [sample for sample in samples if sample.xy_error_m <= threshold]
        thresholds[key] = {
            "samples_inside": len(below),
            "sample_fraction_inside": len(below) / len(samples) if samples else 0.0,
            "best_observer_window": _best_contiguous_window(samples, threshold),
            "best_estimated_state_loop_window": _estimated_state_loop_window(
                samples,
                threshold,
                state_loop_hz,
            ),
        }

    return {
        "state": state,
        "samples": len(samples),
        "start_s": samples[0].stamp_s if samples else None,
        "end_s": samples[-1].stamp_s if samples else None,
        "duration_s": samples[-1].stamp_s - samples[0].stamp_s if len(samples) >= 2 else 0.0,
        "min_xy_error_m": min(xy_values) if xy_values else None,
        "mean_xy_error_m": mean(xy_values) if xy_values else None,
        "p95_xy_error_m": _percentile(xy_values, 95.0),
        "final_xy_error_m": xy_values[-1] if xy_values else None,
        "min_peg_z_m": min(z_values) if z_values else None,
        "max_peg_z_m": max(z_values) if z_values else None,
        "max_force_norm_n": max(force_values) if force_values else None,
        "thresholds": thresholds,
    }


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


def analyze(input_dir: Path, state_loop_hz: float | None = None) -> dict[str, object]:
    samples_path = input_dir / WRENCH_FILE
    if not samples_path.exists():
        raise FileNotFoundError(f"missing {samples_path}")

    inferred_state_loop_hz = _infer_state_loop_hz(input_dir, state_loop_hz)
    samples = _read_samples(samples_path)
    by_state: dict[str, list[Sample]] = {}
    for sample in samples:
        by_state.setdefault(sample.state, []).append(sample)

    states = {
        state: _state_summary(state, state_samples, inferred_state_loop_hz)
        for state, state_samples in sorted(by_state.items())
    }
    return {
        "input_dir": str(input_dir),
        "sample_source": WRENCH_FILE,
        "state_loop_hz": inferred_state_loop_hz,
        "clearance_thresholds_m": list(CLEARANCE_THRESHOLDS_M),
        "total_valid_samples": len(samples),
        "states": states,
    }


def _fmt(value: object, digits: int = 6) -> str:
    if value is None:
        return "none"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def write_outputs(input_dir: Path, result: dict[str, object]) -> None:
    json_path = input_dir / "xy_stability_analysis.json"
    md_path = input_dir / "xy_stability_analysis.md"
    json_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    lines = [
        "# XY Stability Analysis",
        "",
        f"- input_dir: `{result['input_dir']}`",
        f"- sample_source: `{WRENCH_FILE}`",
        f"- total_valid_samples: `{result['total_valid_samples']}`",
        f"- state_loop_hz: `{float(result['state_loop_hz']):.1f}`",
        "",
        "| State | Samples | Min XY m | Mean XY m | P95 XY m | Final XY m | Z range m | Best 1mm ticks | Best 2mm ticks |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    states = result.get("states", {})
    if isinstance(states, dict):
        for state, raw_summary in states.items():
            if not isinstance(raw_summary, dict):
                continue
            thresholds = raw_summary.get("thresholds", {})
            one_mm_ticks = 0
            two_mm_ticks = 0
            if isinstance(thresholds, dict):
                one_mm = thresholds.get("0.001m", {})
                two_mm = thresholds.get("0.002m", {})
                if isinstance(one_mm, dict):
                    window = one_mm.get("best_estimated_state_loop_window", {})
                    one_mm_ticks = window.get("ticks", 0) if isinstance(window, dict) else 0
                if isinstance(two_mm, dict):
                    window = two_mm.get("best_estimated_state_loop_window", {})
                    two_mm_ticks = window.get("ticks", 0) if isinstance(window, dict) else 0
            z_range = (
                f"{_fmt(raw_summary.get('min_peg_z_m'))}..{_fmt(raw_summary.get('max_peg_z_m'))}"
            )
            lines.append(
                "| "
                + " | ".join(
                    [
                        str(state),
                        str(raw_summary.get("samples", 0)),
                        _fmt(raw_summary.get("min_xy_error_m")),
                        _fmt(raw_summary.get("mean_xy_error_m")),
                        _fmt(raw_summary.get("p95_xy_error_m")),
                        _fmt(raw_summary.get("final_xy_error_m")),
                        z_range,
                        str(one_mm_ticks),
                        str(two_mm_ticks),
                    ]
                )
                + " |"
            )

    lines.extend(
        [
            "",
            "Interpretation: this is an offline passive-log diagnostic. The state-loop "
            "window estimate samples the observer stream at the task controller "
            "cadence recorded in trial_outcome.json when available, or the explicit "
            "--state-loop-hz override. This keeps sustained-clearance tick evidence "
            "honest when running non-default cadence diagnostics.",
        ]
    )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze per-state XY clearance stability from wrench_state_samples.csv"
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
