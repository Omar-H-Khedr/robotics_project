"""Repeat-run validator for the Gazebo research baseline.

This is intentionally a process-level harness: each trial starts a fresh
``research_baseline.launch.py`` process, waits for the insertion node to write
``/tmp/insertion_trial_outcome.json``, then records normalized evidence for
robustness claims.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


OUTCOME_PATH = Path("/tmp/insertion_trial_outcome.json")


def _phase_lookup(outcome: dict[str, Any], phase_name: str) -> dict[str, Any]:
    for phase in outcome.get("phases", []):
        if phase.get("phase") == phase_name:
            return phase
    return {}


def _max_cartesian_error(outcome: dict[str, Any]) -> float:
    values = [
        float(phase.get("cart_error_m", 0.0))
        for phase in outcome.get("phases", [])
        if isinstance(phase, dict)
    ]
    return max(values) if values else 0.0


def _failed_phase(outcome: dict[str, Any]) -> str:
    if str(outcome.get("trial_outcome", "")) == "SUCCESS":
        return ""
    for phase in outcome.get("phases", []):
        if isinstance(phase, dict) and not bool(phase.get("success", False)):
            return str(phase.get("phase", "unknown"))
    reason = str(outcome.get("reason", "")).lower()
    if reason.startswith("insert blocked"):
        return "INSERT_PRECONDITION"
    if "insert" in reason:
        return "INSERT"
    if "search" in reason:
        return "SEARCH"
    return ""


def _row_from_outcome(
    trial: int,
    return_code: int | None,
    elapsed_s: float,
    timed_out: bool,
    outcome: dict[str, Any] | None,
) -> dict[str, Any]:
    if not outcome:
        return {
            "trial": trial,
            "final_state": "NO_OUTCOME",
            "trial_outcome": "NO_OUTCOME",
            "success": False,
            "failed_phase": "launch_or_logging",
            "insertion_depth_m": 0.0,
            "final_xy_error_m": 0.0,
            "max_predepth_xy_error_m": 0.0,
            "peak_raw_fz_N": 0.0,
            "sustained_contact_force_N": 0.0,
            "max_insert_contact_force_N": 0.0,
            "insert_predepth_recenter_attempts": 0,
            "insert_predepth_recenter_budget_resets": 0,
            "insert_entry_descent_count": 0,
            "insert_capture_descent_count": 0,
            "insert_shallow_sideload_recovery_attempts": 0,
            "insert_final_sideload_retry_attempts": 0,
            "side_load_abort": False,
            "max_cartesian_error_m": 0.0,
            "timeout": timed_out,
            "safety_abort": False,
            "return_code": return_code,
            "elapsed_s": round(elapsed_s, 2),
            "reason": "No fresh /tmp/insertion_trial_outcome.json was written.",
        }

    metrics = outcome.get("metrics", {})
    trial_outcome = str(outcome.get("trial_outcome", "UNKNOWN"))
    reason = str(outcome.get("reason", ""))
    insert = _phase_lookup(outcome, "INSERT")
    failed_phase = _failed_phase(outcome)
    reason_lower = reason.lower()
    timeout = bool(timed_out or any(
        bool(phase.get("timed_out", False))
        for phase in outcome.get("phases", [])
        if isinstance(phase, dict)
    ))
    safety_abort = (
        "safety threshold" in reason_lower
        or "hard-force" in reason_lower
        or "hard force" in reason_lower
    )
    physical_success = (
        trial_outcome == "SUCCESS"
        and float(metrics.get("insertion_depth_m", 0.0)) >= 0.010
        and float(metrics.get("max_contact_force_N", 0.0)) >= float(
            metrics.get("contact_threshold_N", 5.0)
        )
        and not safety_abort
    )

    return {
        "trial": trial,
        "final_state": str(outcome.get("status", "")),
        "trial_outcome": trial_outcome,
        "success": physical_success,
        "failed_phase": failed_phase,
        "insertion_depth_m": float(metrics.get("insertion_depth_m", 0.0)),
        "final_xy_error_m": float(
            metrics.get("final_insertion_xy_error_m", 0.0)
        ),
        "max_predepth_xy_error_m": float(
            metrics.get("insert_max_predepth_xy_error_m", 0.0)
        ),
        "peak_raw_fz_N": float(metrics.get("max_fz_N", 0.0)),
        "sustained_contact_force_N": float(
            metrics.get("max_contact_force_N", insert.get("contact_force_N", 0.0))
        ),
        "max_insert_contact_force_N": float(
            metrics.get("max_insert_contact_force_N", 0.0)
        ),
        "insert_predepth_recenter_attempts": int(
            metrics.get("insert_predepth_recenter_attempts", 0)
        ),
        "insert_predepth_recenter_budget_resets": int(
            metrics.get("insert_predepth_recenter_budget_resets", 0)
        ),
        "insert_entry_descent_count": int(
            metrics.get("insert_entry_descent_count", 0)
        ),
        "insert_capture_descent_count": int(
            metrics.get("insert_capture_descent_count", 0)
        ),
        "insert_shallow_sideload_recovery_attempts": int(
            metrics.get("insert_shallow_sideload_recovery_attempts", 0)
        ),
        "insert_final_sideload_retry_attempts": int(
            metrics.get("insert_final_sideload_retry_attempts", 0)
        ),
        "side_load_abort": (
            trial_outcome == "ABORTED"
            and ("side-loaded" in reason_lower or "side-load" in reason_lower)
        ),
        "max_cartesian_error_m": _max_cartesian_error(outcome),
        "timeout": timeout,
        "safety_abort": safety_abort,
        "return_code": return_code,
        "elapsed_s": round(elapsed_s, 2),
        "reason": reason,
    }


def _terminate(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def _run_trial(trial: int, args: argparse.Namespace, output_dir: Path) -> dict[str, Any]:
    if OUTCOME_PATH.exists():
        OUTCOME_PATH.unlink()

    log_path = output_dir / f"trial_{trial:02d}.log"
    env = os.environ.copy()
    env["HOME"] = str(output_dir / "home")
    env["ROS_HOME"] = str(output_dir / "ros_home")
    env["ROS_LOG_DIR"] = str(output_dir / "ros_logs")
    Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    Path(env["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    Path(env["HOME"]).mkdir(parents=True, exist_ok=True)

    cmd = [
        "ros2",
        "launch",
        "thesis_bringup",
        "research_baseline.launch.py",
        "use_gui:=false",
    ]
    if args.extra_launch_arg:
        cmd.extend(args.extra_launch_arg)
    tracking_log_dir = ""
    if args.per_trial_tracking_logs and not any(
        arg.startswith("tracking_log_dir:=") for arg in args.extra_launch_arg
    ):
        tracking_path = output_dir / f"trial_{trial:02d}_tracking"
        tracking_path.mkdir(parents=True, exist_ok=True)
        tracking_log_dir = str(tracking_path)
        cmd.append(f"tracking_log_dir:={tracking_log_dir}")

    start = time.monotonic()
    timed_out = False
    outcome: dict[str, Any] | None = None
    return_code: int | None = None
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
        )
        deadline = start + args.timeout_s
        while time.monotonic() < deadline:
            return_code = process.poll()
            if OUTCOME_PATH.exists():
                try:
                    outcome = json.loads(OUTCOME_PATH.read_text(encoding="utf-8"))
                except json.JSONDecodeError:
                    outcome = None
                break
            if return_code is not None:
                break
            time.sleep(1.0)
        else:
            timed_out = True

        _terminate(process)
        return_code = process.poll()

    elapsed_s = time.monotonic() - start
    row = _row_from_outcome(trial, return_code, elapsed_s, timed_out, outcome)
    row["log_path"] = str(log_path)
    row["tracking_log_dir"] = tracking_log_dir
    if outcome is not None:
        (output_dir / f"trial_{trial:02d}_outcome.json").write_text(
            json.dumps(outcome, indent=2),
            encoding="utf-8",
        )
    return row


def _write_outputs(rows: list[dict[str, Any]], output_dir: Path, args: argparse.Namespace) -> None:
    csv_path = output_dir / "repeat_trials.csv"
    fieldnames = [
        "trial",
        "final_state",
        "trial_outcome",
        "success",
        "failed_phase",
        "insertion_depth_m",
        "final_xy_error_m",
        "max_predepth_xy_error_m",
        "peak_raw_fz_N",
        "sustained_contact_force_N",
        "max_insert_contact_force_N",
        "insert_predepth_recenter_attempts",
        "insert_predepth_recenter_budget_resets",
        "insert_entry_descent_count",
        "insert_capture_descent_count",
        "insert_shallow_sideload_recovery_attempts",
        "insert_final_sideload_retry_attempts",
        "side_load_abort",
        "max_cartesian_error_m",
        "timeout",
        "safety_abort",
        "return_code",
        "elapsed_s",
        "reason",
        "log_path",
        "tracking_log_dir",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    total = len(rows)
    successes = sum(1 for row in rows if row["success"])
    timeouts = sum(1 for row in rows if row["timeout"])
    safety_aborts = sum(1 for row in rows if row["safety_abort"])
    side_load_aborts = sum(1 for row in rows if row["side_load_abort"])
    summary = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "command": "ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false",
        "extra_launch_args": list(args.extra_launch_arg),
        "per_trial_tracking_logs": bool(args.per_trial_tracking_logs),
        "trials_requested": args.trials,
        "timeout_s": args.timeout_s,
        "trials_completed": total,
        "success_count": successes,
        "success_rate": round(successes / total, 4) if total else 0.0,
        "timeout_count": timeouts,
        "safety_abort_count": safety_aborts,
        "side_load_abort_count": side_load_aborts,
        "criteria": {
            "physical_success": (
                "trial_outcome == SUCCESS, insertion_depth_m >= 0.010, "
                "max_contact_force_N >= contact_threshold_N, and no safety abort"
            ),
            "robust_success_claim": (
                "requires repeated evidence; one successful insertion-depth event "
                "is not treated as final autonomous peg-in-hole success"
            ),
        },
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    (output_dir / "summary.md").write_text(
        "\n".join(
            [
                "# Research Baseline Repeat Validation",
                "",
                f"- Trials completed: {total}",
                f"- Physical successes: {successes}",
                f"- Success rate: {summary['success_rate']}",
                f"- Timeouts: {timeouts}",
                f"- Safety aborts: {safety_aborts}",
                f"- Side-load aborts: {side_load_aborts}",
                f"- CSV: `{csv_path}`",
                "",
                "Physical success requires measured insertion depth, contact evidence, "
                "and no safety abort. This report does not convert a single run into a "
                "robustness claim.",
                "",
                "| Trial | Outcome | Success | Failed phase | Depth m | Final XY m | "
                "Max pre-depth XY m | Peak raw Fz N | Contact N | Insert contact N | "
                "Pre-depth recenters | Recenter budget resets | Entry descents | "
                "Capture descents | Shallow side-load recoveries | Final side-load "
                "retries | Timeout | Safety abort | Side-load abort |",
                "| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |",
                *[
                    (
                        f"| {row['trial']} | `{row['trial_outcome']}` | "
                        f"`{row['success']}` | `{row['failed_phase']}` | "
                        f"{float(row['insertion_depth_m']):.4f} | "
                        f"{float(row['final_xy_error_m']):.4f} | "
                        f"{float(row['max_predepth_xy_error_m']):.4f} | "
                        f"{float(row['peak_raw_fz_N']):.2f} | "
                        f"{float(row['sustained_contact_force_N']):.2f} | "
                        f"{float(row['max_insert_contact_force_N']):.2f} | "
                        f"{row['insert_predepth_recenter_attempts']} | "
                        f"{row['insert_predepth_recenter_budget_resets']} | "
                        f"{row['insert_entry_descent_count']} | "
                        f"{row['insert_capture_descent_count']} | "
                        f"{row['insert_shallow_sideload_recovery_attempts']} | "
                        f"{row['insert_final_sideload_retry_attempts']} | "
                        f"`{row['timeout']}` | `{row['safety_abort']}` | "
                        f"`{row['side_load_abort']}` |"
                    )
                    for row in rows
                ],
                "",
            ]
        ),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--timeout-s", type=float, default=150.0)
    parser.add_argument(
        "--output-dir",
        default="diagnostics/research_baseline_repeat_validation",
    )
    parser.add_argument(
        "--extra-launch-arg",
        action="append",
        default=[],
        help="Additional launch argument, e.g. robot_model:=lbr_iisy6_r1300",
    )
    parser.add_argument(
        "--per-trial-tracking-logs",
        action="store_true",
        help=(
            "Append tracking_log_dir:=<output-dir>/trial_XX_tracking to each "
            "launch unless tracking_log_dir is already supplied."
        ),
    )
    args = parser.parse_args()

    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = [_run_trial(trial, args, output_dir) for trial in range(1, args.trials + 1)]
    _write_outputs(rows, output_dir, args)
    print(json.dumps({"output_dir": str(output_dir), "trials": len(rows)}, indent=2))


if __name__ == "__main__":
    main()
