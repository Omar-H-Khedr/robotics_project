#!/usr/bin/env python3
"""Analyze Stage C geometry matrix results.

Reads all completed trials, produces per-scenario and overall summaries.
"""
import json
import os
import re
import sys
from pathlib import Path


def parse_trial(trial_dir: str) -> dict:
    """Parse trial outcome from stdout.log or trial_outcome.json."""
    stdout_path = os.path.join(trial_dir, "stdout.log")
    result_path = os.path.join(trial_dir, "trial_result.json")
    outcome_path = os.path.join(trial_dir, "trial_outcome.json")

    if not os.path.exists(stdout_path) and not os.path.exists(outcome_path):
        return None

    # Try trial_outcome.json first (most reliable)
    outcome_data = None
    if os.path.exists(outcome_path):
        try:
            with open(outcome_path) as f:
                outcome_data = json.load(f)
        except (json.JSONDecodeError, OSError):
            outcome_data = None

    result_data = {}
    if os.path.exists(result_path):
        with open(result_path) as f:
            result_data = json.load(f)

    if outcome_data:
        # Parse from outcome JSON
        trial_outcome = outcome_data.get("trial_outcome", "")
        success = trial_outcome == "SUCCESS"
        metrics = outcome_data.get("metrics", {})
        search_entered = "SEARCH" in str(outcome_data.get("phases", []))
        search_converged = metrics.get("search_converged", False)
        contact_guided = metrics.get("contact_guided_insertion", False)
        insertion_depth = metrics.get("insertion_depth_m", 0.0)
        max_force = metrics.get("max_insert_contact_force_N", 0.0)
        final_xy = metrics.get("final_insertion_xy_error_m", 0.0)
        predepth_recenter = metrics.get("insert_predepth_recenter_attempts", 0)
        sideload_recovery = metrics.get("insert_shallow_sideload_recovery_attempts", 0)
        # Failure classification
        failure_phase = ""
        failure_reason = ""
        if not success:
            reason = outcome_data.get("reason", "")
            if "ABORT" in str(outcome_data.get("phases", [])):
                failure_phase = "ABORT"
            if "handoff settle timeout" in reason.lower():
                failure_reason = "handoff_settle_timeout"
            elif "SEARCH timeout" in reason or "recenter_budget" in reason:
                failure_reason = "search_timeout"
            elif "side-load" in reason.lower():
                failure_reason = "sideload_abort"
            elif "XY error" in reason and "exceeds" in reason:
                failure_reason = "xy_exceeded"
            elif failure_phase == "ABORT":
                failure_reason = "safety_abort"
            else:
                failure_reason = reason[:100] if reason else "unknown"
    else:
        # Fallback: parse from stdout
        with open(stdout_path) as f:
            stdout = f.read()

        phase_updates = re.findall(r'Task phase updated: (\w+)', stdout)
        success = False
        if phase_updates:
            last_phase = phase_updates[-1]
            if last_phase == "DONE":
                success = True

        search_entered = bool(re.search(r'Attempting search phase|SEARCH', stdout))
        search_converged = "SEARCH converged" in stdout
        contact_guided = "contact-guided insertion" in stdout.lower()

        depth_matches = re.findall(r'physical_depth=([0-9.]+)m', stdout)
        insertion_depth = max((float(d) for d in depth_matches), default=0.0)

        force_matches = re.findall(r'contact=([0-9.]+)N', stdout)
        max_force = max((float(f) for f in force_matches), default=0.0)

        xy_matches = re.findall(r'xy_error=([0-9.]+)m', stdout)
        final_xy = float(xy_matches[-1]) if xy_matches else 0.0

        recenter_matches = re.findall(r'predepth_recenter_attempts[=:]\s*(\d+)', stdout)
        predepth_recenter = max((int(r) for r in recenter_matches), default=0)

        sideload_matches = re.findall(r'sideload_recovery_attempts[=:]\s*(\d+)', stdout)
        sideload_recovery = max((int(s) for s in sideload_matches), default=0)

        failure_phase = ""
        failure_reason = ""
        if not success and phase_updates:
            last = phase_updates[-1]
            if last == "ABORT":
                failure_phase = "ABORT"
                if "handoff settle timeout" in stdout.lower():
                    failure_reason = "handoff_settle_timeout"
                elif "recenter_budget exhausted" in stdout or "SEARCH timeout" in stdout:
                    failure_reason = "search_timeout"
                elif "no-contact XY error" in stdout:
                    failure_reason = "no_contact_xy_exceeded"
                elif "side-load" in stdout.lower():
                    failure_reason = "sideload_abort"
                else:
                    failure_reason = "unknown_abort"
            elif last == "MOVING_TO_START":
                failure_phase = "MOVING_TO_START"
                failure_reason = "startup_stuck"
            elif last == "APPROACH":
                failure_phase = "APPROACH"
                failure_reason = "approach_stuck"
            else:
                failure_phase = last
                failure_reason = f"stuck_in_{last}"

    return {
        "success": success,
        "search_entered": search_entered,
        "search_converged": search_converged,
        "contact_guided_insertion": contact_guided,
        "insertion_depth_m": insertion_depth,
        "max_contact_force_n": max_force,
        "final_xy_error_m": final_xy,
        "predepth_recenter_attempts": predepth_recenter,
        "sideload_recovery_attempts": sideload_recovery,
        "failure_phase": failure_phase,
        "failure_reason": failure_reason,
        "duration_s": result_data.get("duration_s", 0.0),
    }


def analyze_scenario(scenario_dir: str, scenario_id: str) -> dict:
    """Analyze all trials for a scenario."""
    trials = []
    for i in range(100):
        trial_dir = os.path.join(scenario_dir, f"trial_{i:03d}")
        if not os.path.isdir(trial_dir):
            continue
        result = parse_trial(trial_dir)
        if result is not None:
            result["trial_index"] = i
            result["scenario_id"] = scenario_id
            trials.append(result)

    if not trials:
        return None

    n = len(trials)
    successes = sum(1 for t in trials if t["success"])
    search_entered = sum(1 for t in trials if t["search_entered"])
    search_converged = sum(1 for t in trials if t["search_converged"])
    contact_guided = sum(1 for t in trials if t.get("contact_guided_insertion", False))
    durations = [t["duration_s"] for t in trials if t["duration_s"] > 0]
    forces = [t["max_contact_force_n"] for t in trials]
    depths = [t["insertion_depth_m"] for t in trials if t["insertion_depth_m"] > 0]

    # Failure classification
    failure_counts = {}
    for t in trials:
        if not t["success"] and t["failure_reason"]:
            failure_counts[t["failure_reason"]] = failure_counts.get(t["failure_reason"], 0) + 1

    return {
        "scenario_id": scenario_id,
        "n_trials": n,
        "successes": successes,
        "success_rate": successes / n if n > 0 else 0,
        "search_entry_rate": search_entered / n if n > 0 else 0,
        "search_converge_rate": search_converged / n if n > 0 else 0,
        "contact_guided_count": contact_guided,
        "contact_guided_rate": contact_guided / n if n > 0 else 0,
        "avg_duration_s": sum(durations) / len(durations) if durations else 0,
        "max_force_n": max(forces) if forces else 0,
        "avg_force_n": sum(forces) / len(forces) if forces else 0,
        "max_depth_m": max(depths) if depths else 0,
        "failure_counts": failure_counts,
        "trials": trials,
    }


def main():
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/geometry_matrix_stagec"

    scenarios = [
        "baseline_loose",
        "clearance_medium",
        "clearance_tight",
        "large_peg_large_hole",
        "small_peg_small_hole",
        "misaligned_baseline",
        "tight_plus_misaligned",
    ]

    all_results = {}
    total_trials = 0
    total_successes = 0

    print(f"{'Scenario':<25} {'N':>3} {'OK':>3} {'Rate':>6} {'SEARCH':>7} {'Converge':>8} {'CG':>4} {'AvgDur':>7}")
    print("-" * 80)

    for scenario in scenarios:
        scenario_dir = os.path.join(output_dir, scenario)
        if not os.path.isdir(scenario_dir):
            print(f"{scenario:<25} {'--':>3}")
            continue

        result = analyze_scenario(scenario_dir, scenario)
        if result is None:
            print(f"{scenario:<25} {'0':>3}")
            continue

        all_results[scenario] = result
        total_trials += result["n_trials"]
        total_successes += result["successes"]

        print(
            f"{scenario:<25} {result['n_trials']:>3} {result['successes']:>3} "
            f"{result['success_rate']:>5.0%} {result['search_entry_rate']:>6.0%} "
            f"{result['search_converge_rate']:>7.0%} {result['contact_guided_count']:>4} "
            f"{result['avg_duration_s']:>6.0f}s"
        )

    print("-" * 80)
    rate = total_successes / total_trials if total_trials > 0 else 0
    total_cg = sum(r.get("contact_guided_count", 0) for r in all_results.values())
    print(f"{'TOTAL':<25} {total_trials:>3} {total_successes:>3} {rate:>5.0%} {'':>6} {'':>8} {total_cg:>4}")

    # Write JSON summary
    summary = {
        "total_trials": total_trials,
        "total_successes": total_successes,
        "overall_success_rate": rate,
        "scenarios": {
            k: {kk: vv for kk, vv in v.items() if kk != "trials"}
            for k, v in all_results.items()
        },
    }
    summary_path = os.path.join(output_dir, "stage_c_summary.json")
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nSummary written to {summary_path}")


if __name__ == "__main__":
    main()
