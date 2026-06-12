#!/usr/bin/env python3
"""Multi-scenario row-level context vector data collector.

Runs Gazebo trials with perception logging enabled, extracts 68-dim context
vectors, and saves them in a compact format. Designed for cross-scenario
v2_14 evaluation.

Usage:
    # Smoke test: 1 trial per scenario
    python3 collect_multi_scenario_row_data.py --stage smoke

    # Short: 3 trials per scenario (minimum for cross-scenario evaluation)
    python3 collect_multi_scenario_row_data.py --stage short

    # Full: 5 trials per scenario
    python3 collect_multi_scenario_row_data.py --stage full

    # Single scenario
    python3 collect_multi_scenario_row_data.py --scenario baseline_loose --trials 3
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parent.parent.parent.parent
SCRIPTS_DIR = WORKSPACE / "src" / "thesis_bringup" / "scripts"
EXTRACTOR_SCRIPT = WORKSPACE / "src" / "perception_pipeline" / "perception_pipeline" / "context_vector_extractor.py"

sys.path.insert(0, str(SCRIPTS_DIR))

STAGE_TRIAL_COUNTS = {
    "smoke": 1,
    "short": 3,
    "full": 5,
}

SCENARIOS = [
    "baseline_loose",
    "large_peg_large_hole",
    "small_peg_small_hole",
    "misaligned_baseline",
    "clearance_medium",
    "clearance_tight",
    "tight_plus_misaligned",
]


def run_trial_with_perception(
    scenario_id: str,
    trial_index: int,
    output_dir: Path,
    timeout_s: int = 600,
) -> dict:
    """Run a single Gazebo trial with perception logging enabled.

    Returns trial outcome dict.
    """
    trial_dir = output_dir / scenario_id / f"trial_{trial_index:03d}"
    trial_dir.mkdir(parents=True, exist_ok=True)

    # Build ros2 launch command with perception logging enabled
    launch_file = WORKSPACE / "src" / "thesis_bringup" / "launch" / "research_baseline.launch.py"

    cmd = [
        "bash", "-c",
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {WORKSPACE}/install/setup.bash && "
        f"ros2 launch {launch_file} "
        f"use_gui:=false "
        f"control_rate:=25.0 "
        f"position_gain:=3000.0 "
        f"position_derivative_gain:=10.0 "
        f"joint_damping_scale:=10.0 "
        f"inject_velocity_state:=true "
        f"velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml "
        f"search_recenter_duration_s:=8.0 "
        f"search_settle_duration_s:=9.0 "
        f"insert_handoff_timeout_s:=12.0 "
        f"search_entry_threshold_m:=0.0 "
        f"exit_on_done:=true "
        f"shutdown_on_task_exit:=true "
        f"scenario_id:={scenario_id} "
        f"tracking_log_dir:={trial_dir} "
        f"perception_log_dir:={trial_dir} "
        f"enable_perception_logging:=true "
    ]

    print(f"  Running trial {trial_index}: {scenario_id} (with perception logging)")

    start_time = time.time()
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            cwd=str(WORKSPACE),
        )
        duration_s = time.time() - start_time

        # Save stdout/stderr
        (trial_dir / "stdout.log").write_text(result.stdout)
        (trial_dir / "stderr.log").write_text(result.stderr)

        # Parse outcome
        outcome = _parse_outcome(result.stdout, result.stderr, duration_s)
        outcome["scenario_id"] = scenario_id
        outcome["trial_index"] = trial_index
        outcome["duration_s"] = duration_s

        # Save outcome
        (trial_dir / "trial_outcome.json").write_text(json.dumps(outcome, indent=2))

        return outcome

    except subprocess.TimeoutExpired:
        duration_s = timeout_s
        outcome = {
            "scenario_id": scenario_id,
            "trial_index": trial_index,
            "physical_success": False,
            "timeout": True,
            "duration_s": duration_s,
        }
        (trial_dir / "trial_outcome.json").write_text(json.dumps(outcome, indent=2))
        return outcome

    except Exception as e:
        duration_s = time.time() - start_time
        outcome = {
            "scenario_id": scenario_id,
            "trial_index": trial_index,
            "physical_success": False,
            "error": str(e),
            "duration_s": duration_s,
        }
        (trial_dir / "trial_outcome.json").write_text(json.dumps(outcome, indent=2))
        return outcome


def _parse_outcome(stdout: str, stderr: str, duration_s: float) -> dict:
    """Parse trial outcome from stdout/stderr."""
    outcome = {
        "physical_success": False,
        "search_entered": False,
        "search_converged": False,
        "contact_guided_insertion": False,
        "insertion_depth_m": 0.0,
        "final_xy_error_m": 0.0,
        "max_contact_force_n": 0.0,
        "timeout": False,
        "safety_abort": False,
    }

    for line in stdout.split("\n"):
        if "Task completed successfully" in line or "DONE" in line:
            outcome["physical_success"] = True
        if "SEARCH" in line and "entered" in line.lower():
            outcome["search_entered"] = True
        if "SEARCH" in line and "converged" in line.lower():
            outcome["search_converged"] = True
        if "insertion_depth" in line.lower():
            try:
                val = float(line.split("=")[-1].strip())
                outcome["insertion_depth_m"] = val
            except (ValueError, IndexError):
                pass
        if "ABORTED" in line or "safety_abort" in line.lower():
            outcome["safety_abort"] = True
        if "timeout" in line.lower() and "exceeded" in line.lower():
            outcome["timeout"] = True

    return outcome


def extract_context_vectors(trial_dir: Path) -> int:
    """Extract 68-dim context vectors from multimodal observation CSV.

    Returns number of context vectors extracted.
    """
    csv_path = trial_dir / "multimodal_observation_log.csv"
    if not csv_path.exists():
        print(f"    WARNING: No perception log at {csv_path}")
        return 0

    parquet_path = trial_dir / "context_vectors.parquet"

    try:
        result = subprocess.run(
            [
                "python3", str(EXTRACTOR_SCRIPT),
                "--input-csv", str(csv_path),
                "--output-parquet", str(parquet_path),
            ],
            capture_output=True,
            text=True,
            timeout=60,
            cwd=str(WORKSPACE),
        )

        if result.returncode == 0 and parquet_path.exists():
            import pandas as pd
            df = pd.read_parquet(parquet_path)
            n_rows = len(df)
            print(f"    Extracted {n_rows} context vectors -> {parquet_path.name}")
            return n_rows
        else:
            print(f"    WARNING: Extraction failed: {result.stderr[:200]}")
            return 0

    except Exception as e:
        print(f"    WARNING: Extraction error: {e}")
        return 0


def cleanup_raw_csv(trial_dir: Path):
    """Remove large raw CSV files to save disk space."""
    for csv_file in trial_dir.glob("*.csv"):
        if csv_file.name != "multimodal_observation_log.csv":
            continue
        size_mb = csv_file.stat().st_size / (1024 * 1024)
        csv_file.unlink()
        print(f"    Cleaned up {csv_file.name} ({size_mb:.1f}MB)")


def main():
    parser = argparse.ArgumentParser(description="Multi-scenario row-level data collector")
    parser.add_argument("--stage", choices=["smoke", "short", "full"], default="short",
                        help="Number of trials per scenario")
    parser.add_argument("--scenario", type=str, default=None,
                        help="Run single scenario only")
    parser.add_argument("--trials", type=int, default=None,
                        help="Override trial count")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory")
    parser.add_argument("--timeout", type=int, default=600,
                        help="Timeout per trial in seconds")
    args = parser.parse_args()

    # Determine trial count
    if args.trials is not None:
        n_trials = args.trials
    else:
        n_trials = STAGE_TRIAL_COUNTS[args.stage]

    # Determine scenarios
    if args.scenario:
        scenarios = [args.scenario]
    else:
        scenarios = SCENARIOS

    # Output directory
    output_dir = Path(args.output_dir) if args.output_dir else \
        WORKSPACE / "diagnostics" / "multi_scenario_row_level_data"

    print(f"=== Multi-Scenario Row-Level Data Collection ===")
    print(f"Stage: {args.stage} ({n_trials} trials/scenario)")
    print(f"Scenarios: {len(scenarios)}")
    print(f"Total trials: {len(scenarios) * n_trials}")
    print(f"Output: {output_dir}\n")

    # Check disk space
    total_bytes = 0
    for trial in range(n_trials):
        for scenario in scenarios:
            # Estimate ~646MB per trial for raw CSV, minus cleanup
            total_bytes += 50 * 1024 * 1024  # 50MB per trial after cleanup
    free_bytes = shutil.disk_usage(str(output_dir.parent)).free
    if total_bytes > free_bytes * 0.8:
        print(f"WARNING: Low disk space. Need ~{total_bytes/1e9:.1f}GB, have {free_bytes/1e9:.1f}GB")

    # Run trials
    all_outcomes = []
    total_vectors = 0

    for scenario_id in scenarios:
        print(f"\n--- Scenario: {scenario_id} ---")
        for trial_idx in range(n_trials):
            print(f"\n  Trial {trial_idx + 1}/{n_trials}:")

            # Run trial with perception logging
            outcome = run_trial_with_perception(
                scenario_id, trial_idx, output_dir, args.timeout
            )
            all_outcomes.append(outcome)

            # Extract context vectors
            trial_dir = output_dir / scenario_id / f"trial_{trial_idx:03d}"
            n_vectors = extract_context_vectors(trial_dir)
            total_vectors += n_vectors

            # Clean up raw CSV to save space
            cleanup_raw_csv(trial_dir)

            success = "SUCCESS" if outcome.get("physical_success") else "FAILED"
            print(f"    Result: {success} ({outcome.get('duration_s', 0):.1f}s)")

    # Save summary
    summary = {
        "total_trials": len(all_outcomes),
        "total_context_vectors": total_vectors,
        "scenarios": scenarios,
        "trials_per_scenario": n_trials,
        "outcomes": all_outcomes,
    }

    summary_path = output_dir / "collection_summary.json"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)

    # Print summary
    successes = sum(1 for o in all_outcomes if o.get("physical_success"))
    print(f"\n=== Collection Summary ===")
    print(f"Total trials: {len(all_outcomes)}")
    print(f"Successes: {successes}/{len(all_outcomes)} ({successes/len(all_outcomes)*100:.0f}%)")
    print(f"Total context vectors: {total_vectors}")
    print(f"Summary saved to: {summary_path}")


if __name__ == "__main__":
    main()
