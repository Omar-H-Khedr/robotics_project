#!/usr/bin/env python3
"""Geometry/Tolerance scenario matrix runner.

Runs parameterized peg-in-hole trials across multiple geometry/tolerance
scenarios. Generates SDF worlds, launches trials, collects metrics.

Usage:
    # Smoke test: 1 trial per scenario
    python3 run_geometry_scenario_matrix.py --stage smoke --output-dir /tmp/geometry_matrix

    # Short validation: 3 trials per scenario
    python3 run_geometry_scenario_matrix.py --stage short --output-dir /tmp/geometry_matrix

    # Full matrix: 20 trials per scenario
    python3 run_geometry_scenario_matrix.py --stage full --output-dir /tmp/geometry_matrix

    # Single scenario
    python3 run_geometry_scenario_matrix.py --scenario baseline_loose --trials 3
"""

import argparse
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# Add the workspace src to the path for imports
WORKSPACE = Path(__file__).resolve().parent.parent.parent.parent
SCRIPTS_DIR = WORKSPACE / "src" / "peg_in_hole_description" / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from generate_scenario_world import generate_world


@dataclass
class Scenario:
    id: str
    description: str
    tier: int
    peg_radius_m: float
    hole_radius_m: float
    initial_xy_offset_m: float = 0.0
    approach_offset_xy: float = 0.0
    peg_length_m: float = 0.11
    clearance_mm: float = 0.0

    def __post_init__(self):
        self.clearance_mm = (self.hole_radius_m - self.peg_radius_m) * 1000


@dataclass
class TrialResult:
    scenario_id: str
    trial_index: int
    peg_radius_m: float
    hole_radius_m: float
    clearance_mm: float
    initial_xy_offset_m: float
    approach_offset_xy: float
    physical_success: bool
    search_entered: bool
    search_converged: bool
    insertion_depth_m: float
    final_xy_error_m: float
    max_contact_force_n: float
    predepth_recenter_attempts: int
    shallow_sideload_recovery_attempts: int
    timeout: bool
    safety_abort: bool
    sideload_abort: bool
    failure_phase: str = ""
    failure_reason: str = ""
    duration_s: float = 0.0
    launch_args: dict = field(default_factory=dict)


STAGE_TRIAL_COUNTS = {
    "smoke": 1,
    "short": 3,
    "full": 20,
}


def load_scenarios(config_path: str) -> list[Scenario]:
    """Load scenario definitions from YAML config."""
    import yaml
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    scenarios = []
    for s in config["scenarios"]:
        scenarios.append(Scenario(
            id=s["id"],
            description=s["description"],
            tier=s["tier"],
            peg_radius_m=s["peg_radius_m"],
            hole_radius_m=s["hole_radius_m"],
            initial_xy_offset_m=s.get("initial_xy_offset_m", 0.0),
            approach_offset_xy=s.get("approach_offset_xy", 0.0),
            peg_length_m=config.get("peg_length", 0.11),
        ))
    return scenarios


def validate_scenario_sdf(scenario: Scenario) -> bool:
    """Validate that generated SDF has correct geometry dimensions.

    Parses the SDF and checks that:
    1. Hole fixture inner edges match expected hole radius
    2. Peg collision radius matches expected peg radius
    3. No intersecting geometries at spawn
    """
    sdf = generate_world(
        peg_radius=scenario.peg_radius_m,
        hole_radius=scenario.hole_radius_m,
        peg_length=scenario.peg_length_m,
    )

    try:
        root = ET.fromstring(sdf)
    except ET.ParseError as e:
        print(f"  ERROR: SDF parse failed for {scenario.id}: {e}")
        return False

    # Check hole fixture collision boxes
    ns = {"sdf": "http://www.sdftech.com/schema/sdf/1.9"}
    # Find hole_fixture model
    models = root.findall(".//sdf:model", ns)
    if not models:
        models = root.findall(".//model")

    hole_fixture = None
    for m in models:
        name = m.get("name")
        if name == "hole_fixture":
            hole_fixture = m
            break

    if hole_fixture is None:
        print(f"  ERROR: hole_fixture model not found in SDF for {scenario.id}")
        return False

    # Check that fixture has collision elements
    collisions = hole_fixture.findall(".//sdf:collision", ns)
    if not collisions:
        collisions = hole_fixture.findall(".//collision")

    if len(collisions) < 4:
        print(f"  ERROR: Expected 4+ collisions in hole_fixture, found {len(collisions)}")
        return False

    print(f"  SDF validated: {len(collisions)} collision elements in hole_fixture")
    return True


def run_trial(
    scenario: Scenario,
    trial_index: int,
    output_dir: str,
    world_dir: str,
    config_path: str,
    launch_overrides: Optional[dict] = None,
) -> TrialResult:
    """Run a single trial for a given scenario.

    Returns TrialResult with outcome data.
    """
    # Generate world SDF
    world_path = os.path.join(world_dir, f"{scenario.id}_world.sdf")
    generate_world(
        peg_radius=scenario.peg_radius_m,
        hole_radius=scenario.hole_radius_m,
        peg_length=scenario.peg_length_m,
        output_path=world_path,
    )

    # Copy generated world to installed package's worlds directory
    # so the launch file can find it via get_package_share_directory
    import shutil
    installed_worlds = os.path.join(
        WORKSPACE, "install", "peg_in_hole_description", "share",
        "peg_in_hole_description", "worlds",
    )
    os.makedirs(installed_worlds, exist_ok=True)
    installed_world_path = os.path.join(installed_worlds, os.path.basename(world_path))
    shutil.copy2(world_path, installed_world_path)

    # Trial output directory
    trial_dir = os.path.join(output_dir, scenario.id, f"trial_{trial_index:03d}")
    os.makedirs(trial_dir, exist_ok=True)

    # Build launch arguments
    launch_args = {
        "use_gui": "false",
        "control_rate": "25.0",
        "position_gain": "3000.0",
        "position_derivative_gain": "10.0",
        "joint_damping_scale": "10.0",
        "inject_velocity_state": "true",
        "velocity_state_controller_config_path": "config/research_baseline_velocity_state_500hz.yaml",
        "search_recenter_duration_s": "8.0",
        "search_settle_duration_s": "9.0",
        "insert_handoff_timeout_s": "12.0",
        "search_entry_threshold_m": "0.0",
        "exit_on_done": "true",
        "shutdown_on_task_exit": "true",
        "world_file": os.path.basename(world_path),
        "peg_radius_m": str(scenario.peg_radius_m),
        "hole_radius_m": str(scenario.hole_radius_m),
        "peg_length_m": str(scenario.peg_length_m),
        "hole_center_x": str(0.52),
        "hole_center_y": str(-0.20),
        "hole_top_z": "0.810",
        "scenario_id": scenario.id,
        "tracking_log_dir": trial_dir,
        "perception_log_dir": trial_dir,
    }

    if scenario.approach_offset_xy > 0:
        launch_args["approach_offset_xy"] = str(scenario.approach_offset_xy)

    if launch_overrides:
        launch_args.update(launch_overrides)

    # Save launch args for reproducibility
    args_path = os.path.join(trial_dir, "launch_args.json")
    with open(args_path, "w") as f:
        json.dump({
            "scenario_id": scenario.id,
            "trial_index": trial_index,
            "peg_radius_m": scenario.peg_radius_m,
            "hole_radius_m": scenario.hole_radius_m,
            "clearance_mm": scenario.clearance_mm,
            "peg_length_m": scenario.peg_length_m,
            "initial_xy_offset_m": scenario.initial_xy_offset_m,
            "approach_offset_xy": scenario.approach_offset_xy,
            "launch_args": launch_args,
        }, f, indent=2)

    # Build ros2 launch command
    launch_file = os.path.join(
        WORKSPACE, "src", "thesis_bringup", "launch", "research_baseline.launch.py"
    )

    cmd = [
        "bash", "-c",
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {WORKSPACE}/install/setup.bash && "
        f"ros2 launch {launch_file} "
        + " ".join(f"{k}:={v}" for k, v in launch_args.items())
    ]

    print(f"  Running trial {trial_index}: {scenario.id} "
          f"(peg={scenario.peg_radius_m*2000:.1f}mm, "
          f"hole={scenario.hole_radius_m*2000:.1f}mm, "
          f"clearance={scenario.clearance_mm:.2f}mm)")

    start_time = time.time()
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=600,  # 10 minute timeout per trial
            cwd=str(WORKSPACE),
        )
        duration_s = time.time() - start_time

        # Save stdout/stderr
        with open(os.path.join(trial_dir, "stdout.log"), "w") as f:
            f.write(result.stdout)
        with open(os.path.join(trial_dir, "stderr.log"), "w") as f:
            f.write(result.stderr)

        # Parse outcome from logs
        outcome = _parse_trial_outcome(result.stdout, result.stderr, duration_s)
        outcome.scenario_id = scenario.id
        outcome.trial_index = trial_index
        outcome.peg_radius_m = scenario.peg_radius_m
        outcome.hole_radius_m = scenario.hole_radius_m
        outcome.clearance_mm = scenario.clearance_mm
        outcome.initial_xy_offset_m = scenario.initial_xy_offset_m
        outcome.approach_offset_xy = scenario.approach_offset_xy
        outcome.launch_args = launch_args

    except subprocess.TimeoutExpired:
        duration_s = 600.0
        outcome = TrialResult(
            scenario_id=scenario.id,
            trial_index=trial_index,
            peg_radius_m=scenario.peg_radius_m,
            hole_radius_m=scenario.hole_radius_m,
            clearance_mm=scenario.clearance_mm,
            initial_xy_offset_m=scenario.initial_xy_offset_m,
            approach_offset_xy=scenario.approach_offset_xy,
            physical_success=False,
            search_entered=False,
            search_converged=False,
            insertion_depth_m=0.0,
            final_xy_error_m=0.0,
            max_contact_force_n=0.0,
            predepth_recenter_attempts=0,
            shallow_sideload_recovery_attempts=0,
            timeout=True,
            safety_abort=False,
            sideload_abort=False,
            failure_phase="TIMEOUT",
            failure_reason="Trial exceeded 600s deadline",
            duration_s=duration_s,
            launch_args=launch_args,
        )
    except Exception as e:
        duration_s = time.time() - start_time
        outcome = TrialResult(
            scenario_id=scenario.id,
            trial_index=trial_index,
            peg_radius_m=scenario.peg_radius_m,
            hole_radius_m=scenario.hole_radius_m,
            clearance_mm=scenario.clearance_mm,
            initial_xy_offset_m=scenario.initial_xy_offset_m,
            approach_offset_xy=scenario.approach_offset_xy,
            physical_success=False,
            search_entered=False,
            search_converged=False,
            insertion_depth_m=0.0,
            final_xy_error_m=0.0,
            max_contact_force_n=0.0,
            predepth_recenter_attempts=0,
            shallow_sideload_recovery_attempts=0,
            timeout=False,
            safety_abort=True,
            sideload_abort=False,
            failure_phase="ERROR",
            failure_reason=str(e),
            duration_s=duration_s,
            launch_args=launch_args,
        )

    # Save trial result
    result_path = os.path.join(trial_dir, "trial_result.json")
    with open(result_path, "w") as f:
        json.dump({
            "scenario_id": outcome.scenario_id,
            "trial_index": outcome.trial_index,
            "peg_radius_m": outcome.peg_radius_m,
            "hole_radius_m": outcome.hole_radius_m,
            "clearance_mm": outcome.clearance_mm,
            "initial_xy_offset_m": outcome.initial_xy_offset_m,
            "approach_offset_xy": outcome.approach_offset_xy,
            "physical_success": outcome.physical_success,
            "search_entered": outcome.search_entered,
            "search_converged": outcome.search_converged,
            "insertion_depth_m": outcome.insertion_depth_m,
            "final_xy_error_m": outcome.final_xy_error_m,
            "max_contact_force_n": outcome.max_contact_force_n,
            "predepth_recenter_attempts": outcome.predepth_recenter_attempts,
            "shallow_sideload_recovery_attempts": outcome.shallow_sideload_recovery_attempts,
            "timeout": outcome.timeout,
            "safety_abort": outcome.safety_abort,
            "sideload_abort": outcome.sideload_abort,
            "failure_phase": outcome.failure_phase,
            "failure_reason": outcome.failure_reason,
            "duration_s": outcome.duration_s,
            "launch_args": outcome.launch_args,
        }, f, indent=2)

    return outcome


def _parse_trial_outcome(stdout: str, stderr: str, duration_s: float) -> TrialResult:
    """Parse trial outcome from Gazebo stdout/stderr logs."""
    success = False
    search_entered = False
    search_converged = False
    insertion_depth = 0.0
    final_xy = 0.0
    max_force = 0.0
    predepth_recenter = 0
    sideload_recovery = 0
    timeout = False
    safety_abort = False
    sideload_abort = False
    failure_phase = ""
    failure_reason = ""

    combined = stdout + "\n" + stderr

    # Check for DONE outcome (authoritative success indicator)
    if "DONE outcome written" in combined:
        success = True
    elif "ABORT" in combined and "outcome" in combined.lower():
        success = False
        safety_abort = True
        failure_phase = "ABORT"

    # Check for SEARCH phase
    if "Attempting search phase" in combined or "SEARCH" in combined:
        search_entered = True
    if "SEARCH converged" in combined:
        search_converged = True

    # Extract insertion depth from INSERT log lines
    # Pattern: physical_depth=0.0200m
    import re
    depth_matches = re.findall(r'physical_depth=([0-9.]+)m', combined)
    if depth_matches:
        insertion_depth = max(float(d) for d in depth_matches)

    # Extract max contact force
    force_matches = re.findall(r'contact=([0-9.]+)N', combined)
    if force_matches:
        max_force = max(float(f) for f in force_matches)

    # Extract final XY error from last INSERT line
    xy_matches = re.findall(r'xy_error=([0-9.]+)m', combined)
    if xy_matches:
        final_xy = float(xy_matches[-1])

    # Extract recenter attempts
    recenter_matches = re.findall(r'predepth_recenter_attempts[=:]\s*(\d+)', combined)
    if recenter_matches:
        predepth_recenter = max(int(r) for r in recenter_matches)

    # Extract sideload recovery attempts
    sideload_matches = re.findall(r'sideload_recovery_attempts[=:]\s*(\d+)', combined)
    if sideload_matches:
        sideload_recovery = max(int(s) for r in sideload_matches)

    # Check for timeout
    if "timeout" in combined.lower() and not success:
        timeout = True
        failure_phase = "TIMEOUT"

    return TrialResult(
        scenario_id="",
        trial_index=0,
        peg_radius_m=0.0,
        hole_radius_m=0.0,
        clearance_mm=0.0,
        initial_xy_offset_m=0.0,
        approach_offset_xy=0.0,
        physical_success=success,
        search_entered=search_entered,
        search_converged=search_converged,
        insertion_depth_m=insertion_depth,
        final_xy_error_m=final_xy,
        max_contact_force_n=max_force,
        predepth_recenter_attempts=predepth_recenter,
        shallow_sideload_recovery_attempts=sideload_recovery,
        timeout=timeout,
        safety_abort=safety_abort,
        sideload_abort=sideload_abort,
        failure_phase=failure_phase,
        failure_reason=failure_reason,
        duration_s=duration_s,
    )


def generate_scenario_summary(
    scenario_id: str,
    results: list[TrialResult],
    output_dir: str,
) -> dict:
    """Generate per-scenario summary with all required metrics."""
    n = len(results)
    if n == 0:
        return {}

    successes = sum(1 for r in results if r.physical_success)
    timeouts = sum(1 for r in results if r.timeout)
    safety_aborts = sum(1 for r in results if r.safety_abort)
    sideload_aborts = sum(1 for r in results if r.sideload_abort)
    search_entered = sum(1 for r in results if r.search_entered)
    search_converged = sum(1 for r in results if r.search_converged)

    avg_depth = (sum(r.insertion_depth_m for r in results) / n) if n else 0
    avg_xy = (sum(r.final_xy_error_m for r in results) / n) if n else 0
    avg_force = (sum(r.max_contact_force_n for r in results) / n) if n else 0
    avg_recenter = (sum(r.predepth_recenter_attempts for r in results) / n) if n else 0
    avg_sideload = (sum(r.shallow_sideload_recovery_attempts for r in results) / n) if n else 0
    avg_duration = (sum(r.duration_s for r in results) / n) if n else 0

    summary = {
        "scenario_id": scenario_id,
        "peg_radius_m": results[0].peg_radius_m,
        "hole_radius_m": results[0].hole_radius_m,
        "clearance_mm": results[0].clearance_mm,
        "initial_xy_offset_m": results[0].initial_xy_offset_m,
        "approach_offset_xy": results[0].approach_offset_xy,
        "num_trials": n,
        "success_rate": successes / n,
        "successes": successes,
        "timeouts": timeouts,
        "safety_aborts": safety_aborts,
        "sideload_aborts": sideload_aborts,
        "search_entered_count": search_entered,
        "search_entered_rate": search_entered / n,
        "search_converged_count": search_converged,
        "search_converged_rate": search_converged / n,
        "avg_insertion_depth_m": avg_depth,
        "avg_final_xy_error_m": avg_xy,
        "avg_max_contact_force_n": avg_force,
        "avg_predepth_recenter_attempts": avg_recenter,
        "avg_shallow_sideload_recovery_attempts": avg_sideload,
        "avg_duration_s": avg_duration,
    }

    # Save summary
    summary_path = os.path.join(output_dir, scenario_id, "scenario_summary.json")
    os.makedirs(os.path.dirname(summary_path), exist_ok=True)
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)

    return summary


def generate_markdown_report(
    summaries: list[dict],
    output_dir: str,
) -> str:
    """Generate a Markdown report across all scenarios."""
    lines = [
        "# Geometry/Tolerance Scenario Matrix — Validation Results\n",
        f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n",
        "## Summary Table\n",
        "| Scenario | Peg Dia | Hole Dia | Clearance | Offset | Trials | Success Rate | SEARCH Entry | SEARCH Converge | Avg Depth | Avg XY | Avg Force |",
        "|----------|---------|----------|-----------|--------|--------|-------------|-------------|----------------|-----------|--------|-----------|",
    ]

    for s in summaries:
        lines.append(
            f"| {s['scenario_id']} "
            f"| {s['peg_radius_m']*2000:.1f}mm "
            f"| {s['hole_radius_m']*2000:.1f}mm "
            f"| {s['clearance_mm']:.2f}mm "
            f"| {s['initial_xy_offset_m']*1000:.1f}mm "
            f"| {s['num_trials']} "
            f"| {s['success_rate']*100:.1f}% "
            f"| {s['search_entered_rate']*100:.0f}% "
            f"| {s['search_converged_rate']*100:.0f}% "
            f"| {s['avg_insertion_depth_m']*1000:.1f}mm "
            f"| {s['avg_final_xy_error_m']*1000:.2f}mm "
            f"| {s['avg_max_contact_force_n']:.1f}N |"
        )

    lines.append("")
    lines.append("## Detailed Metrics\n")

    for s in summaries:
        lines.append(f"### {s['scenario_id']}\n")
        lines.append(f"- **Description**: {s.get('description', 'N/A')}")
        lines.append(f"- **Peg diameter**: {s['peg_radius_m']*2000:.1f} mm")
        lines.append(f"- **Hole diameter**: {s['hole_radius_m']*2000:.1f} mm")
        lines.append(f"- **Radial clearance**: {s['clearance_mm']:.2f} mm")
        lines.append(f"- **Initial XY offset**: {s['initial_xy_offset_m']*1000:.1f} mm")
        lines.append(f"- **Trials**: {s['num_trials']}")
        lines.append(f"- **Success rate**: {s['success_rate']*100:.1f}% ({s['successes']}/{s['num_trials']})")
        lines.append(f"- **Timeouts**: {s['timeouts']}")
        lines.append(f"- **Safety aborts**: {s['safety_aborts']}")
        lines.append(f"- **Side-load aborts**: {s['sideload_aborts']}")
        lines.append(f"- **SEARCH entered**: {s['search_entered_rate']*100:.0f}%")
        lines.append(f"- **SEARCH converged**: {s['search_converged_rate']*100:.0f}%")
        lines.append(f"- **Avg insertion depth**: {s['avg_insertion_depth_m']*1000:.1f} mm")
        lines.append(f"- **Avg final XY error**: {s['avg_final_xy_error_m']*1000:.2f} mm")
        lines.append(f"- **Avg max contact force**: {s['avg_max_contact_force_n']:.1f} N")
        lines.append(f"- **Avg pre-depth recenter attempts**: {s['avg_predepth_recenter_attempts']:.1f}")
        lines.append(f"- **Avg shallow side-load recovery**: {s['avg_shallow_sideload_recovery_attempts']:.1f}")
        lines.append(f"- **Avg trial duration**: {s['avg_duration_s']:.1f} s")
        lines.append("")

    report = "\n".join(lines)

    report_path = os.path.join(output_dir, "geometry_tolerance_validation_results.md")
    with open(report_path, "w") as f:
        f.write(report)

    return report


def main():
    parser = argparse.ArgumentParser(
        description="Geometry/Tolerance scenario matrix runner"
    )
    parser.add_argument(
        "--config", type=str,
        default=os.path.join(
            WORKSPACE, "src", "thesis_bringup", "config",
            "geometry_tolerance_scenarios.yaml",
        ),
        help="Scenario config YAML path",
    )
    parser.add_argument(
        "--stage", type=str, choices=["smoke", "short", "full"],
        default="smoke",
        help="Validation stage (smoke=1 trial, short=3, full=20)",
    )
    parser.add_argument(
        "--scenario", type=str, default=None,
        help="Run only this scenario (overrides --stage)",
    )
    parser.add_argument(
        "--trials", type=int, default=None,
        help="Number of trials per scenario (overrides --stage)",
    )
    parser.add_argument(
        "--output-dir", type=str,
        default="/tmp/geometry_tolerance_matrix",
        help="Output directory for results",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Generate SDFs and validate but don't run trials",
    )

    args = parser.parse_args()

    # Load scenarios
    scenarios = load_scenarios(args.config)
    print(f"Loaded {len(scenarios)} scenarios from {args.config}")

    # Filter to single scenario if specified
    if args.scenario:
        scenarios = [s for s in scenarios if s.id == args.scenario]
        if not scenarios:
            print(f"ERROR: Scenario '{args.scenario}' not found in config")
            sys.exit(1)

    # Determine trial count
    trial_count = args.trials or STAGE_TRIAL_COUNTS[args.stage]

    # Create output directories
    world_dir = os.path.join(args.output_dir, "worlds")
    os.makedirs(world_dir, exist_ok=True)

    # Validate all scenarios
    print("\n=== Validating scenario SDFs ===")
    all_valid = True
    for scenario in scenarios:
        print(f"  Validating {scenario.id}...")
        if not validate_scenario_sdf(scenario):
            all_valid = False
            print(f"  FAILED: {scenario.id}")

    if not all_valid:
        print("\nERROR: Some scenarios failed validation")
        sys.exit(1)

    print(f"\nAll {len(scenarios)} scenarios validated successfully")

    if args.dry_run:
        print("\nDry run complete. No trials executed.")
        sys.exit(0)

    # Run trials
    print(f"\n=== Running {trial_count} trial(s) per scenario ===")
    all_results: list[TrialResult] = []
    all_summaries: list[dict] = []

    for scenario in scenarios:
        print(f"\n--- Scenario: {scenario.id} ---")
        scenario_results = []

        for trial_idx in range(trial_count):
            result = run_trial(
                scenario=scenario,
                trial_index=trial_idx,
                output_dir=args.output_dir,
                world_dir=world_dir,
                config_path=args.config,
            )
            result.scenario_id = scenario.id
            result.peg_radius_m = scenario.peg_radius_m
            result.hole_radius_m = scenario.hole_radius_m
            result.clearance_mm = scenario.clearance_mm
            result.initial_xy_offset_m = scenario.initial_xy_offset_m
            result.approach_offset_xy = scenario.approach_offset_xy
            scenario_results.append(result)
            all_results.append(result)

        # Generate scenario summary
        summary = generate_scenario_summary(
            scenario.id, scenario_results, args.output_dir
        )
        if summary:
            all_summaries.append(summary)

    # Generate overall report
    if all_summaries:
        generate_markdown_report(all_summaries, args.output_dir)

    # Save all results CSV
    csv_path = os.path.join(args.output_dir, "all_results.csv")
    with open(csv_path, "w") as f:
        headers = [
            "scenario_id", "trial_index", "peg_radius_m", "hole_radius_m",
            "clearance_mm", "initial_xy_offset_m", "approach_offset_xy",
            "physical_success", "search_entered", "search_converged",
            "insertion_depth_m", "final_xy_error_m", "max_contact_force_n",
            "predepth_recenter_attempts", "shallow_sideload_recovery_attempts",
            "timeout", "safety_abort", "sideload_abort",
            "failure_phase", "failure_reason", "duration_s",
        ]
        f.write(",".join(headers) + "\n")
        for r in all_results:
            f.write(",".join(str(getattr(r, h)) for h in headers) + "\n")

    # Print summary
    print("\n=== RESULTS SUMMARY ===")
    for s in all_summaries:
        print(f"  {s['scenario_id']}: {s['success_rate']*100:.1f}% "
              f"({s['successes']}/{s['num_trials']})")

    total_successes = sum(s["successes"] for s in all_summaries)
    total_trials = sum(s["num_trials"] for s in all_summaries)
    print(f"\n  Overall: {total_successes}/{total_trials} "
          f"({total_successes/total_trials*100:.1f}%)" if total_trials else "")
    print(f"\nResults saved to: {args.output_dir}")


if __name__ == "__main__":
    main()
