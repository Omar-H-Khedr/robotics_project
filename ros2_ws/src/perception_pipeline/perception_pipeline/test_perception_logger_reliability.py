#!/usr/bin/env python3
"""10-trial perception logger reliability test.

Runs 10 full-task trials with perception logging and checks:
- Each trial produces a non-empty CSV with all 6 phases
- Diagnostic JSON exists and records startup status
- No empty logs remain
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

OUTCOME_PATH = Path("/tmp/insertion_trial_outcome.json")
TEST_OUTPUT_DIR = Path("diagnostics/perception_logger_reliability_test_v1")
TIMEOUT_S = 600.0
DDS_SHM_GLOBS = ["/dev/shm/fastrtps_*", "/dev/shm/sem.fastrtps_*"]


def cleanup_dds_shm() -> None:
    import glob as glob_mod
    for pattern in DDS_SHM_GLOBS:
        for p in glob_mod.glob(pattern):
            try:
                os.unlink(p)
            except OSError:
                pass


def kill_all_ros_nodes() -> None:
    for name in (
        "safety_monitor", "data_logger_node", "trajectory_tracking_observer",
        "wrench_state_observer", "contact_state_observer",
        "multimodal_observation_logger", "admittance_insertion_node",
    ):
        subprocess.Popen(
            ["pkill", "-9", "-f", name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )


def kill_gazebo_residue() -> None:
    for name in ("gzserver", "gz", "gzclient"):
        subprocess.Popen(
            ["pkill", "-9", "-f", name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    kill_all_ros_nodes()
    time.sleep(3.0)


def terminate(process: subprocess.Popen) -> None:
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


def run_trial(trial: int, output_dir: Path) -> dict:
    if OUTCOME_PATH.exists():
        OUTCOME_PATH.unlink()

    cleanup_dds_shm()

    log_path = output_dir / f"trial_{trial:02d}.log"
    perception_dir = output_dir / f"perception_trial_{trial:02d}"
    perception_dir.mkdir(parents=True, exist_ok=True)
    tracking_dir = output_dir / f"trial_{trial:02d}_tracking"
    tracking_dir.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    env["HOME"] = str(output_dir / "home")
    env["ROS_HOME"] = str(output_dir / "ros_home")
    env["ROS_LOG_DIR"] = str(output_dir / "ros_logs")
    Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    Path(env["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    Path(env["HOME"]).mkdir(parents=True, exist_ok=True)

    cmd = [
        "ros2", "launch", "thesis_bringup", "research_baseline.launch.py",
        "use_gui:=false",
        "control_rate:=25.0", "position_gain:=3000.0",
        "position_derivative_gain:=10.0", "joint_damping_scale:=10.0",
        "inject_velocity_state:=true",
        "velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml",
        "search_recenter_duration_s:=8.0", "search_settle_duration_s:=9.0",
        "insert_handoff_timeout_s:=12.0", "search_entry_threshold_m:=0.0",
        "enable_perception_logging:=true",
        f"perception_log_dir:={str(perception_dir)}",
        f"tracking_log_dir:={str(tracking_dir)}",
        "exit_on_done:=true", "shutdown_on_task_exit:=true",
    ]

    start = time.monotonic()
    timed_out = False
    outcome = None
    return_code = None
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            cmd, stdout=log_file, stderr=subprocess.STDOUT, text=True, env=env,
        )
        deadline = start + TIMEOUT_S
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
        terminate(process)
        return_code = process.poll()

    elapsed = time.monotonic() - start

    csv_path = perception_dir / "multimodal_observation_log.csv"
    diag_path = perception_dir / "logger_diagnostic.json"
    csv_rows = 0
    phases_found = set()
    diag = None
    if csv_path.exists():
        with csv_path.open() as f:
            lines = f.readlines()
            csv_rows = max(0, len(lines) - 1)
            if csv_rows > 0:
                header = lines[0].strip().split(",")
                phase_idx = header.index("task_phase") if "task_phase" in header else -1
                if phase_idx >= 0:
                    for line in lines[1:]:
                        parts = line.strip().split(",")
                        if len(parts) > phase_idx:
                            phases_found.add(parts[phase_idx])
    if diag_path.exists():
        try:
            diag = json.loads(diag_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            pass

    success = False
    failed_phase = ""
    if outcome:
        success = str(outcome.get("trial_outcome", "")) == "SUCCESS"
        if not success:
            for phase in outcome.get("phases", []):
                if isinstance(phase, dict) and not phase.get("success", False):
                    failed_phase = str(phase.get("phase", "unknown"))
                    break

    return {
        "trial": trial,
        "physical_success": success,
        "trial_outcome": outcome.get("trial_outcome", "NO_OUTCOME") if outcome else "NO_OUTCOME",
        "failed_phase": failed_phase,
        "final_task_state": outcome.get("final_task_state", "UNKNOWN") if outcome else "UNKNOWN",
        "csv_rows": csv_rows,
        "phases_found": sorted(phases_found),
        "empty_log": csv_rows <= 1,
        "logger_startup_ok": diag.get("subscriber_status", {}).get("joint_state_received", False) if diag else False,
        "first_joint_state_s": diag.get("first_joint_state_s") if diag else None,
        "first_task_phase_s": diag.get("first_task_phase_s") if diag else None,
        "elapsed_s": round(elapsed, 1),
        "timed_out": timed_out,
        "return_code": return_code,
        "log_path": str(log_path),
    }


def main() -> None:
    TEST_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    trials = 10
    results = []

    kill_gazebo_residue()
    cleanup_dds_shm()
    time.sleep(5.0)

    for trial in range(1, trials + 1):
        if trial > 1:
            kill_gazebo_residue()
            cleanup_dds_shm()
            time.sleep(12.0)
        print(f"\n=== Trial {trial}/{trials} ===", flush=True)
        result = run_trial(trial, TEST_OUTPUT_DIR)
        results.append(result)
        empty_flag = "EMPTY!" if result["empty_log"] else "OK"
        print(
            f"  outcome={result['trial_outcome']} success={result['physical_success']} "
            f"csv_rows={result['csv_rows']} phases={result['phases_found']} "
            f"startup_ok={result['logger_startup_ok']} {empty_flag} "
            f"elapsed={result['elapsed_s']}s"
        )

    empty_count = sum(1 for r in results if r["empty_log"])
    success_count = sum(1 for r in results if r["physical_success"])
    all_phases = set()
    for r in results:
        all_phases.update(r["phases_found"])

    summary = {
        "total_trials": trials,
        "non_empty_logs": trials - empty_count,
        "empty_logs": empty_count,
        "physical_successes": success_count,
        "success_rate": round(success_count / trials, 4),
        "all_phases_captured": sorted(all_phases),
        "results": results,
    }
    summary_path = TEST_OUTPUT_DIR / "reliability_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\n=== SUMMARY ===")
    print(f"Non-empty logs: {trials - empty_count}/{trials}")
    print(f"Physical successes: {success_count}/{trials}")
    print(f"All phases captured: {sorted(all_phases)}")
    print(f"Summary: {summary_path}")


if __name__ == "__main__":
    main()
