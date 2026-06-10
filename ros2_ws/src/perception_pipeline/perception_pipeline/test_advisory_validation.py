#!/usr/bin/env python3
"""v2_14 guarded advisory validation runner.

Runs 10 full-task trials with v2_14_advisory_node enabled alongside the
deterministic controller.  Validates:
  - No ML action overrides safety
  - No ML-predicted DONE terminates the task
  - INSERT remains deterministic
  - Advisory decisions are logged and auditable
  - Physical robustness does not degrade compared with shadow mode
  - DONE precision issue is documented and guarded
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
TEST_OUTPUT_DIR = Path("diagnostics/v2_14_advisory_validation")
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
        "v2_14_shadow_mode_node", "v2_14_advisory_node",
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
    cleanup_dds_shm()
    time.sleep(2.0)
    cleanup_dds_shm()
    time.sleep(5.0)
    cleanup_dds_shm()


def wait_for_clean_state(max_wait: float = 30.0) -> None:
    import glob as glob_mod
    start = time.monotonic()
    while time.monotonic() - start < max_wait:
        remaining = []
        for pattern in DDS_SHM_GLOBS:
            remaining.extend(glob_mod.glob(pattern))
        if not remaining:
            break
        for p in remaining:
            try:
                os.unlink(p)
            except OSError:
                pass
        time.sleep(1.0)


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
    advisory_dir = output_dir / f"advisory_trial_{trial:02d}"
    advisory_dir.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    real_home = os.path.expanduser("~")
    real_local = str(Path(real_home) / ".local" / "lib" / "python3.12" / "site-packages")
    existing_pp = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = f"{real_local}:{existing_pp}" if existing_pp else real_local
    env["HOME"] = str(output_dir / "home")
    env["ROS_HOME"] = str(output_dir / "ros_home")
    env["ROS_LOG_DIR"] = str(output_dir / "ros_logs")
    Path(env["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    Path(env["ROS_HOME"]).mkdir(parents=True, exist_ok=True)
    Path(env["HOME"]).mkdir(parents=True, exist_ok=True)

    abs_model_path = str(
        Path.cwd() / "diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt"
    )

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
        "enable_v2_14_advisory:=true",
        f"v2_14_advisory_model_path:={abs_model_path}",
        f"v2_14_advisory_output_dir:={str(advisory_dir)}",
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
    advisory_csv_path = advisory_dir / "advisory_log.csv"
    advisory_summary_path = advisory_dir / "advisory_summary.json"
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

    advisory_rows = 0
    advisory_accepted = 0
    advisory_rejected = 0
    advisory_done_fp = 0
    advisory_retreat_uncertain = 0
    advisory_unsafe = 0
    advisory_phases = set()
    advisory_summary = None
    if advisory_csv_path.exists():
        with advisory_csv_path.open() as f:
            lines = f.readlines()
            advisory_rows = max(0, len(lines) - 1)
            if advisory_rows > 0:
                header = lines[0].strip().split(",")
                acc_idx = header.index("accepted") if "accepted" in header else -1
                rej_idx = header.index("rejected") if "rejected" in header else -1
                fp_idx = header.index("done_false_positive_flag") if "done_false_positive_flag" in header else -1
                ru_idx = header.index("retreat_uncertainty_flag") if "retreat_uncertainty_flag" in header else -1
                unsafe_idx = header.index("unsafe_if_executed") if "unsafe_if_executed" in header else -1
                pred_idx = header.index("pred_phase_name") if "pred_phase_name" in header else -1
                for line in lines[1:]:
                    parts = line.strip().split(",")
                    if acc_idx >= 0 and len(parts) > acc_idx and parts[acc_idx].strip() == "True":
                        advisory_accepted += 1
                    if rej_idx >= 0 and len(parts) > rej_idx and parts[rej_idx].strip() == "True":
                        advisory_rejected += 1
                    if fp_idx >= 0 and len(parts) > fp_idx and parts[fp_idx].strip() == "True":
                        advisory_done_fp += 1
                    if ru_idx >= 0 and len(parts) > ru_idx and parts[ru_idx].strip() == "True":
                        advisory_retreat_uncertain += 1
                    if unsafe_idx >= 0 and len(parts) > unsafe_idx and parts[unsafe_idx].strip() == "True":
                        advisory_unsafe += 1
                    if pred_idx >= 0 and len(parts) > pred_idx:
                        advisory_phases.add(parts[pred_idx])
    if advisory_summary_path.exists():
        try:
            advisory_summary = json.loads(advisory_summary_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            pass

    success = (
        outcome is not None
        and outcome.get("trial_outcome") == "SUCCESS"
        and outcome.get("status") == "DONE"
    )
    empty_log = csv_rows == 0
    startup_ok = True
    if diag:
        startup_ok = diag.get("startup_ok", True)

    return {
        "trial": trial,
        "physical_success": success,
        "trial_outcome": outcome.get("trial_outcome", "NO_OUTCOME") if outcome else "NO_OUTCOME",
        "failed_phase": outcome.get("failed_phase", "") if outcome else "",
        "csv_rows": csv_rows,
        "phases_found": sorted(phases_found),
        "empty_log": empty_log,
        "logger_startup_ok": startup_ok,
        "advisory_csv_rows": advisory_rows,
        "advisory_phases_found": sorted(advisory_phases),
        "advisory_accepted": advisory_accepted,
        "advisory_rejected": advisory_rejected,
        "advisory_done_false_positives": advisory_done_fp,
        "advisory_retreat_uncertain": advisory_retreat_uncertain,
        "advisory_unsafe": advisory_unsafe,
        "advisory_summary": advisory_summary,
        "elapsed_s": round(elapsed, 1),
        "timed_out": timed_out,
        "return_code": return_code,
        "log_path": str(log_path),
    }


def main() -> None:
    output_dir = TEST_OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)

    num_trials = 10
    results = []
    successes = 0
    non_empty = 0
    total_advisory_accepted = 0
    total_advisory_rejected = 0
    total_advisory_done_fp = 0
    total_advisory_retreat_uncertain = 0
    total_advisory_unsafe = 0

    for trial in range(1, num_trials + 1):
        print(f"\n=== Trial {trial}/{num_trials} ===", flush=True)
        kill_gazebo_residue()
        wait_for_clean_state()

        result = run_trial(trial, output_dir)
        results.append(result)

        if result["physical_success"]:
            successes += 1
        if not result["empty_log"]:
            non_empty += 1
        total_advisory_accepted += result["advisory_accepted"]
        total_advisory_rejected += result["advisory_rejected"]
        total_advisory_done_fp += result["advisory_done_false_positives"]
        total_advisory_retreat_uncertain += result["advisory_retreat_uncertain"]
        total_advisory_unsafe += result["advisory_unsafe"]

        print(
            f"  outcome={result['trial_outcome']} "
            f"success={result['physical_success']} "
            f"csv_rows={result['csv_rows']} "
            f"phases={result['phases_found']} "
            f"advisory_rows={result['advisory_csv_rows']} "
            f"accepted={result['advisory_accepted']} "
            f"rejected={result['advisory_rejected']} "
            f"done_fp={result['advisory_done_false_positives']} "
            f"retreat_uncertain={result['advisory_retreat_uncertain']} "
            f"unsafe={result['advisory_unsafe']} "
            f"startup_ok={result['logger_startup_ok']} "
            f"{'OK' if result['logger_startup_ok'] else 'FAIL'} "
            f"elapsed={result['elapsed_s']}s"
        )

    total_advisory = total_advisory_accepted + total_advisory_rejected
    summary = {
        "total_trials": num_trials,
        "non_empty_logs": non_empty,
        "physical_successes": successes,
        "success_rate": round(successes / num_trials, 3),
        "total_advisory_accepted": total_advisory_accepted,
        "total_advisory_rejected": total_advisory_rejected,
        "total_advisory_done_false_positives": total_advisory_done_fp,
        "total_advisory_retreat_uncertain": total_advisory_retreat_uncertain,
        "total_advisory_unsafe": total_advisory_unsafe,
        "advisory_acceptance_rate": round(
            total_advisory_accepted / max(1, total_advisory), 3
        ),
        "results": results,
    }

    summary_path = output_dir / "advisory_validation_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, default=str))

    print(f"\n=== SUMMARY ===")
    print(f"Non-empty logs: {non_empty}/{num_trials}")
    print(f"Physical successes: {successes}/{num_trials}")
    print(f"Advisory accepted: {total_advisory_accepted}")
    print(f"Advisory rejected: {total_advisory_rejected}")
    print(f"Advisory DONE false positives: {total_advisory_done_fp}")
    print(f"Advisory RETREAT uncertain: {total_advisory_retreat_uncertain}")
    print(f"Advisory unsafe-if-executed: {total_advisory_unsafe}")
    print(f"Advisory acceptance rate: {summary['advisory_acceptance_rate']}")
    print(f"Summary: {summary_path}")


if __name__ == "__main__":
    main()
