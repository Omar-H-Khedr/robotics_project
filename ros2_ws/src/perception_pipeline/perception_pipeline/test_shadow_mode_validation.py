#!/usr/bin/env python3
"""v2_14 shadow-mode validation runner.

Runs 10 full-task trials with v2_14_shadow_mode_node enabled alongside the
deterministic controller.  Validates:
  - Shadow-mode inference node produces non-empty logs
  - Per-tick predictions cover all task phases
  - Safety-gating correctly defers on INSERT
  - Agreement rates vs ground truth
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
TEST_OUTPUT_DIR = Path("diagnostics/v2_14_shadow_mode_validation")
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
        "v2_14_shadow_mode_node", "live_v2_14_inference_node",
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
    shadow_dir = output_dir / f"shadow_trial_{trial:02d}"
    shadow_dir.mkdir(parents=True, exist_ok=True)

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
        "enable_v2_14_shadow_mode:=true",
        f"v2_14_shadow_output_dir:={str(shadow_dir)}",
        f"v2_14_shadow_model_path:={str(Path.cwd() / 'diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt')}",
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
    shadow_csv_path = shadow_dir / "shadow_v2_14_inference_log.csv"
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

    shadow_rows = 0
    shadow_phases = set()
    shadow_agreement_count = 0
    shadow_fallback_count = 0
    shadow_insert_defer_count = 0
    if shadow_csv_path.exists():
        with shadow_csv_path.open() as f:
            lines = f.readlines()
            shadow_rows = max(0, len(lines) - 1)
            if shadow_rows > 0:
                header = lines[0].strip().split(",")
                agreement_idx = header.index("agreement") if "agreement" in header else -1
                fallback_idx = header.index("use_fallback") if "use_fallback" in header else -1
                pred_phase_idx = header.index("pred_phase_name") if "pred_phase_name" in header else -1
                gt_phase_idx = header.index("gt_phase") if "gt_phase" in header else -1
                for line in lines[1:]:
                    parts = line.strip().split(",")
                    if pred_phase_idx >= 0 and len(parts) > pred_phase_idx:
                        shadow_phases.add(parts[pred_phase_idx])
                    if agreement_idx >= 0 and len(parts) > agreement_idx:
                        if parts[agreement_idx].strip() == "True":
                            shadow_agreement_count += 1
                    if fallback_idx >= 0 and len(parts) > fallback_idx:
                        if parts[fallback_idx].strip() == "True":
                            shadow_fallback_count += 1
                    if gt_phase_idx >= 0 and pred_phase_idx >= 0:
                        if len(parts) > gt_phase_idx and len(parts) > pred_phase_idx:
                            gt_p = parts[gt_phase_idx].strip()
                            pred_p = parts[pred_phase_idx].strip()
                            if gt_p == "INSERT" and pred_p == "INSERT":
                                if fallback_idx >= 0 and len(parts) > fallback_idx:
                                    if parts[fallback_idx].strip() == "True":
                                        shadow_insert_defer_count += 1

    success = False
    failed_phase = ""
    if outcome:
        success = str(outcome.get("trial_outcome", "")) == "SUCCESS"
        if not success:
            for phase in outcome.get("phases", []):
                if isinstance(phase, dict) and not phase.get("success", False):
                    failed_phase = str(phase.get("phase", "unknown"))
                    break

    agreement_rate = shadow_agreement_count / max(1, shadow_rows)
    fallback_rate = shadow_fallback_count / max(1, shadow_rows)

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
        "shadow_csv_rows": shadow_rows,
        "shadow_phases_found": sorted(shadow_phases),
        "shadow_agreement_count": shadow_agreement_count,
        "shadow_agreement_rate": round(agreement_rate, 4),
        "shadow_fallback_count": shadow_fallback_count,
        "shadow_fallback_rate": round(fallback_rate, 4),
        "shadow_insert_defer_count": shadow_insert_defer_count,
        "elapsed_s": round(elapsed, 1),
        "timed_out": timed_out,
        "return_code": return_code,
        "log_path": str(log_path),
    }


def main() -> None:
    TEST_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    trials = int(os.environ.get("VALIDATION_TRIALS", "10"))
    results = []

    kill_gazebo_residue()
    cleanup_dds_shm()
    time.sleep(5.0)

    for trial in range(1, trials + 1):
        if trial > 1:
            kill_gazebo_residue()
            wait_for_clean_state()
            time.sleep(15.0)
        print(f"\n=== Trial {trial}/{trials} ===", flush=True)
        result = run_trial(trial, TEST_OUTPUT_DIR)
        results.append(result)
        empty_flag = "EMPTY!" if result["empty_log"] else "OK"
        print(
            f"  outcome={result['trial_outcome']} success={result['physical_success']} "
            f"csv_rows={result['csv_rows']} phases={result['phases_found']} "
            f"shadow_rows={result['shadow_csv_rows']} "
            f"shadow_agreement={result['shadow_agreement_rate']:.1%} "
            f"shadow_fallback={result['shadow_fallback_rate']:.1%} "
            f"startup_ok={result['logger_startup_ok']} {empty_flag} "
            f"elapsed={result['elapsed_s']}s"
        )

    empty_count = sum(1 for r in results if r["empty_log"])
    success_count = sum(1 for r in results if r["physical_success"])
    all_phases = set()
    all_shadow_phases = set()
    total_shadow_rows = 0
    total_agreement = 0
    total_fallback = 0
    total_insert_defer = 0
    for r in results:
        all_phases.update(r["phases_found"])
        all_shadow_phases.update(r["shadow_phases_found"])
        total_shadow_rows += r["shadow_csv_rows"]
        total_agreement += r["shadow_agreement_count"]
        total_fallback += r["shadow_fallback_count"]
        total_insert_defer += r["shadow_insert_defer_count"]

    summary = {
        "total_trials": trials,
        "non_empty_logs": trials - empty_count,
        "empty_logs": empty_count,
        "physical_successes": success_count,
        "success_rate": round(success_count / trials, 4),
        "all_phases_captured": sorted(all_phases),
        "shadow_mode": {
            "total_shadow_rows": total_shadow_rows,
            "all_shadow_phases": sorted(all_shadow_phases),
            "overall_agreement_rate": round(total_agreement / max(1, total_shadow_rows), 4),
            "overall_fallback_rate": round(total_fallback / max(1, total_shadow_rows), 4),
            "total_insert_defer_count": total_insert_defer,
        },
        "results": results,
    }
    summary_path = TEST_OUTPUT_DIR / "shadow_validation_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"\n=== SUMMARY ===")
    print(f"Non-empty logs: {trials - empty_count}/{trials}")
    print(f"Physical successes: {success_count}/{trials}")
    print(f"All phases captured: {sorted(all_phases)}")
    print(f"Shadow mode agreement rate: {summary['shadow_mode']['overall_agreement_rate']:.1%}")
    print(f"Shadow mode fallback rate: {summary['shadow_mode']['overall_fallback_rate']:.1%}")
    print(f"Summary: {summary_path}")


if __name__ == "__main__":
    main()
