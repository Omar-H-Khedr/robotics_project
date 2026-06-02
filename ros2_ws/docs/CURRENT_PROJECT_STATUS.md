# Current Project Status

Date: 2026-06-02

## Review Summary

The repository is an active ROS 2 Jazzy / Gazebo research workspace, not a blank future workspace. The current implementation has moved beyond proposal-only diagnostics into a controller-driven KUKA LBR iisy 6 R1300 Gazebo baseline. Controllers can activate and the task controller can command simulated motion.

The project must not claim final autonomous peg-in-hole success yet. The defensible claim is a first simulated insertion event with measured insertion depth. Repeated runtime validation is still required.

## Evidence Reviewed

- `README.md`
- `docs/IWIT_Expose-Template_v5.docx`
- `docs/context/proposal_context.md`
- `docs/context/proposal_full.md`
- `docs/context/robot_cell_audit.md`
- `docs/ROBOT_DATASHEET_CHECK.md`
- recent git history through `e210b5e`
- launch/config/model/task-control source files
- `admittance_insertion_node.py`
- `baseline_joint_sequence_executor.py`
- `spawn_robot_sdf.py`
- existing diagnostics under `diagnostics/` and `results/`

## Corrected Documentation Position

Do not use wording such as "first successful autonomous peg-in-hole" for the current state. Use:

**first simulated insertion event with measured insertion depth**

until repeated validation demonstrates robust success.

## Current Technical Baseline

- Robot: KUKA LBR iisy 6 R1300 project-local model adaptation.
- SAFE_HOME: `[0.0, -0.8, 1.2, 0.0, 0.8, 0.0]`.
- Spawn: x=0.80, y=-0.75, z=0.735, yaw=1.5708.
- Controller stack: `joint_state_broadcaster` and `joint_trajectory_controller`.
- Canonical `research_baseline.launch.py` uses `thesis_bringup/config/research_baseline_bridge.yaml` without a `/joint_states` Gazebo bridge. `joint_state_broadcaster` is the intended single `/joint_states` source.
- FT bridge target: `/ft_sensor_wrench`.
- Insertion controller: topic-based trajectory publishing with median Fz baseline, SEARCH phase, single-point INSERT, final JSON outcome logging.

## Immediate Fixes Applied In This Review Stage

- Fixed duplicate phase-result logging for degraded `MOVING_TO_START` and `APPROACH` timeout-with-grace paths.
- Added `experiment_manager.research_baseline_repeat_validator` to run fresh repeated Gazebo trials and collect normalized evidence.
- Updated README and task-control docs to state the insertion evidence honestly.
- Recreated the missing `docs/PROJECT_CONTEXT.md` with current project truth.

## Remaining Runtime Validation Requirement

Repeat validation was run after build:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run experiment_manager research_baseline_repeat_validator --trials 3 --timeout-s 220 --output-dir diagnostics/research_baseline_repeat_validation_v2
```

Outputs:

- `diagnostics/research_baseline_repeat_validation_v2/repeat_trials.csv`
- `diagnostics/research_baseline_repeat_validation_v2/summary.json`
- `diagnostics/research_baseline_repeat_validation_v2/summary.md`
- per-trial launch logs and outcome JSON files

Result: 0/3 physical successes.

| Trial | Outcome | Reason |
|---|---|---|
| 1 | DEGRADED | INSERT failed; depth 0.0037 m, peak raw Fz 1237.45 N |
| 2 | ABORTED | INSERT safety threshold exceeded; peak raw Fz 3716.2 N |
| 3 | NO_OUTCOME | Harness timeout during SEARCH before final outcome |

This confirms the baseline is not robust. It also confirms that the high-force concern is real and worse than the earlier 1049 N spike in at least one repeated run.

## Open Risks

- Non-deterministic MOVING_TO_START failure rate is not quantified by a fresh repeated run yet.
- Peak raw Fz spikes are confirmed: 1237.45 N and 3716.2 N were recorded in the 2026-06-01 repeat run.
- Large Cartesian errors during MOVING_TO_START and APPROACH remain unresolved.
- Multi-point INSERT is still not reliable.
- Contact/gravity baseline validity needs scenario-specific validation.
- Older `docs/context/robot_cell_audit.md` contains stale iisy3 statements and should be superseded by `docs/PROJECT_CONTEXT.md` plus `docs/ROBOT_DATASHEET_CHECK.md`.

## Next Milestone

`research_baseline_above_hole_tracking_stabilization`

Reason: force-safe insert stabilization blocked unsafe INSERT when peg Z was too high, but validation still failed. The 2026-06-01 force-safe validation (`diagnostics/research_baseline_force_safe_insert_v3`) showed:

| Trial | Outcome | Reason |
|---|---|---|
| 1 | NO_OUTCOME | Hard-force abort in SEARCH at raw Fz 1164.3 N, then harness timeout before final JSON |
| 2 | ABORTED | MOVING_TO_START timeout/degraded failure, max raw Fz 77.38 N |
| 3 | ABORTED | Hard-force abort in SEARCH at raw Fz 2990.77 N |

This means the next technical problem is not just INSERT. SEARCH/approach correction can generate unsafe force before insertion. The next milestone should move lateral alignment above the workpiece, verify no-contact XY convergence, then descend vertically only after XY is stable and peg Z reaches the force-safe precondition.

The no-contact alignment gate was then implemented and validated in `diagnostics/research_baseline_no_contact_alignment_v1`:

| Trial | Outcome | Reason |
|---|---|---|
| 1 | ABORTED | APPROACH blocked at above-hole XY error 0.1034 m |
| 2 | ABORTED | APPROACH blocked at above-hole XY error 0.0911 m |
| 3 | ABORTED | APPROACH blocked at above-hole XY error 0.0872 m |

This removed descent/SEARCH from these bad initial alignments and produced complete outcome JSON for all three trials. It did not solve task execution. The next milestone is above-hole tracking stabilization: improve the `MOVING_TO_START` target execution so the peg reaches the no-contact XY gate (`<=0.030 m`) before any descent is attempted.

An above-hole target-refresh experiment was run in `diagnostics/research_baseline_above_hole_tracking_v1`. It was not retained because it worsened safety: two of three trials hard-aborted in `MOVING_TO_START` with raw Fz spikes of 4086.95 N and 1766.64 N, and the remaining trial still failed the no-contact gate at 0.1116 m XY error.

## 2026-06-02 Joint-State Source Integrity

Milestone: `research_baseline_joint_state_source_integrity`

Evidence: `diagnostics/research_baseline_joint_state_source_integrity/summary.md`

The canonical baseline now avoids the shared KUKA Gazebo `/joint_states` bridge and uses a project-local bridge config for `/clock`, `/cmd_vel`, D405 topics, contact, and F/T only. Runtime evidence showed:

- headless Gazebo launched and spawned `lbr_iisy6_r1300`;
- `research_baseline_ros_gz_bridge` did not create a `/joint_states` bridge;
- `joint_state_broadcaster` and `joint_trajectory_controller` activated;
- `/joint_states` samples contained named joints `joint_1` through `joint_6`;
- `ros2 node info /joint_state_broadcaster` listed `/joint_states` as a publisher.

The trial remained a bounded failure, not a success: it timed out in `MOVING_TO_START` with late observed XY error 0.066 m, so descent remained blocked by the 0.002 m no-contact gate.

## 2026-06-02 Tracking Gain Audit

Milestone: `research_baseline_tracking_gain_audit`

Evidence: `diagnostics/research_baseline_tracking_gain_audit/summary.md`

Offline IK confirmed that `AXIS_ALIGN_POSE` is reachable from `SAFE_HOME` with near-zero FK residual, so the current failure is runtime tracking/physics/controller behavior rather than an unreachable Cartesian target.

Runtime tests showed:

- canonical gain 1000 can get near the target but oscillates/drifts, with best observed XY around 0.011 m before drifting back outside the gate;
- gain 250 was accepted by `gz_ros2_control` but was worse, reaching only about 0.023 m XY at 60 s and drifting to about 0.072 m;
- a bounded repeated joint-refinement experiment was rejected and removed because it increased joint error up to about 1.15 rad and left XY error around 0.14-0.23 m.

Retained implementation changes are diagnostic/configuration only: launch-time `position_gain` override and `MOVING_TO_START` joint-error logging. The no-contact descent gate remains 0.002 m.
