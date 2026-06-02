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
- `diagnostics/research_baseline_cell_model_consistency/summary.md`
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
- Canonical controller parameters: `thesis_bringup/config/research_baseline_ros2_control.yaml` loaded by `spawn_robot_sdf.py`.
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

## 2026-06-02 Cell Model Consistency

Milestone: `research_baseline_cell_model_consistency`

Evidence: `diagnostics/research_baseline_cell_model_consistency/summary.md`

The current model/configuration layer now consistently targets the KUKA LBR iisy 6 R1300 workcell:

- canonical robot metadata names `KUKA LBR iisy 6 R1300`;
- the deprecated cylinder robot is marked as a placeholder and must not be used for new baseline work;
- D405 perception topics match the canonical bridge topics;
- D405 world visual SDF syntax validates;
- task target Z values match the controller's `HOLE_TOP_Z=0.810` convention;
- the research gripper includes a fixed 25 mm peg and `peg_tip` frame;
- the optional robot-wrapper camera TF is disabled by default, because the Gazebo world owns the D405 sensor.

Validation passed for Xacro expansion, world SDF validation with local model path, robot URDF-to-SDF conversion, Python syntax checks, and targeted `colcon build`.

A 90 s headless launch spawned `lbr_iisy6_r1300`, started D405 and F/T bridges, loaded `gz_ros2_control`, and activated `joint_state_broadcaster` plus `joint_trajectory_controller`. The run still timed out in `MOVING_TO_START`; best observed XY error was about 0.027 m at 60 s, then drifted to about 0.070 m by 75 s. This is not insertion success and it keeps tracking/physics as the next blocker.

Remaining risk: Gazebo/DART still reports that KUKA mesh collision geometry could not be created. This is now a high-priority physics-credibility risk for the next milestone.

## 2026-06-02 Primitive Collision Geometry

Milestone: `research_baseline_primitive_collision_geometry`

Evidence: `diagnostics/research_baseline_primitive_collision_geometry/summary.md`

The canonical `lbr_iisy6_r1300_research_gripper.urdf.xacro` wrapper now requests primitive collision geometry from the project-local iisy6 macro. This keeps mesh visuals but gives Gazebo/DART cylinders/boxes for `base_link` through `link_6`, avoiding the previously observed KUKA arm mesh-collision rejection messages.

The iisy6 macro explicitly uses the existing iisy11 R1300 mesh assets for visual/default mesh compatibility because both are 1300 mm reach variants and the iisy6-specific mesh assets are not available in the local submodule state. Primitive collisions are used for the active Gazebo baseline.

Validation passed for Xacro expansion, URDF-to-SDF conversion, generated-URDF inspection, targeted `colcon build`, and a 90 s headless launch. The launch reached `MOVING_TO_START` completion once with `xy_error=0.0006 m`, then transitioned into `APPROACH`.

This is still not insertion success. `APPROACH` did not stabilize before timeout, with late approach Cartesian error around 0.039 m. The next blocker is approach/descent tracking stability after valid above-hole alignment.

## 2026-06-02 Strict Above-Hole Stability Gate

Milestone: `research_baseline_strict_above_hole_stability_gate`

Evidence: `diagnostics/research_baseline_strict_above_hole_stability_gate/summary.md`

The degraded `MOVING_TO_START` proceed path has been removed. The controller no longer descends from a single transient XY-good sample; it must satisfy the existing strict joint, Cartesian, and 2 mm XY gates for `STABILIZE_TICKS` before entering `APPROACH`.

Validation passed for Python syntax, targeted `colcon build`, and a 120 s headless launch. The launch aborted safely in `MOVING_TO_START` at 90 s:

- `cart_err=0.022 m`;
- `xy_err=0.018 m`;
- `joint_err=0.034 rad`;
- `stable=0/5`;
- `Outcome: ABORTED`;
- `Depth: 0.0000 m`;
- `Max Fz: 707.9 N`.

This is a safety improvement, not task success. The next blocker is stable above-hole tracking and high free-space F/T behavior before any descent, contact search, insertion, or learning milestone can be credible.

## 2026-06-02 Research Baseline ROS 2 Control Config

Milestone: `research_baseline_ros2_control_config`

Evidence: `diagnostics/research_baseline_ros2_control_config/summary.md`

The canonical `research_baseline.launch.py` now passes a project-local controller YAML into `spawn_robot_sdf.py` instead of relying on the upstream `kuka_resources/config/fake_hardware_config_6_axis.yaml` path. The upstream config remains the fallback for comparison, but the research launch owns its controller assumptions.

The research config uses:

- `controller_manager.update_rate: 250 Hz`;
- `joint_trajectory_controller.state_publish_rate: 100 Hz`;
- `joint_trajectory_controller.action_monitor_rate: 50 Hz`;
- `allow_nonzero_velocity_at_trajectory_end: false`.

Validation passed for Python syntax, targeted `thesis_bringup` build, and a 120 s headless launch. Runtime logs showed both `joint_state_broadcaster` and `joint_trajectory_controller` loaded `/home/omar/code/robotics_project/ros2_ws/install/thesis_bringup/share/thesis_bringup/config/research_baseline_ros2_control.yaml`. The controller update warning changed to a 0.004 s desired period, confirming the 250 Hz config was active.

The task outcome remained a bounded safety failure:

- `Outcome: ABORTED`;
- `Reason: MOVING_TO_START timeout/failure (90.0s)`;
- `cart_err=0.015 m`;
- `xy_err=0.011 m`;
- `joint_err=0.018 rad`;
- `stable=0/5`;
- `Depth: 0.0000 m`;
- `Max Fz: 171.1 N`.

This is not insertion success. The next blocker remains stable above-hole convergence/holding under the preserved strict no-contact gate. The next milestone should add commanded-versus-actual trajectory tracking evidence and then tune trajectory timing, hold behavior, or controller/physics parameters from measured tracking data.

## 2026-06-02 Trajectory Tracking Observer

Milestone: `research_baseline_trajectory_tracking_observer`

Evidence: `diagnostics/research_baseline_trajectory_tracking/summary.md`

The canonical launch now starts a passive `trajectory_tracking_observer` by default. It compares task-published `/joint_trajectory_controller/joint_trajectory` commands against named `/joint_states`, writes compact summaries, and does not publish commands. The direct `/joint_trajectory_controller/state` topic was discoverable in topic lists but did not deliver samples during validation, so the observer records that count separately and relies on command-vs-feedback tracking for evidence.

Validation passed for Python syntax, targeted `thesis_bringup` build, and a 150 s headless launch with `tracking_log_dir:=diagnostics/research_baseline_trajectory_tracking`.

The task outcome remained a bounded safety failure:

- `Outcome: ABORTED`;
- `Reason: MOVING_TO_START timeout/failure (90.0s)`;
- `cart_err=0.014 m`;
- `xy_err=0.010 m`;
- `joint_err=0.014 rad`;
- `stable=0/5`;
- `Depth: 0.0000 m`;
- `Max Fz: 168.6 N`.

Tracking summary:

- observed trajectory commands: 2;
- direct JTC state samples: 0;
- command-vs-joint-state samples: 15,893;
- max absolute position error: 0.045250 rad;
- mean max absolute position error: 0.015565 rad;
- p95 max absolute position error: 0.024368 rad;
- final max absolute position error: 0.014106 rad.

This evidence confirms the next blocker is stable final tracking/hold at the above-hole pose, not launch wiring. The next milestone should tune trajectory timing, final hold/stabilization behavior, and controller/physics parameters from measured tracking data without relaxing the 2 mm no-contact descent gate.
