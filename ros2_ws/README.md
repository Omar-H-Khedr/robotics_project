# ROS 2 Jazzy / Gazebo Peg-in-Hole Research Workspace

Current status as of 2026-06-02: this is an active ROS 2 Jazzy workspace for a Gazebo-based KUKA LBR iisy 6 R1300 peg-in-hole research baseline. The project has a working robot spawn path, active ros2_control controllers, a fixed grasped peg model, a fixed hole fixture, force/torque bridge plumbing, contact observability, and an admittance-style insertion controller.

The strongest historical insertion evidence remains a **single simulated insertion-depth event**: measured insertion depth about 0.011 m with sustained contact around 142.9 N. This is not robust autonomous peg-in-hole success. The current safer iisy6 baseline reaches the strict above-hole XY gate with an axis-aligned vertical peg, then aborts honestly in `APPROACH` because the 67 mm descent is not tracked. Known unresolved concerns include high raw F/T spikes, large approach tracking errors, broken multi-point INSERT behavior, and failed repeated validation.

Latest tracking evidence localizes the approach/descent blocker to `joint_2`: normal, slow-descent, and high-gain diagnostics all leave `joint_2` about 0.107-0.111 rad from target, keeping the peg tip about 67-70 mm above the commanded touch pose. This supports a controller/physics/joint-authority investigation before any learning or insertion-claim work.

## Milestones

| Milestone | Status |
| --- | --- |
| proposal_simulation_cell_v1_2_rgbd_image_bridge_fix | Completed |
| proposal_simulation_cell_v1_3_contact_physics_validation | Completed |
| proposal_simulation_cell_v1_5_safety_virtual_force_interface | Completed |
| proposal_simulation_cell_v1_6_safety_gate_readiness | Completed |
| proposal_simulation_cell_v1_7_pre_control_contract | Completed |
| proposal_simulation_cell_v1_8_control_development_scaffold | Completed |
| proposal_simulation_cell_v1_9_no_motion_control_law_dry_run | Completed |
| proposal_simulation_cell_v1_10_experiment_configuration_matrix | Completed |
| proposal_simulation_cell_v1_11_single_scenario_loader_validation | Completed |
| proposal_simulation_cell_v1_12_scenario_batch_selector | Completed |
| proposal_simulation_cell_v1_13_batch_execution_plan_validator | Completed |
| proposal_simulation_cell_v1_14_batch_dry_run_orchestrator | Completed |
| proposal_simulation_cell_v1_15_evidence_package_generator | Completed |
| proposal_simulation_cell_v1_16_reproducibility_checklist | Completed |
| proposal_simulation_cell_v1_17_release_documentation_index | Completed |
| proposal_simulation_cell_v2_0_first_gazebo_motion_smoke_test | Completed |
| proposal_simulation_cell_v2_1_gazebo_motion_validation_suite | Completed |
| proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation | Completed |
| proposal_simulation_cell_v2_3_moveit_model_alignment_and_plan_only_validation | Completed |
| proposal_simulation_cell_v2_4_moveit_gazebo_execution_validation | Completed |
| proposal_simulation_cell_v2_5_guarded_pre_contact_task_sequence | Completed |
| proposal_simulation_cell_v2_6_contact_gated_guarded_approach_validation | Completed |
| proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_calibration | Completed |
| proposal_simulation_cell_v2_8_contact_reachability_and_trigger_validation | Completed |
| proposal_simulation_cell_v2_9_non_overlapping_approach_to_contact_validation | Completed |
| proposal_simulation_cell_v2_10_misalignment_contact_gate_batch_validation | Completed |
| proposal_simulation_cell_v2_11_multimodal_contact_observation_logging | Completed |
| proposal_simulation_cell_v2_12_context_vector_extraction | Completed |
| proposal_simulation_cell_v2_13_context_encoder_prototype | Completed |
| proposal_simulation_cell_v2_14_context_conditioned_guarded_action_validation | Completed |
| proposal_simulation_cell_v2_15_context_action_ablation_validation | Completed |
| proposal_simulation_cell_v2_16_guarded_peg_in_hole_objective_validation | Completed (no peg insertion) |
| research_baseline_v0_1_lbr_iisy6_r1300_end_to_end_fixes | Completed |
| research_baseline_v0_2_camera_visual_size_fix | Completed |
| admittance_controller_v2_honest_tracking_and_contact_estimation | Implemented; first insertion-depth event observed; repeat validation pending |
| research_baseline_repeat_validation | Failed: 0/3 physical successes in 2026-06-01 v2 repeat run |
| research_baseline_move_to_start_hold_correction | Rejected: transient good XY samples but no stable gate |
| research_baseline_raw_wrench_abort | Completed: raw wrench spikes now latched and abort active motion |
| research_baseline_contact_wrench_correlation | Completed: no canonical contact-topic messages during raw wrench abort |
| research_baseline_ft_mount_effort_limit | Completed: F/T mount limit corrected; raw spike reduced but still aborts safely |
| research_baseline_contact_bridge_full_paths | Completed: full-path contact bridge shows peg-target contact during MOVING_TO_START |
| research_baseline_axis_aligned_start_pose | Completed: vertical peg start pose removes raw-force abort; still times out at strict stability gate |
| research_baseline_search_fail_closed_v2 | Completed: 120 s axis-aligned start reaches strict 2 mm XY gate; failed APPROACH now aborts before SEARCH |
| research_baseline_slow_approach_descent_v1 | Rejected: 41.7 s descent still stalls near joint_2 with about 0.070 m Cartesian error |
| research_baseline_approach_gain_3000_v1 | Rejected: gain 3000 worsens APPROACH to about 0.073 m Cartesian error and higher raw wrench |
| research_baseline_joint2_approach_tracking_diagnostic | Completed: reusable analyzer confirms joint_2 dominates missing descent across recent approach runs |
| research_baseline_joint_damping_scale_0p2_v1 | Rejected: broad damping reduction triggers hard-force abort in MOVING_TO_START |
| research_baseline_joint_effort_scale_2p0_v1 | Rejected: doubled effort reaches APPROACH but immediately hard-aborts on unsafe wrench/contact |
| research_baseline_contact_pair_attribution_v1 | Completed: unsafe doubled-effort reproduction attributed MOVING_TO_START contact to link_5 versus target plate |
| research_baseline_tool_tip_frame_correction_v1 | Completed: corrected peg-tip frame removes reproduced link_5 target-plate clearance collision; still no insertion |

## 2026-06-02 Joint 2 Approach Tracking Diagnostic

Milestone: `research_baseline_joint2_approach_tracking_diagnostic`

The workspace now includes `approach_tracking_analyzer`, an offline diagnostic
tool for the passive trajectory observer CSVs. It reads named-joint command and
feedback data, computes per-joint error statistics for the approach command,
and maps final feedback through the local iisy6 peg-tip kinematics.

Validation commands:

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/approach_tracking_analyzer.py
colcon build --symlink-install --packages-select thesis_bringup
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_search_fail_closed_v2
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_slow_approach_descent_v1
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_approach_gain_3000_v1
```

Evidence:

- `diagnostics/research_baseline_search_fail_closed_v2/approach_tracking_analysis.md`
- `diagnostics/research_baseline_slow_approach_descent_v1/approach_tracking_analysis.md`
- `diagnostics/research_baseline_approach_gain_3000_v1/approach_tracking_analysis.md`

Result: all three recent approach variants fail through the same dominant
tracking signature, not through a bad Cartesian command. `joint_2` has p95
absolute approach error `0.108733 rad` in the fail-closed run, `0.106741 rad`
in the slow-descent run, and `0.110990 rad` in the high-gain run. Final peg-tip
feedback remains near `z=0.897-0.900 m` while the command target is
`z=0.830 m`.

Current conclusion: the next milestone should investigate iisy6 joint dynamics,
effort/damping assumptions, and Gazebo position-control authority around
`joint_2`. Safety gates remain correct; do not loosen the 2 mm no-contact gate,
approach Z preconditions, or hard-force abort to mask this failure.

## 2026-06-02 Joint Damping Scale 0.2 Diagnostic

Milestone: `research_baseline_joint_damping_scale_0p2_v1`

`spawn_robot_sdf` now exposes diagnostic-only launch arguments for converted
SDF joint dynamics:

- `joint_damping_scale`, default `1.0`
- `joint_effort_scale`, default `1.0`

The canonical default preserves the converted robot model. The first diagnostic
run used `joint_damping_scale:=0.2` with default effort limits, default position
gain, and unchanged safety gates.

Validation command:

```bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  joint_damping_scale:=0.2 \
  tracking_log_dir:=diagnostics/research_baseline_joint_damping_scale_0p2_v1
```

Evidence: `diagnostics/research_baseline_joint_damping_scale_0p2_v1/summary.md`

Result: rejected. The run aborted in `MOVING_TO_START` before descent with
`|Fz|=1181.04 N`, force norm `1272.74 N`, Cartesian error `0.272028 m`, and
zero insertion depth. Broad damping reduction did not solve the approach
blocker and is not a credible canonical change.

## 2026-06-02 Joint Effort Scale 2.0 Diagnostic

Milestone: `research_baseline_joint_effort_scale_2p0_v1`

The second dynamics diagnostic used `joint_effort_scale:=2.0` with canonical
damping, default position gain, and unchanged task safety gates.

Validation command:

```bash
timeout 260s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  joint_effort_scale:=2.0 \
  tracking_log_dir:=diagnostics/research_baseline_joint_effort_scale_2p0_v1
```

Evidence: `diagnostics/research_baseline_joint_effort_scale_2p0_v1/summary.md`

Result: rejected. The run reached `MOVING_TO_START` in `81.4 s` with
`initial_xy_error=0.0007 m`, then hard-aborted in `APPROACH` after `0.5 s` with
`|Fz|=968.4 N`, force norm `1009.7 N`, total max force norm `1043.0 N`, and
zero insertion depth. Doubling effort improves authority enough to begin
descent, but it immediately creates unsafe force/contact evidence and is not a
credible canonical setting. Command-index approach analysis showed the peg was
still at `z=0.890982 m` against the `z=0.830000 m` target when the abort was
triggered, with `joint_2` still `0.099473 rad` from the final target.

## 2026-06-02 Contact Pair Attribution Diagnostic

Milestone: `research_baseline_contact_pair_attribution_v1`

`contact_state_observer` now records exact Gazebo collision pairs in the contact
CSV and summary. This is passive instrumentation only; it does not change
motion, contact handling, or safety gates.

Validation command:

```bash
timeout 260s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  joint_effort_scale:=2.0 \
  tracking_log_dir:=diagnostics/research_baseline_contact_pair_attribution_v1
```

Evidence: `diagnostics/research_baseline_contact_pair_attribution_v1/summary.md`

Result: the reproduced doubled-effort run aborted safely in
`MOVING_TO_START`, before approach, at raw `|Fz|=1018.9 N` and force norm
`1195.8 N`. Contact attribution showed the target-source contact was
`lbr_iisy6_r1300::link_5::link_5_collision <-> target_plate::plate_link::target_plate_collision`,
with no peg-source or hole-source rows. This is robot-link clearance contact
with the target plate, not valid peg insertion contact.

Current conclusion: the next milestone is clearance-aware motion/geometry
validation for `MOVING_TO_START` and the target fixture. The no-contact descent
gate and global hard-force abort remain correct and must not be loosened to hide
this failure.

## 2026-06-02 Tool Tip Frame Correction

Milestone: `research_baseline_tool_tip_frame_correction_v1`

Contact-pair attribution showed the modeled wrist was colliding with the target
plate before descent. The root cause was the research gripper peg-tip frame:
`peg_tip` was at the near-palm end of the 110 mm peg, so the controller drove
the wrist down to put that near-palm point over the hole. The gripper model now
places the peg and fingers on the negative local tool-Z side, with `peg_tip` and
`gripper_tcp` at `z=-0.130 m`; `RobotKinematics` now uses the matching
`link_6 -> peg_tip` offset.

Validation passed for Python syntax, xacro expansion, targeted `colcon build`,
offline clearance analysis, and a canonical 240 s headless launch.

Evidence: `diagnostics/research_baseline_tool_tip_frame_correction_v1/summary.md`

Runtime result: `ABORTED` in `MOVING_TO_START`, not insertion success. The run
timed out at 120 s with final phase Cartesian error `0.015238 m`, joint error
`0.022126 rad`, and XY error about `0.014 m`. It recorded zero contact-topic
samples, max raw `|Fz|=172.83 N`, and max raw force norm `270.82 N`. Offline
clearance analysis found `0/201` planned and `0/1338` runtime-feedback
`link_5` target-plate intersections.

Current conclusion: the clearance collision has been removed. The immediate
blocker is now final above-hole XY stabilization under the preserved 2 mm
no-contact gate.

## research_baseline_v0_1_lbr_iisy6_r1300_end_to_end_fixes

Status: `end_to_end_motion_validated`

The research baseline v0.1 sprint fixes three critical issues in the Phase 2B unified research baseline and validates end-to-end Gazebo motion with the correct KUKA LBR iisy 6 R1300 robot model.

### Three Fixes

**Fix 1 — Correct robot model: lbr_iisy3_r760 → lbr_iisy6_r1300**
The baseline was configured for the wrong KUKA model (3 kg payload, shorter reach). Switched to the correct 6 kg model in:
- `thesis_bringup/config/research_baseline.yaml`: `robot_name` → `KUKA LBR iisy 6 R1300`
- `thesis_bringup/launch/research_baseline.launch.py`: `RESEARCH_ROBOT_XACRO` → `lbr_iisy6_r1300_research_gripper.urdf.xacro`, `robot_model` default → `lbr_iisy6_r1300`
- Added new URDF xacro: `peg_in_hole_description/urdf/lbr_iisy6_r1300_research_gripper.urdf.xacro`

**Fix 2 — Corrected robot spawn position and initial joint injection**
Previously used `ros_gz_sim create` with no way to pass initial joint positions. Replaced with custom `spawn_robot_sdf` node that processes the xacro with `initial_joint_N` arguments, baking in the SAFE_HOME pose at spawn time. Also:
- Spawn z adjusted from `0.75` → `0.735` to match the pedestal top_plate surface
- Cartesian target heights in `kuka_task_control/config/peg_hole_cartesian_targets.yaml` lowered by −0.035 m to compensate

**Fix 3 — Sequential launch ordering with event handlers**
All nodes previously launched simultaneously, causing controller spawners to fail because the controller manager was not ready. Changed to ordered launch:
1. `spawn_robot` → on exit → `joint_state_broadcaster` → on exit → `joint_trajectory_controller` → on exit → `admittance_insertion_node`
2. Added FT sensor bridge (`ft_sensor_bridge.yaml`), `data_logger_node`, and `admittance_insertion_node` to the launch
3. Added `--controller-manager-timeout 60 --switch-timeout 30` to spawners

### Exact Test Commands

Headless validation (120 s timeout):
```
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup research_baseline.launch.py headless:=true timeout_seconds:=120 2>&1 | tee /tmp/launch_run.log
```

Visible GUI test:
```
ros2 launch thesis_bringup research_baseline.launch.py
```

Command motion (second terminal):
```
ros2 launch kuka_task_control baseline_trajectory.launch.py
```

### Controller Status

```
joint_trajectory_controller  joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
```

Both controllers were loaded, configured, and activated successfully.

### Initial Joint Position Evidence

Hardware interface confirmed SAFE_HOME pose at startup:
```
joint_1:  0.000000  (SAFE_HOME:  0.0)
joint_2: -0.799989  (SAFE_HOME: -0.8) ✓
joint_3:  1.200006  (SAFE_HOME:  1.2) ✓
joint_4:  0.000003  (SAFE_HOME:  0.0)
joint_5:  0.799983  (SAFE_HOME:  0.8) ✓
joint_6:  0.000002  (SAFE_HOME:  0.0)
```

### Motion Evidence

The admittance insertion node transitioned `IDLE → MOVING_TO_START` and sent a trajectory goal to the controller. The controller received and executed it:

```
[joint_trajectory_controller]: Received new action goal
[joint_trajectory_controller]: Accepted new action goal
[joint_trajectory_controller]: Goal reached, success!
[joint_trajectory_controller]: Received new action goal  (second phase)
[joint_trajectory_controller]: Accepted new action goal
```

Joint positions changed from SAFE_HOME to:
```
Initial: [0.000, -0.800, 1.200, 0.000, 0.800, 0.000]
Final:   [-0.052, -0.298, 0.841, 0.009, 0.800, 0.044]
```

The robot moved from the SAFE_HOME posture towards the task start pose.

### Known Limitations

1. **Task did not progress beyond MOVING_TO_START** — The admittance node stayed in MOVING_TO_START for the full 120 s test. The trajectory executed and completed ("Goal reached, success!"), but the automaton did not detect contact or transition to MOVING_TO_CONTACT/INSERTION. The second goal was also accepted.
2. **Data logger records NaN for velocity** — The Gazebo joint state bridge does not publish velocity data, so the data logger CSV shows `nan` in velocity columns.
3. **Robot model not in `kuka_robot_descriptions`** — The `lbr_iisy6_r1300_research_gripper.urdf.xacro` is a project-specific variant; the base meshes come from the external submodule.
4. **Package not found in subshell** — Running `baseline_trajectory.launch.py` from a clean terminal requires sourcing the workspace first.

### Next Milestone

`proposal_simulation_cell_v2_17_contact_gated_moving_to_start_transition`

- Why the admittance node stays in MOVING_TO_START after trajectory completion
- Debug the contact detection threshold (5.0 N) vs. observed contact wrench (~2.7 N, below threshold)
- Verify the FT sensor bridge remapping from `/world/peg_in_hole_world/model/lbr_iisy6_r1300/joint/ft_sensor_joint/sensor/ft_sensor/forcetorque` → `/ft_sensor_wrench`
- Confirm the automaton state machine logic checks for contact after trajectory complete

## admittance_controller_v2_honest_tracking_and_contact_estimation

Status: `first_simulated_insertion_event_observed_repeat_validation_pending`

The v2 controller has produced one simulated insertion-depth event with measured insertion depth, but that single run is not enough to claim robust autonomous peg-in-hole success. Until repeated validation shows stable behavior, the correct wording is: **first simulated insertion event with measured insertion depth**.

### Fix 1 — State machine honesty

The original state machine (IDLE → MOVING_TO_START → APPROACH → INSERT → RETREAT → DONE) had no mechanism to detect or report failure:
- If MOVING_TO_START or APPROACH timed out without reaching Cartesian tolerance (0.025 m), the controller proceeded blindly to the next phase.
- The DONE state reported "Full cycle completed successfully" even when tracking never converged and the peg never entered the hole.
- XY error at the hole surface (0.04 m) was 40× the required clearance (~0.001 m), but the controller proceeded to INSERT anyway.

**Fix applied:**
- MOVING_TO_START and APPROACH now ABORT with a logged reason if the trajectory does not converge within the configured tolerance and timeout. No silent proceed.
- A CHECK_ALIGNMENT sub-phase was considered but replaced with direct XY-error gating: APPROACH checks `pre_insertion_xy_error ≤ INSERTION_XY_TOLERANCE (0.002 m)` before allowing INSERT. If the error is too large, a SEARCH phase is attempted before aborting.
- DONE is never reached without a correct trial outcome (SUCCESS, DEGRADED, ABORTED) and a human-readable reason string. The outcome distinguishes "tracking timeout" from "alignment error" from "insertion succeeded" from "insertion incomplete".
- Each phase records `{success, cart_error, joint_error, timeout, message}` for the final diagnostic log.

### Fix 2 — Motion and tracking accuracy

The original controller sent a single JointTrajectory point on the topic interface with a fixed 5 s duration. There was no feedback from the controller, no multi-point interpolation, and no adaptation to the distance-to-target.

**Fix applied:**
- Long moves are broken into intermediate waypoints (linear interpolation in joint space) with durations scaled by the max joint-space distance.
- The FollowJointTrajectory action client is used when available, with fallback to the topic interface.
- Trajectory durations are computed as `max(5, min(15, distance × 15))` seconds, giving the controller more time for large motions.
- Multiple waypoints (2–10 depending on distance) give the controller smoother targets.

**Known limitation:** The `gz_ros2_control/GazeboSimSystem` hardware interface uses position command interfaces only (no velocity/effort). Tracking accuracy is fundamentally limited by the PD gains in the simulation plugin, which are not user-configurable from the ROS side. The 0.025 m Cartesian tolerance and 0.002 m XY alignment tolerance are engineering targets; actual performance depends on Gazebo physics settings and controller tuning.

### Fix 3 — Gravity and contact estimation

The original controller captured a single `_baseline_fz` at state transition. During INSERT, the robot configuration changes significantly, causing the gravity component at the FT sensor to drift by 30 N or more. Contact was computed as `Fz − baseline`, so contact remained 0.00 N even when Fz reached 82.87 N.

**Fix applied:**
- A running median filter over a sliding window of 50 Fz samples is continuously updated while the controller is active.
- The baseline is computed as the median of recent samples (robust to outliers).
- A 2.0 N deadband prevents noise from being reported as contact.
- Contact force = `max(0, Fz − baseline − deadband)`.
- The baseline is valid after 10 samples have been collected.

**Limitation:** The running median assumes the robot is in free space (no contact) during baseline collection. If the peg contacts the hole surface while the baseline window includes contact forces, the baseline will drift upward and mask real contact. Future work: gate the baseline update on Z-height (only collect when peg Z > touch_Z + margin).

### Fix 4 — Search/homing phase

When the pre-insertion XY error exceeds `INSERTION_XY_TOLERANCE (0.002 m)`, a simple spiral search is executed at the touch Z height (0.830 m). The search:
- Starts at radius 0.003 m and expands to max 0.015 m.
- Visits 8 angular positions per radius.
- Uses IK + trajectory publication (controller-driven, not fake).
- After each step, rechecks the XY error. If within tolerance, proceeds to INSERT.
- Exhaustion without convergence → ABORT with reason.

### Fix 5 — Comprehensive logging

All phases and metrics are logged:
- State transitions with timestamps
- Per-phase tracking errors (Cartesian and joint)
- Peg-tip XY error at pre-insertion
- Insertion depth (from Cartesian Z tracking)
- Raw Fz (max observed)
- Gravity baseline estimate (median of running window)
- Contact force estimate (Fz − baseline − deadband)
- Trial outcome: SUCCESS / DEGRADED / ABORTED with reason
- A JSON file is written to `/tmp/insertion_trial_outcome.json` for post-mortem analysis
- A JSON message is published on `/insertion_log` for real-time monitoring

### Repeat Validation Harness

`experiment_manager` now includes a process-level repeat-run harness:

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run experiment_manager research_baseline_repeat_validator --trials 3 --timeout-s 150
```

The harness starts a fresh `research_baseline.launch.py use_gui:=false` process for each trial, waits for `/tmp/insertion_trial_outcome.json`, and writes:

- `diagnostics/research_baseline_repeat_validation/repeat_trials.csv`
- `diagnostics/research_baseline_repeat_validation/summary.json`
- `diagnostics/research_baseline_repeat_validation/summary.md`
- per-trial launch logs and outcome JSON files

Physical success is counted only when `trial_outcome == SUCCESS`, insertion depth is at least 0.010 m, contact force exceeds the configured threshold, and no safety abort occurs.

### 2026-06-01 Repeat Validation Result

Command:

```bash
ros2 run experiment_manager research_baseline_repeat_validator --trials 3 --timeout-s 220 --output-dir diagnostics/research_baseline_repeat_validation_v2
```

Result: `0/3` physical successes.

- Trial 1: `DEGRADED`, failed INSERT, depth `0.0037 m`, peak raw Fz `1237.45 N`, max contact `1142.44 N`.
- Trial 2: `ABORTED`, failed INSERT by safety threshold, depth `0.0367 m`, peak raw Fz `3716.2 N`, max contact `3682.56 N`.
- Trial 3: `NO_OUTCOME`, harness timeout during extended SEARCH before insertion outcome.

Current conclusion: the baseline is not robust. The next technical milestone is force-safe insertion stabilization: reduce search/insert contact spikes, prevent unsafe descents when XY tracking is poor, and make SEARCH bounded by explicit timeout/outcome criteria.

### 2026-06-01 Force-Safe Insert Stabilization Result

Implemented after the failed repeat run:

- Physical insertion depth is now measured as depth below the hole top (`hole_top_z - peg_z`), not merely relative downward motion.
- INSERT is blocked unless XY error is below `0.015 m`, peg tip Z is at or below `0.845 m`, and force is below the safety threshold.
- SEARCH has a bounded timeout.
- Raw Fz above `1000 N` now triggers a hard global abort outside INSERT as well.
- ABORT retreat timeout is shortened so failures can be logged promptly.

Validation command:

```bash
ros2 run experiment_manager research_baseline_repeat_validator --trials 3 --timeout-s 130 --output-dir diagnostics/research_baseline_force_safe_insert_v3
```

Result: `0/3` physical successes. The controller is safer about not entering INSERT when the peg is too high, but the baseline remains failed:

- Trial 1: hard-force abort in SEARCH at raw Fz `1164.3 N`; no final outcome JSON before harness timeout.
- Trial 2: MOVING_TO_START timeout/degraded failure, max raw Fz `77.38 N`.
- Trial 3: hard-force abort in SEARCH at raw Fz `2990.77 N`, final `ABORTED`.

Current conclusion: high force is not only an INSERT problem; SEARCH/approach correction can generate unsafe contact before insertion. The next milestone is to replace surface-level SEARCH with a no-contact XY alignment strategy above the workpiece, then descend only after XY alignment is stable.

### 2026-06-01 No-Contact Alignment Gate Result

Implemented after SEARCH was shown to be unsafe:

- APPROACH is now blocked unless above-hole XY error after `MOVING_TO_START` is at or below `0.030 m`.
- This prevents descent and contact-seeking SEARCH when the robot is still laterally far from the hole.

Validation command:

```bash
ros2 run experiment_manager research_baseline_repeat_validator --trials 3 --timeout-s 120 --output-dir diagnostics/research_baseline_no_contact_alignment_v1
```

Result: `0/3` physical successes, but all three trials produced bounded final outcomes before descent:

- Trial 1: `ABORTED`, APPROACH blocked at XY error `0.1034 m`, peak raw Fz `481.15 N`.
- Trial 2: `ABORTED`, APPROACH blocked at XY error `0.0911 m`, peak raw Fz `360.40 N`.
- Trial 3: `ABORTED`, APPROACH blocked at XY error `0.0872 m`, peak raw Fz `193.17 N`.

Current conclusion: the controller now fails earlier and more honestly before descending, but MOVING_TO_START tracking is too poor for the task. The next technical milestone is to improve above-hole joint target generation/tracking so the peg reaches the no-contact XY gate reliably.

### 2026-06-01 Above-Hole Target Refresh Experiment

An experiment re-published the final `MOVING_TO_START` target to improve hold tracking. Validation command:

```bash
ros2 run experiment_manager research_baseline_repeat_validator --trials 3 --timeout-s 130 --output-dir diagnostics/research_baseline_above_hole_tracking_v1
```

Result: `0/3` physical successes and worse safety behavior. Two trials hard-aborted in `MOVING_TO_START` with raw Fz spikes of `4086.95 N` and `1766.64 N`; the third still failed the no-contact XY gate at `0.1116 m`.

The target-refresh strategy was not retained. Current conclusion: above-hole tracking cannot be fixed by repeatedly re-publishing the same joint target; the next attempt should revisit the joint target itself, controller gains/physics, or a safer multi-stage free-space path.

### 2026-06-02 Move-To-Start Hold Correction Experiment

A bounded final hold experiment tested whether up to three same-target hold commands could settle the already-computed `MOVING_TO_START` joint target without weakening the strict 2 mm no-contact descent gate.

Validation command:

```bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_move_to_start_hold_correction
```

Result: the strategy was rejected and the code was not retained. The trial produced transient good XY samples (`0.0028 m` and `0.0008 m`) but never satisfied the consecutive stability gate. It timed out safely in `MOVING_TO_START`:

- `Outcome: ABORTED`
- `cart_err=0.016 m`
- `xy_err=0.011 m`
- `joint_err=0.014 rad`
- `stable=0/5`
- `Depth: 0.0000 m`
- `Max Fz: 170.8 N`

Tracking evidence in `diagnostics/research_baseline_move_to_start_hold_correction/trajectory_tracking_summary.md` showed p95 max joint error `0.026865 rad` and final max joint error `0.030123 rad`. Current conclusion: repeated final hold commands can momentarily cross the XY threshold, but do not create a stable safe descent condition. The next work should diagnose runtime tracking/physics and free-space F/T behavior near the above-hole target.

### 2026-06-02 Raw Wrench Abort Instrumentation

Implemented after passive wrench evidence showed that the controller could miss sub-control-period raw wrench spikes during `MOVING_TO_START`.

Changes:

- Added passive `wrench_state_observer` to log `/ft_sensor_wrench` by insertion state and peg pose.
- Added callback-level raw wrench peak tracking in `admittance_insertion_node`.
- Hard-force abort now latches on raw `|Fz|` or force norm above `1000 N` in active task states, including `MOVING_TO_START`.
- Outcome JSON now records `max_abs_fz_N` and `max_force_norm_N`; legacy `max_fz_N` is preserved for the repeat validator.

Validation command:

```bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_raw_wrench_abort
```

Result: safety abort in `MOVING_TO_START`, not insertion success:

- `Outcome: ABORTED`
- `Reason: Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START`
- `Max |Fz|: 1943.3 N`
- `Max |F|: 2936.5 N`
- `Depth: 0.0000 m`

Current conclusion: high free-space raw wrench spikes are now measured and safety-latched. The next blocker is determining whether they come from hidden contact, FT sensor semantics, inertial dynamics, or Gazebo/controller physics.

### 2026-06-02 Contact-Wrench Correlation

Added passive `contact_state_observer` for the canonical bridged contact topics: `/gazebo/contacts/peg`, `/gazebo/contacts/hole`, and `/gazebo/contacts/target`.

Validation command:

```bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_contact_wrench_correlation
```

Result: another safe raw-wrench abort in `MOVING_TO_START`, with no insertion:

- `Outcome: ABORTED`
- `Max |Fz|: 1396.8 N`
- `Max |F|: 2624.1 N`
- `Depth: 0.0000 m`

Contact observer result:

- contact samples: `0`
- positive contact samples: `0`
- max contact force from contact topics: `0.000000 N`

Current conclusion: the canonical contact topics did not provide positive contact evidence for the raw wrench spike. This narrows the next investigation to FT sensor semantics, inertial/dynamic loads, uninstrumented collision pairs, or Gazebo/controller physics.

### 2026-06-02 F/T Mount Effort-Limit Validation

The F/T measurement joint in `lbr_iisy6_r1300_research_gripper.urdf.xacro` is a
zero-range revolute joint because Gazebo's URDF-to-SDF conversion collapses
fixed joints and would remove the named joint needed for the joint-level
force-torque sensor. The previous `effort=1`, `velocity=0` limit was physically
too weak for a rigid sensor mount, so it was changed to `effort=10000`,
`velocity=100` while preserving lower/upper limits at `0`.

Validation command:

```bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_ft_mount_effort_limit
```

Result: the F/T bridge and controllers still loaded, and the spike was reduced
but not eliminated:

- `Outcome: ABORTED`
- `Reason: Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START`
- `Max |Fz|: 612.25 N`
- `Max |F|: 1765.41 N`
- `Depth: 0.0000 m`
- contact-topic samples: `0`

Current conclusion: the weak measurement-joint limit was a credibility issue and
has been corrected, but it was not the full root cause. The next investigation
should localize uninstrumented collisions or late free-space dynamics near the
above-hole target; the hard-force abort remains unchanged.

### 2026-06-02 Full-Path Contact Bridge Validation

The research baseline contact bridge now maps full Gazebo contact sensor paths
back to stable ROS topics. `spawn_robot_sdf.py` also injects a robot-mounted
`peg_contact_sensor` on the converted `ft_sensor_link`, because the active peg
is fixed into the robot model rather than spawned as the standalone
`cylindrical_peg`.

Validation command:

```bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_contact_bridge_full_paths
```

Result: the three contact bridges were created and Gazebo reported peg, target,
and fixture contact sensors publishing. The task still aborted safely in
`MOVING_TO_START`, but the contact observer now recorded positive peg-target
contact:

- `Outcome: ABORTED`
- `Max |Fz|: 939.36 N`
- `Max |F|: 1748.50 N`
- `Depth: 0.0000 m`
- contact samples: `50`
- positive contact samples: `50`
- `MOVING_TO_START` peg/target max contact force: `2427.31 N`

Current conclusion: the previous zero-contact result was an observability gap.
The raw wrench abort is now correlated with peg-target contact before descent.
The next change should keep the no-contact start pose physically clear of the
target plate; the hard-force abort and strict stability gate remain unchanged.

### 2026-06-02 Axis-Aligned Start Pose Validation

The previous `MOVING_TO_START` target used position-only IK. Offline FK showed
that the peg tip reached `[0.520, -0.200, 0.885]` while the peg body was tilted
about 116 deg from world +Z, allowing the peg body to sweep into the target
plate before descent. `RobotKinematics.inverse_position_axis(...)` now solves
peg-tip position while constraining peg local +Z to world +Z, with iisy6 joint
limits enforced.

Validation command:

```bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_axis_aligned_start_pose
```

Result: no raw hard-force abort occurred, and no peg-source contact rows were
recorded. The run still failed honestly at `MOVING_TO_START`:

- `Outcome: ABORTED`
- `Reason: MOVING_TO_START timeout/failure (90.0s)`
- final `cart_err`: `0.011498 m`
- final logged `xy_err`: `0.006 m`
- `Max |Fz|: 554.24 N`
- `Max |F|: 628.61 N`
- `Depth: 0.0000 m`

Current conclusion: axis-aligned IK fixes the tilted-peg safety issue but
creates a larger 2.4145 rad no-contact move that does not settle within the
existing 90 s timeout and 2 mm XY stability gate. The next milestone should
improve trajectory timing or split the start move through a clear staging pose;
do not loosen the safety gate.

### Files changed

- `kuka_task_control/kuka_task_control/admittance_insertion_node.py` — Complete rewrite of the state machine with honest tracking, running gravity baseline, multi-point trajectories, SEARCH phase, and comprehensive outcome logging.
- `README.md` — Added this section.

### Exact Test Commands

Same as research baseline:
```
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup research_baseline.launch.py headless:=true timeout_seconds:=120 2>&1 | tee /tmp/launch_run.log
```

Note: the current launch file exposes `use_gui:=false` for headless server operation. External timeout should be applied with shell `timeout` or the repeat validator.


## research_baseline_v0_2_camera_visual_size_fix

Status: `camera_visual_size_fixed`

The research baseline v0.2 fix reduces the D405 RGB-D camera visual body to a realistic small external camera size.

### Fix — Camera body box size reduced

The camera visual geometry was a 40 mm × 40 mm × 25 mm dark-gray box that appeared too large relative to the robot and workspace. Reduced to 30 mm × 25 mm × 20 mm.

**Files changed:**
- `peg_in_hole_description/worlds/peg_in_hole_world.sdf:117` — SDF world model box size (the actual Gazebo model)
- `peg_in_hole_description/urdf/lbr_iisy6_r1300_research_gripper.urdf.xacro:86` — URDF xacro box size (disabled via `include_camera:=false` in the research baseline)
- `peg_in_hole_description/urdf/lbr_iisy3_r760_research_gripper.urdf.xacro` — same fix for consistency
- `peg_in_hole_description/urdf/lbr_iisy11_r1300_research_gripper.urdf.xacro` — same fix for consistency

**New approximate camera dimensions:**
- width: 0.030 m (30 mm)
- depth: 0.025 m (25 mm)
- height: 0.020 m (20 mm)

**Camera placement unchanged:**
- Pose: `0.42 -0.55 1.18 0.95 0 0.35` — outside robot workspace, no collision
- Orientation: roll=0.95 rad, yaw=0.35 rad — still points toward the peg-hole workspace

**Not changed:**
- Robot model, joints, initial joint positions, controllers, baseline trajectory, Gazebo spawn logic
- Table, peg, hole, fixture, or workspace dimensions
- Camera sensor parameters (resolution, FOV, clip range, topics)
- Active camera sensor disabled state for WSL/Gazebo stability (remains disabled in URDF, enabled in SDF world model)

### Exact Test Commands

Same as research baseline:
```
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup research_baseline.launch.py
```

### Verification

Launch completed successfully:
- Gazebo loaded the world with the smaller camera model
- Robot spawned correctly at the pedestal mount pose
- Controllers loaded and activated: `joint_state_broadcaster`, `joint_trajectory_controller`
- Camera sensors publishing: `/d405/color/image_raw`, `/d405/depth/image_rect_raw`
- Admittance insertion node transitioned `IDLE → MOVING_TO_START`
- No collision issues observed

## proposal_simulation_cell_v2_16_guarded_peg_in_hole_objective_validation

Status: `guarded_peg_in_hole_objective_attempt_completed_with_failure_reason`

The v2.16 proposal simulation sprint adds the first main-objective guarded peg-in-hole validation attempt. It computes peg/hole geometry, validates an above-hole alignment phase, executes guarded insertion steps only when alignment and safety gates permit it, records insertion success or an exact failure reason, and monitors contact force and safety gates. The recorded attempt stopped before insertion because the initial lateral alignment error was above tolerance.

This sprint is Gazebo-only. It does not use a real robot, use a physical endpoint, perform forceful contact, or fake insertion success. Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_16/`.

## proposal_simulation_cell_v2_15_context_action_ablation_validation

Status: `context_action_ablation_validated`

The v2.15 proposal simulation sprint adds a paired diagnostic ablation comparing fixed-baseline guarded action parameters with deterministic context-conditioned guarded action parameters. The five validated scenarios are tested under two action modes, and paired comparison reports are generated for trigger step, max force, final return error, safety violations, and action parameter differences.

This sprint is diagnostic ablation only. It does not run RL training, train a policy, create fake learning results, use a real robot, use a physical endpoint, execute peg insertion, or perform forceful contact. Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_15/`.

## proposal_simulation_cell_v2_14_context_conditioned_guarded_action_validation

Status: `context_conditioned_guarded_action_validated`

The v2.14 proposal simulation sprint uses the real v2.13 deterministic context embeddings to generate guarded action suggestions for the five validated contact-gate scenarios. It validates the suggested action parameters against safety bounds, then performs Gazebo-only contact-gated execution with initial no-contact checks, stop-on-contact, retreat, post-retreat no-contact checks, and return-to-ready checks.

This sprint does not train a policy, run RL training, create fake learning results, use a real robot, use a physical endpoint, execute peg insertion, or perform forceful contact. Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_14/`.

## proposal_simulation_cell_v2_13_context_encoder_prototype

Status: `context_encoder_prototype_validated`

The v2.13 proposal simulation sprint adds a deterministic context encoder prototype using the real v2.12 simulation context vectors. It defines a stable context feature schema, validates required normalized features, generates deterministic 8-D context embeddings for each scenario, and writes similarity plus nearest-context reports.

The prototype does not train a policy, run RL training, create fake learning results, use a real robot, or execute peg insertion. Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_13/`.

## proposal_simulation_cell_v2_12_context_vector_extraction

Status: `context_vector_extraction_validated`

The v2.12 proposal simulation sprint extracts compact context vectors from the real v2.11 Gazebo simulation observation logs. It reads the v2.11 multimodal observation log, contact-transition log, scenario summary, RGB-D frame-count report, channel completeness report, and safety report.

Scenario-level context vectors, contact-transition feature vectors, episode summaries, observation-channel summaries, safety-gated context summaries, and a metadata manifest are generated. This sprint performs feature extraction only: no fake dataset, fake result, learning, policy training, real robot execution, peg insertion, or forceful contact is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_12/`.

## proposal_simulation_cell_v2_11_multimodal_contact_observation_logging

Status: `multimodal_contact_observation_logging_validated`

The v2.11 proposal simulation sprint adds synchronized multimodal contact observation logging. It replays the five validated v2.10 misalignment contact-gate scenarios and records observation rows for RGB-D availability, joint state, TF/tool pose, contact wrench, task phase, scenario metadata, and contact transition labels.

RGB-D topics were available, frame counts were recorded, and lightweight metadata was saved without writing full image datasets. The outputs are real Gazebo simulation observation logs only. No fake dataset, experimental performance claim, real robot execution, physical endpoint, peg insertion, forceful contact, or learning is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_11/`.

## proposal_simulation_cell_v2_10_misalignment_contact_gate_batch_validation

Status: `misalignment_contact_gate_batch_validated`

The v2.10 proposal simulation sprint adds misalignment contact-gate batch validation using MoveIt planning and Gazebo-only execution. It runs five actual Gazebo scenarios: nominal centered, positive x offset, negative x offset, positive y offset, and negative y offset. Each scenario computes the calibration pad pose from robot/tool/table geometry, applies the lateral offset, verifies initial no-contact, triggers contact after guarded approach motion, stops on contact, retreats, verifies post-retreat no-contact, and returns to ready.

Scenario definitions, pad poses, IK reachability, initial no-contact checks, contact transitions, post-retreat checks, safety evidence, and endpoint checks are recorded. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or fake scenario evidence is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_10/`.

## proposal_simulation_cell_v2_9_non_overlapping_approach_to_contact_validation

Status: `non_overlapping_approach_to_contact_validated`

The v2.9 proposal simulation sprint adds non-overlapping approach-to-contact validation using MoveIt planning and Gazebo-only execution. It computes the robot/tool/table/pad geometry, places the simulation-only calibration pad on the computed tool path with positive clearance, verifies the initial no-contact standby condition, and executes bounded approach motion until contact triggers after motion rather than at step 0.

Stop-on-contact, retreat, post-retreat no-contact, return-to-ready, raw contact evidence, and derived compliant force evidence are recorded. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_9/`.

## proposal_simulation_cell_v2_8_contact_reachability_and_trigger_validation

Status: `contact_reachability_and_trigger_validated`

The v2.8 proposal simulation sprint adds contact reachability and trigger validation using MoveIt planning and Gazebo-only execution. It computes a simulation-only calibration pad pose relative to the tool/distal-link path, checks raw contact topic wiring, records raw contact plus derived wrench evidence, and runs bounded contact-trigger steps through the verified Gazebo simulation endpoint.

The contact gate triggered with nonzero raw contact and derived wrench evidence. Stop-on-contact, retreat, return-to-ready, and final state validation are recorded. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_8/`.

## proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_calibration

Status: `contact_triggered_guarded_touch_not_reached`

The v2.7 proposal simulation sprint adds contact-triggered guarded touch calibration using MoveIt planning and Gazebo-only execution. It uses a simulation-only contact calibration target, records guarded touch steps, records contact wrench reports, and validates stop-on-contact plus retreat behavior if the contact gate triggers.

The contact gate was not reached within the bounded guarded touch steps, and the diagnostic output records that result without fake contact evidence. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_7/`.

## proposal_simulation_cell_v2_6_contact_gated_guarded_approach_validation

Status: `contact_gated_guarded_approach_validated_no_contact_detected`

The v2.6 proposal simulation sprint adds a contact-gated guarded approach sequence using MoveIt planning and Gazebo-only execution. It validates ready, pre-approach, pre-contact standby, guarded approach steps, stop-on-contact or stand-off gating, retreat, return-to-ready, and final state validation through the verified Gazebo simulation endpoint.

Guarded approach step reports, contact gate reports, joint-state evidence, endpoint checks, and contact wrench monitoring are recorded. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_6/`.

## proposal_simulation_cell_v2_5_guarded_pre_contact_task_sequence

Status: `guarded_pre_contact_task_sequence_validated`

The v2.5 proposal simulation sprint adds a guarded pre-contact task sequence using MoveIt planning and Gazebo-only execution. It validates ready, pre-approach, pre-insertion standby, hold, and return phases while verifying the Gazebo simulation endpoint before executed phases.

Phase reports, joint-state evidence, endpoint checks, and contact wrench monitoring are recorded. No real robot execution, physical endpoint, peg insertion, contact-seeking motion, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_5/`.

## proposal_simulation_cell_v2_4_moveit_gazebo_execution_validation

Status: `moveit_gazebo_execution_validated`

The v2.4 proposal simulation sprint adds the first MoveIt-generated Gazebo-only trajectory execution. It verifies the Gazebo simulation controller endpoint, generates and executes one small MoveIt plan only in Gazebo, records joint-state before/after evidence, returns to the initial posture, and monitors the contact wrench.

No real robot execution, physical endpoint, peg insertion, contact-seeking motion, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_4/`.

## proposal_simulation_cell_v2_3_moveit_model_alignment_and_plan_only_validation

Status: `moveit_model_alignment_and_plan_only_validated`

The v2.3 proposal simulation sprint adds a MoveIt/Gazebo model alignment audit, five nearby diagnostic IK checks for repeatability, and MoveIt plan-only validation.

No trajectory execution, controller execution, real robot execution, `FollowJointTrajectory` execution, peg insertion, contact-seeking motion, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_3/`.

## proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation

Status: `moveit_ik_diagnostic_validated`

The v2.2 proposal simulation sprint adds diagnostic-only MoveIt IK validation. It loads the diagnostic MoveIt model, starts `move_group` with trajectory execution disabled, verifies `/compute_ik`, and records the IK request and response.

No real robot execution, controller execution, trajectory execution, `FollowJointTrajectory` execution, peg insertion, contact-seeking motion, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_2/`.

## proposal_simulation_cell_v2_1_gazebo_motion_validation_suite

Status: `gazebo_motion_validation_suite_validated`

The v2.1 proposal simulation sprint adds a combined Gazebo-only motion validation suite. It tests single forward and return motion, three repeatability cycles, and a small two-joint motion using the Gazebo simulation controller.

The suite records joint-state evidence, repeatability and return errors, contact wrench monitoring, and a safety report. No real robot execution, MoveIt, `/compute_ik`, learning, scenario batch execution, peg insertion, or contact-seeking motion is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_1/`.

## proposal_simulation_cell_v2_0_first_gazebo_motion_smoke_test

Status: `first_gazebo_motion_smoke_test_validated`

The v2.0 proposal simulation sprint adds the first intentional Gazebo-only motion smoke test. It sends one small joint-space movement for the selected sixth-axis joint, records joint-state before/after evidence, monitors the contact wrench topic, and writes a safety report.

The smoke test is Gazebo-only: no real robot execution, no MoveIt, no `/compute_ik`, no learning, and no scenario batch execution are used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_0/`.

## proposal_simulation_cell_v1_17_release_documentation_index

Status: `release_documentation_index_validated`

The v1.17 proposal simulation sprint adds a release documentation index, reviewer quickstart, sprint traceability, and no-false-claims statement. The documents link the v1.15 evidence package and v1.16 reproducibility checklist, summarize completed sprints v1.0, v1.1, v1.2, v1.3, and v1.5 through v1.16, and confirm that v1.4 remains absent/not implemented.

The release index is documentation-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, no `FollowJointTrajectory`, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_17/`.

## proposal_simulation_cell_v1_16_reproducibility_checklist

Status: `reproducibility_checklist_validated`

The v1.16 proposal simulation sprint adds a reproducibility checklist and reviewer-facing implementation summary. It verifies that the v1.15 evidence package and evidence registry are available, checks implemented diagnostics folders, and confirms that v1.4 remains absent/not implemented.

The checklist is diagnostic-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_16/`.

## proposal_simulation_cell_v1_15_evidence_package_generator

Status: `evidence_package_validated`

The v1.15 proposal simulation sprint adds an evidence package generator. It collects evidence from v1.0, v1.1, v1.2, v1.3, and v1.5 through v1.14, marks v1.4 as absent/not implemented and not invented, generates the proposal simulation evidence package, and creates a validated evidence summary.

The package is evidence-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_15/`.

## proposal_simulation_cell_v1_14_batch_dry_run_orchestrator

Status: `batch_dry_run_orchestrator_validated`

The v1.14 proposal simulation sprint adds a batch dry-run orchestrator. It converts the v1.13 batch execution plan into blocked dry-run orchestration records, defines the per-scenario gate-check order, and adds the blocked batch execution report.

The orchestration is configuration-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_14/`.

## proposal_simulation_cell_v1_13_batch_execution_plan_validator

Status: `batch_execution_plan_validated`

The v1.13 proposal simulation sprint adds a batch execution plan validator. It converts the selected v1.12 batch into a configuration-only execution plan, lists required gates for every scenario, and defines planned diagnostic outputs.

The plan is configuration-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_13/`.

## proposal_simulation_cell_v1_12_scenario_batch_selector

Status: `scenario_batch_selector_validated`

The v1.12 proposal simulation sprint adds a scenario batch selector. It loads a representative selected batch from the v1.10 matrix and validates the selected scenarios.

The batch is configuration-only: no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_12/`.

## proposal_simulation_cell_v1_11_single_scenario_loader_validation

Status: `single_scenario_loader_validated`

The v1.11 proposal simulation sprint adds a single-scenario loader. It loads the selected scenario from the v1.10 matrix and validates the selected scenario configuration.

The loader is configuration-only: no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_11/`.

## proposal_simulation_cell_v1_10_experiment_configuration_matrix

Status: `experiment_configuration_matrix_validated`

The v1.10 proposal simulation sprint adds an experiment configuration matrix for future peg-in-hole validation scenarios. It defines scenario variants for clearance, x/y offset, angular misalignment, insertion depth, and contact thresholds.

The matrix is configuration-only: no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_10/`.

## proposal_simulation_cell_v1_9_no_motion_control_law_dry_run

Status: `no_motion_control_law_dry_run_validated`

The v1.9 proposal simulation sprint adds a no-motion control-law dry run. It reads validated simulated inputs and generates diagnostic control-law output, a blocked control command, and safety clipping/reporting evidence without connecting any output to execution.

The dry-run command remains blocked. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_9/`.

## proposal_simulation_cell_v1_8_control_development_scaffold

Status: `control_development_scaffold_validated`

The v1.8 proposal simulation sprint adds a control-development scaffold for future controller work without executing robot motion. It includes the control input monitor, diagnostic command proposal, command blocker, safety gate checker, control boundary checker, and control readiness report.

The command proposal is diagnostic only and blocked. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_8/`.

## proposal_simulation_cell_v1_7_pre_control_contract

Status: `pre_control_contract_validated`

The v1.7 proposal simulation sprint adds a pre-control simulation contract. It defines the required input signal contract, allowed diagnostic output suggestions, forbidden execution interfaces, readiness dependency contract, and future controller boundary before any controller work is introduced.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_7/`.

Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

## proposal_simulation_cell_v1_6_safety_gate_readiness

Status: `safety_gate_readiness_validated`

The v1.6 proposal simulation sprint adds readiness gates for the next control-development stage. It evaluates the sensor gate, contact gate, safety gate, virtual-force gate, admittance gate, execution-disabled gate, and proposal readiness gate from validated simulation diagnostic signals only.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_6/`.

Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, no real robot execution, no `FollowJointTrajectory`, and no command output.

## proposal_simulation_cell_v1_5_safety_virtual_force_interface

Status: `safety_virtual_force_interface_validated`

The v1.5 proposal simulation sprint adds a simulation-only runtime safety and virtual-force interface foundation. It adds the safety status interface on `/proposal_simulation_cell/safety_status`, contact-state classification on `/proposal_simulation_cell/contact_state`, virtual-force diagnostic command suggestions on `/proposal_simulation_cell/virtual_force_command`, and admittance diagnostic command suggestions on `/proposal_simulation_cell/admittance_command_suggestion`.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_5/`. The validated run used Gazebo fallback because Isaac Sim was unavailable. The contact wrench topic and sample were available, the maximum observed force was `0.0981000000182301 N`, and the final contact state was `contact_below_threshold` against the configured `0.1 N` detection threshold.

Safety constraints are enforced in config and diagnostics: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, no real robot execution, no `FollowJointTrajectory`, and no command execution.
