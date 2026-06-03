# Current Project Status

Date: 2026-06-03

## Review Summary

The repository is an active ROS 2 Jazzy / Gazebo research workspace, not a blank future workspace. The current implementation has moved beyond proposal-only diagnostics into a controller-driven KUKA LBR iisy 6 R1300 Gazebo baseline. Controllers can activate and the task controller can command simulated motion.

The project must not claim final autonomous peg-in-hole success yet. The defensible claim is a historical simulated insertion-depth/contact event, followed by a stricter current baseline that correctly rejects side-loaded insertion as `DEGRADED`.

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
- `diagnostics/research_baseline_search_fail_closed_v2/summary.md`
- `diagnostics/research_baseline_search_fail_closed_v2/approach_tracking_analysis.md`
- `diagnostics/research_baseline_slow_approach_descent_v1/approach_tracking_analysis.md`
- `diagnostics/research_baseline_approach_gain_3000_v1/approach_tracking_analysis.md`
- `diagnostics/research_baseline_joint_damping_scale_0p2_v1/summary.md`
- `diagnostics/research_baseline_joint_effort_scale_2p0_v1/summary.md`
- `diagnostics/research_baseline_contact_pair_attribution_v1/summary.md`
- `diagnostics/research_baseline_tool_tip_frame_correction_v1/summary.md`
- `diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1/summary.md`
- `diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1/summary.md`
- `diagnostics/research_baseline_start_gain_3000_after_tool_fix_v1/summary.md`
- `diagnostics/research_baseline_zero_derivative_trajectory_hold_v1/summary.md`
- `diagnostics/research_baseline_tool_tip_frame_correction_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_start_gain_3000_after_tool_fix_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_zero_derivative_trajectory_hold_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_tool_tip_frame_correction_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_start_gain_3000_after_tool_fix_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_zero_derivative_trajectory_hold_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_start_endpoint_correction_v1/summary.md`
- `diagnostics/research_baseline_start_endpoint_correction_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_start_endpoint_correction_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_joint_damping_scale_2p0_v1/summary.md`
- `diagnostics/research_baseline_joint_damping_scale_2p0_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_joint_damping_scale_2p0_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_damping_2p0_gain_1500_v1/summary.md`
- `diagnostics/research_baseline_damping_2p0_gain_1500_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_damping_2p0_gain_1500_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_trajectory_command_capture_v1/summary.md`
- `diagnostics/research_baseline_trajectory_command_capture_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_canonical_after_command_capture_v1/summary.md`
- `diagnostics/research_baseline_canonical_after_command_capture_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_canonical_after_command_capture_v1/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_moving_to_start_xy_distribution_analyzer_v1/summary.md`
- `diagnostics/research_baseline_jtc_controller_state_observer_v1/summary.md`
- `diagnostics/research_baseline_jtc_controller_state_observer_v1/trajectory_tracking_summary.md`
- `diagnostics/research_baseline_controller_state_tracking_v2/summary.md`
- `diagnostics/research_baseline_controller_state_tracking_v2/moving_to_start_tracking_analysis.md`
- `diagnostics/research_baseline_controller_state_tracking_v2/endpoint_hold_dynamics_analysis.md`
- `diagnostics/research_baseline_controller_state_tracking_v2/trajectory_tracking_summary.md`
- `diagnostics/research_baseline_endpoint_hold_dynamics_analyzer_v1/summary.md`
- `diagnostics/research_baseline_joint_damping_scale_5p0_v1/summary.md`
- `diagnostics/research_baseline_joint_damping_scale_5p0_v1/approach_tracking_analysis.md`
- existing diagnostics under `diagnostics/` and `results/`

## Corrected Documentation Position

Do not use wording such as "first successful autonomous peg-in-hole" for the current state. Use:

**first simulated insertion-depth/contact event, not validated physical success**

until repeated validation demonstrates robust success.

## Current Technical Baseline

- Robot: KUKA LBR iisy 6 R1300 project-local model adaptation.
- SAFE_HOME: `[0.0, -0.8, 1.2, 0.0, 0.8, 0.0]`.
- Spawn: x=0.80, y=-0.75, z=0.735, yaw=1.5708.
- Controller stack: `joint_state_broadcaster` and `joint_trajectory_controller`.
- Canonical controller parameters: `thesis_bringup/config/research_baseline_ros2_control.yaml` loaded by `spawn_robot_sdf.py`.
- Latest validated baseline milestone: `research_baseline_insert_xy_drift_diagnostic_v1` adds an offline INSERT drift analyzer and shows controller feedback can exceed the 25 mm peg / 27 mm hole radial clearance before or during early INSERT. The preceding `research_baseline_insert_sideload_abort_v1` run remains the latest runtime safety behavior: it reported `ABORTED` at depth `0.0011 m` and XY error `0.0032 m`, reducing passive contact-topic rows from `1293` to `2` compared with the prior physical-XY-gate run.
- Previous milestone `research_baseline_insert_physical_xy_gate_v1` reached depth `0.0177 m` and task-side INSERT contact `55.4 N`, but correctly reported `DEGRADED` because final insertion XY error `0.0030 m` exceeds the physical radial clearance of `0.0010 m`.
- Historical milestone `research_baseline_insert_sim_time_completion_v4` reached final outcome `SUCCESS` under older depth/contact criteria, but that wording is now superseded by the physical-clearance gate.
- Timing evidence from that run shows the INSERT command was observed for `23.111 s` after a `20.000 s` command. The prior failed behavior advanced to RETREAT after about `10.6 s` of controller-state INSERT time.
- Success classification now uses `max_insert_contact_force_N`, not global contact, so RETREAT contact cannot create a false insertion success.
- The passive Gazebo contact observer recorded contact-topic rows only in `RETREAT` for the historical v4 depth/contact run; RETREAT contact reached `249.593329 N`. This remains important historical withdrawal evidence.
- A staged vertical-lift-then-home withdrawal diagnostic preserved insertion success but was rejected because it worsened RETREAT contact to `486.746287 N` over `307` rows and raised max raw force norm to `285.8 N`.
- Withdrawal contact timing analysis shows contact occurs during the first post-insert extraction command, before the home command. In the rejected staged run, all `307` contact samples occurred during the vertical lift stage and the peak force occurred while the peg was still inserted `0.019732 m`.
- Physical XY gate validation shows the same issue in the current run: RETREAT peak contact `589.942680 N` occurred at depth `0.017561 m` and XY error `0.005906 m`.
- INSERT side-load abort validation reduced this extraction-contact failure mode: only `2` passive contact-topic rows were recorded, both with `0.000000 N` max force, after aborting at shallow side-loaded insertion.
- INSERT XY drift analysis shows the remaining blocker is not only final success classification: in `research_baseline_insert_sideload_abort_v1`, the controller-state feedback already had pre-command final XY `0.001569 m` and command-window initial XY `0.002539 m`; in `research_baseline_insert_physical_xy_gate_v1`, first side-load occurred at depth `0.001274 m` with XY `0.002153 m`. The next control change should add a no-contact INSERT clearance gate before deeper descent, then address single-point INSERT path drift.
- Older controller-state tracking and endpoint-hold diagnostics remain important historical evidence: canonical pre-damping runs failed the strict above-hole hold gate, while 5x damping moved the blocker downstream to approach/insert timing.
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

- The corrected tool-tip frame removes the reproduced `link_5` target-plate collision, but `MOVING_TO_START` still failed the strict 2 mm no-contact gate. The latest slow-settle diagnostic crossed the gate only transiently and timed out with final XY about 0.011 m.
- Post-tool global gain diagnostics at `position_gain:=2000` and `position_gain:=3000` were safe but rejected. Gain 2000 improved the final timeout to about 2 mm XY but reached only three consecutive strict observer samples; gain 3000 was worse, with final XY about 7 mm and only two consecutive strict observer samples.
- Explicit zero velocity/acceleration trajectory points were safe but rejected. The run reached minimum replayed XY `0.000014 m`, but held only four strict observer samples and timed out at final XY about 14 mm.
- Offline above-hole hold analysis of five post-tool runs confirmed that transient strict-gate crossings are not enough. None reached the required estimated five 10 Hz stable ticks; the best estimated state-loop hold was one tick.
- Selector-based MOVING_TO_START command tracking showed the usable post-tool runs have distributed joint error with persistent final XY drift, not a single dominant joint. Some diagnostics only captured abort-retreat and should not be used for start-command attribution.
- A bounded endpoint-correction experiment was safe but rejected. It accepted three small corrections, reached final timeout XY about 4 mm, and still failed the strict five-tick hold gate.
- A 2x joint-damping diagnostic was safe and improved tracking/hold evidence, but was rejected because the strict gate still failed at `2/5` estimated stable ticks and final XY about 7 mm.
- `APPROACH` currently commands a 67 mm Cartesian descent but measured peg Z remains near 0.90 m instead of reaching the 0.83 m touch target.
- Peak raw Fz spikes are confirmed: 1237.45 N and 3716.2 N were recorded in the 2026-06-01 repeat run.
- Large Cartesian errors during APPROACH remain unresolved.
- Multi-point INSERT is still not reliable.
- Contact/gravity baseline validity needs scenario-specific validation.
- Contact events must be interpreted by collision pair. The latest attributed contact run showed `link_5_collision` hitting the target plate during `MOVING_TO_START`, which is invalid robot-link clearance contact rather than peg insertion contact.
- Older `docs/context/robot_cell_audit.md` contains stale iisy3 statements and should be superseded by `docs/PROJECT_CONTEXT.md` plus `docs/ROBOT_DATASHEET_CHECK.md`.

## Next Milestone

`research_baseline_insert_centering_and_extraction_contact_reduction`

Reason: `research_baseline_insert_sideload_abort_v1` now fails closed when
inserted-depth XY drift exceeds physical clearance, preventing the deeper
invalid insertion and high-force extraction seen in
`research_baseline_insert_physical_xy_gate_v1`. The next safety-critical
improvement should reduce the cause of the side-load abort, especially XY drift
during INSERT, then rerun the same analyzers. Repeated validation should follow
only after the physical success gate is satisfied without triggering this abort.

Historical context: force-safe insert stabilization blocked unsafe INSERT when peg Z was too high, but validation still failed. The 2026-06-01 force-safe validation (`diagnostics/research_baseline_force_safe_insert_v3`) showed:

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

This removed descent/SEARCH from these bad initial alignments and produced complete outcome JSON for all three trials. It did not solve task execution. The next milestone remains above-hole hold/tracking stabilization: improve `MOVING_TO_START` target execution so the corrected peg tip reaches and holds the strict no-contact XY gate (`<=0.002 m`) for the required consecutive state-machine ticks before any descent is attempted.

An above-hole target-refresh experiment was run in `diagnostics/research_baseline_above_hole_tracking_v1`. It was not retained because it worsened safety: two of three trials hard-aborted in `MOVING_TO_START` with raw Fz spikes of 4086.95 N and 1766.64 N, and the remaining trial still failed the no-contact gate at 0.1116 m XY error.

After the tool-tip frame correction removed the reproduced `link_5` target-plate collision, a one-shot 20 s same-target settle was also rejected in `diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1`. It crossed the strict 2 mm gate only transiently, timed out safely in `MOVING_TO_START` with final `xy_err=0.011 m`, recorded zero contact-topic samples, and left the controller source unchanged.

Post-correction global gain diagnostics were then run at `position_gain:=2000`
and `position_gain:=3000`. Both preserved clearance and contact safety, but
both failed the strict above-hole stability gate. Gain 2000 was closest
(`xy_err=0.002 m`, `stable=0/5`, best strict replay streak three samples);
gain 3000 was worse (`xy_err=0.007 m`, best streak two samples). The canonical
gain remains unchanged.

An explicit zero-velocity/zero-acceleration trajectory-point experiment was
then rejected in `diagnostics/research_baseline_zero_derivative_trajectory_hold_v1`.
It remained safe and clearance-clean, but timed out with final `xy_err=0.014 m`
and only four consecutive strict observer samples. The source change was
removed.

An offline above-hole hold analyzer was then added and run over the five
post-tool diagnostics. It writes compact `above_hole_hold_analysis.md/json`
files and estimates the task controller's five-tick 10 Hz stable-gate
requirement from passive `wrench_state_samples.csv`. All analyzed runs failed
the estimated gate despite transient sub-millimetre XY crossings; the best
estimated state-loop hold was one tick.

A selector-based MOVING_TO_START tracking analyzer was then added and run over
the same diagnostics. It selects the axis-align command by FK target pose and
marks retreat-only command logs as missing start-command evidence. The usable
post-tool runs showed final XY drift of `0.012767 m` for slow settle,
`0.003992 m` for gain 2000, and `0.011025 m` for zero-derivative hold, with
worst p95 joint errors spread across `joint_4`, `joint_3`, and `joint_5`
respectively. This does not support another single-joint or broad-gain
diagnostic as the immediate next fix.

A bounded endpoint-correction experiment was then tested in
`diagnostics/research_baseline_start_endpoint_correction_v1` and rejected. It
did not descend, recorded zero contact-topic samples, and kept peak raw force
norm at `270.2 N`, but still aborted in `MOVING_TO_START` with final
`xy_err=0.004 m` and `stable=0/5`. Offline hold analysis still found only one
estimated stable tick. The source change was removed.

A 2x damping diagnostic was then tested in
`diagnostics/research_baseline_joint_damping_scale_2p0_v1` and rejected as a
canonical change. It preserved zero contact-topic samples and reduced peak raw
force norm to `255.5 N`; trajectory tracking improved to p95 max joint error
about `0.0158 rad`, and above-hole hold analysis improved to two estimated
stable ticks. It still aborted in `MOVING_TO_START` with final `xy_err=0.007 m`
and `stable=0/5`, so it does not justify changing canonical damping.

## 2026-06-03 Approach Z-Precondition Gate

Milestone: `research_baseline_approach_z_precondition_gate_v1`

Evidence: `diagnostics/research_baseline_approach_z_precondition_gate_v1/summary.md`

The task controller now requires the preserved force-safe Z precondition before
`APPROACH` can complete. This is a safety tightening: it does not loosen the
strict 2 mm no-contact start gate, the INSERT XY gate, or the hard-force abort.

Validation passed Python syntax, targeted `colcon build --packages-select
kuka_task_control thesis_bringup`, a headless launch with
`joint_damping_scale:=5.0`, and the three passive tracking analyzers.

Runtime evidence:

- `MOVING_TO_START` completed with final command-attributed XY error `0.000812 m`;
- `APPROACH` did not complete while `peg_z=0.8452 m` because the precondition was still false;
- `APPROACH` completed at `peg_z=0.8417 m`;
- `SEARCH` was entered for pre-insertion XY `0.0027 m` and converged to about `0.0010 m`;
- `INSERT` executed, but reported `physical_depth=0.0000 m`;
- approach analyzer reported target `(0.520000, -0.200000, 0.830000)`, final feedback `(0.517594, -0.202990, 0.840723)`, and missing descent `0.010723 m`;
- contact-topic rows began during `RETREAT`, not `INSERT`, and reached `max_contact_force_n=1970.434828`;
- wrench summary recorded `RETREAT` max abs Fz `594.283889 N`.

This is not task success. It is evidence that the phase transition is now more
honest and that the next blocker has moved to insertion-depth/contact
interpretation plus retreat collision safety.

## 2026-06-03 Insert / Retreat Contact Analyzer

Milestone: `research_baseline_insert_retreat_contact_analyzer_v1`

Evidence: `diagnostics/research_baseline_insert_retreat_contact_analyzer_v1/summary.md`

Added a passive offline analyzer for INSERT tracking and RETREAT contact
attribution. It selects the insert command by FK target near the canonical final
insertion pose, computes peg-tip depth relative to `HOLE_TOP_Z=0.810 m`, and
summarizes contact pairs and wrench peaks by state.

Validation passed Python syntax, targeted `colcon build --packages-select
thesis_bringup`, and analyzer execution on
`diagnostics/research_baseline_approach_z_precondition_gate_v1`.

Analyzer result:

- selected INSERT command index `2`;
- target peg-tip pose `(0.520004, -0.200001, 0.790008)`;
- final feedback pose `(0.518227, -0.201338, 0.813985)`;
- missing descent to target `0.023977 m`;
- minimum feedback Z `0.811899 m`;
- max physical insertion depth `0.000000 m`;
- contact-topic rows only attributed to `RETREAT`;
- top RETREAT pair: `grasped_peg_collision_2 <-> target_plate_collision`;
- additional high-force RETREAT pair: `gripper_right_finger_collision_4 <-> target_plate_collision`.

This confirms that zero depth in the latest run is consistent with measured
feedback staying above the plate top. The next implementation should prevent a
failed insert from retreating laterally through the plate; a clearance-lift
segment before moving toward `SAFE_HOME` is the technically next candidate.

## 2026-06-03 Retreat Clearance Lift

Milestone: `research_baseline_retreat_clearance_lift_v1`

Evidence: `diagnostics/research_baseline_retreat_clearance_lift_v1/summary.md`

`RETREAT` now publishes a vertical peg-tip clearance lift before moving toward
`SAFE_HOME`. The lift uses axis-constrained IK waypoints at the current peg XY,
then follows joint-space waypoints to home from the lifted posture. If lift IK
fails, the controller falls back to the old joint-space retreat.

Validation passed Python syntax, targeted `colcon build --packages-select
kuka_task_control thesis_bringup`, a headless launch with
`joint_damping_scale:=5.0`, and passive analyzers.

Runtime result:

- final outcome `DEGRADED`;
- reason `Phase(s) failed: INSERT`;
- depth `0.0008 m`;
- max raw `|Fz|=128.1 N`;
- max raw force norm `208.5 N`;
- final XY error `0.0011 m`;
- phase sequence: MOVING_TO_START OK, APPROACH OK, SEARCH OK, INSERT FAIL, RETREAT OK.

Retreat contact improved materially:

- previous RETREAT contact rows: `7776`;
- new RETREAT contact rows: `4`;
- previous RETREAT max contact force: `1970.434828 N`;
- new RETREAT max contact force: `36.335073 N`;
- previous RETREAT max raw `|Fz|`: `594.283889 N`;
- new RETREAT max raw `|Fz|`: `128.114936 N`.

This is a retreat safety improvement, not insertion success. The next blocker
is insertion-depth realization: the insert target remains near `z=0.790 m`,
but the latest run reached only `0.000836 m` maximum physical depth.

## 2026-06-03 Insert Sim-Time Completion

Milestone: `research_baseline_insert_sim_time_completion_v4`

Evidence: `diagnostics/research_baseline_insert_sim_time_completion_v4/summary.md`

The task controller now evaluates INSERT completion with ROS/Gazebo time, not
control-loop tick count. The launch file passes `use_sim_time` to
`admittance_insertion_node`, matching the controller manager and passive
observers. The controller also records `max_insert_contact_force_N` and uses it
for insert success and final outcome, preventing RETREAT contact from satisfying
the insertion-contact requirement.

Validation passed Python syntax, targeted `colcon build --packages-select
kuka_task_control thesis_bringup`, a headless launch with
`joint_damping_scale:=5.0`, and the passive tracking/contact analyzers.

Runtime result:

- final outcome `SUCCESS`;
- reason `Full cycle completed. Insertion depth 0.019m, contact 60.1N during INSERT`;
- final insertion depth `0.0191 m`;
- max task-side insert contact `60.1 N`;
- max global task-side contact `85.0 N`;
- max raw `|Fz|=133.33 N`;
- max raw force norm `211.14 N`;
- pre-insertion XY error `0.0006 m`;
- phase sequence: MOVING_TO_START OK, APPROACH OK, SEARCH OK, INSERT OK, RETREAT OK.

Controller-state timing:

- INSERT command receipt `57.608 s`;
- RETREAT command receipt `80.719 s`;
- INSERT command duration `20.000 s`;
- observed INSERT window `23.111 s`.

Offline analyzer result:

- final peg-tip feedback `(0.518615, -0.199574, 0.790557)`;
- final physical depth `0.019443 m`;
- maximum physical depth `0.024350 m`;
- INSERT p95 max joint position error `0.008321 rad`.

Limitations:

- This was one simulated insertion-depth/contact trial under older criteria, not robust success, and it is superseded by the physical XY gate.
- Gazebo contact-topic rows were recorded only in `RETREAT` for this run; INSERT contact evidence is from the task F/T estimator.
- RETREAT contact-topic max force was `249.593329 N`, mainly peg versus target plate right collision. Insert centering and extraction-contact reduction are the next safety-critical milestones before repeated-validation claims.

## 2026-06-03 Staged Withdrawal Diagnostic

Milestone: `research_baseline_staged_withdrawal_v1`

Evidence: `diagnostics/research_baseline_staged_withdrawal_v1/summary.md`

A staged vertical-lift-then-home RETREAT was tested to reduce successful-insert
withdrawal contact. The run preserved insertion success:

- final outcome `SUCCESS`;
- insertion depth `0.0208 m`;
- task-side insert contact evidence `59.5 N`;
- phase sequence MOVING_TO_START, APPROACH, SEARCH, INSERT, RETREAT all OK.

However, the staged withdrawal worsened contact evidence:

- v4 RETREAT contact rows `41`;
- staged v1 RETREAT contact rows `307`;
- v4 RETREAT max contact force `249.593329 N`;
- staged v1 RETREAT max contact force `486.746287 N`;
- v4 max raw force norm `211.14 N`;
- staged v1 max raw force norm `285.8 N`.

Decision: rejected and source change removed. The next withdrawal fix should
inspect fixture/hole contact geometry and peg load during vertical extraction
rather than only splitting the trajectory into lift and home stages.

## 2026-06-03 Withdrawal Contact Timing

Milestone: `research_baseline_withdrawal_contact_timing_v1`

Evidence: `diagnostics/research_baseline_withdrawal_contact_timing_v1/summary.md`

The new `withdrawal_contact_timing_analyzer` reads recorded
`trajectory_commands.csv`, `contact_state_samples.csv`, and controller-state
tracking feedback, then attributes positive contact rows to the active command
window and peg-tip feedback pose.

Validation:

- Python syntax passed for the analyzer;
- `colcon build --symlink-install --packages-select thesis_bringup` passed;
- analyzer ran on `research_baseline_insert_sim_time_completion_v4` and
  `research_baseline_staged_withdrawal_v1`.

Findings:

- v4: all `41` positive contact samples occurred in `RETREAT_1`; first contact
  was `0.554 s` after RETREAT receipt while the peg was still inserted
  `0.022622 m`;
- v4 peak contact was `249.593329 N` at depth `0.003927 m` and XY error
  `0.005688 m`, peg versus target plate right collision;
- staged v1: all `307` positive contact samples occurred in the vertical
  `RETREAT_1` command; no contact rows were attributed to the later
  `RETREAT_2` home command;
- staged v1 peak contact was `486.746287 N` at depth `0.019732 m` and XY error
  `0.005979 m`, peg versus target plate left collision.

Interpretation: successful-insert withdrawal contact is generated during
initial extraction while the peg is still inside or near the hole. The next
implementation should address side-loaded extraction/contact geometry rather
than only splitting or delaying home motion.

## 2026-06-03 Insert Physical XY Gate

Milestone: `research_baseline_insert_physical_xy_gate_v1`

Evidence: `diagnostics/research_baseline_insert_physical_xy_gate_v1/summary.md`

The task success contract now includes final INSERT XY error against the actual
task geometry. With a 25 mm peg and 27 mm hole, radial clearance is `0.0010 m`.
`admittance_insertion_node` records `final_insertion_xy_error_m` and reports
`DEGRADED` when depth/contact are present but the peg is side-loaded beyond
that clearance.

Validation:

- Python syntax passed;
- targeted `colcon build --symlink-install --packages-select kuka_task_control thesis_bringup` passed;
- headless launch reached task `DONE`;
- moving-to-start, endpoint-hold, approach, insert/retreat, and withdrawal timing analyzers ran on the new diagnostic directory.

Runtime result:

- final outcome `DEGRADED`;
- reason `Final insertion XY error (0.0030m) exceeds physical hole clearance (0.0010m). Peg is side-loaded; do not count as physical success.`;
- insertion depth `0.0177 m`;
- max task-side INSERT contact `55.4 N`;
- final insertion XY error `0.0030 m`;
- max raw `|Fz|=227.09 N`;
- max raw force norm `234.96 N`;
- INSERT contact-topic rows `6`, max `130.162091 N`;
- RETREAT contact-topic rows `1287`, max `589.942680 N`.

Decision: this supersedes the earlier v4 success wording. The current baseline
has a controller-driven insertion-depth/contact event, not validated physical
success. The next implementation should reduce inserted-depth XY drift and
side-loaded extraction contact before repeated-validation or learning claims.

## 2026-06-03 Insert Sideload Abort

Milestone: `research_baseline_insert_sideload_abort_v1`

Evidence: `diagnostics/research_baseline_insert_sideload_abort_v1/summary.md`

The task now aborts INSERT when the peg is at least `0.0010 m` below the hole
top and XY error exceeds the physical radial clearance `0.0010 m` for `3`
consecutive control ticks.

Validation:

- Python syntax passed;
- targeted `colcon build --symlink-install --packages-select kuka_task_control thesis_bringup` passed;
- headless launch reached task `DONE`;
- moving-to-start, endpoint-hold, approach, insert/retreat, and withdrawal timing analyzers ran.

Runtime result:

- final outcome `ABORTED`;
- reason `INSERT aborted: side-loaded peg at depth 0.0011m with XY error 0.0032m, exceeding physical clearance 0.0010m for 3 ticks.`;
- insertion depth metric `0.0038 m`;
- max task-side INSERT contact `53.75 N`;
- max raw `|Fz|=128.53 N`;
- max raw force norm `204.14 N`;
- passive contact-topic rows `2`;
- max passive contact-topic force `0.000000 N`.

Comparison: the prior physical-XY-gate run recorded `1293` passive contact rows
and RETREAT contact max `589.942680 N`. This side-load abort is a safety
improvement and an honest failure, not task success. The next implementation
should reduce the inserted-depth XY drift that triggers the abort.

## 2026-06-03 Insert XY Drift Diagnostic

Milestone: `research_baseline_insert_xy_drift_diagnostic_v1`

Evidence: `diagnostics/research_baseline_insert_xy_drift_diagnostic_v1/summary.md`

Added `insert_xy_drift_analyzer`, a passive offline analyzer for INSERT command
windows. It selects the INSERT command by FK target, uses
`trajectory_controller_state_samples.csv` when available, reconstructs peg-tip
Cartesian feedback, and reports pre-command boundary XY, first
physical-clearance violation, first meaningful depth, first side-load sample,
and nearest F/T/contact values.

Validation passed:

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/insert_xy_drift_analyzer.py src/thesis_bringup/setup.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select thesis_bringup
source install/setup.bash
ros2 run thesis_bringup insert_xy_drift_analyzer diagnostics/research_baseline_insert_sideload_abort_v1
ros2 run thesis_bringup insert_xy_drift_analyzer diagnostics/research_baseline_insert_physical_xy_gate_v1
```

Results:

- current side-load abort run: pre-command final XY `0.001569 m`, command-window initial XY `0.002539 m`, first meaningful depth `0.001316 m` at XY `0.002488 m`;
- prior physical-XY-gate run: first meaningful depth `0.001103 m` at XY `0.000323 m`, but first side-load at depth `0.001274 m` with XY `0.002153 m`;
- both runs use one-point INSERT commands.

Interpretation: the baseline can violate physical radial clearance before or
during early INSERT. Do not loosen side-load or final XY gates. The next
implementation should add a no-contact INSERT clearance gate before deeper
descent, then reduce single-point INSERT path drift.

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

## 2026-06-02 Axis-Aligned Start And Search Fail-Closed

Milestone: `research_baseline_search_fail_closed_v2`

Evidence: `diagnostics/research_baseline_search_fail_closed_v2/summary.md`

The controller now gives the safer axis-aligned vertical-peg `MOVING_TO_START` posture a scoped 120 s timeout. This is not a descent or insertion gate relaxation; the strict above-hole XY requirement remains 0.002 m. Validation reached the strict gate:

- `MOVING_TO_START`: success, duration `96.3 s`;
- above-hole XY error: `0.0018 m`;
- Cartesian error: `0.012606 m`;
- joint error: `0.063314 rad`.

The same validation then exposed the current blocker in `APPROACH`: the controller commanded a 67 mm descent from about `z=0.897 m` to `z=0.830 m`, but the measured peg stayed near `z=0.90 m` and XY drifted to about `0.018 m`. `APPROACH` timed out at 90 s and transitioned directly to `ABORT`:

- outcome: `ABORTED`;
- reason: `APPROACH timeout/degraded failure (90.0s). cart_err=0.072m, joint_err=0.107rad, tolerance=0.050m`;
- insertion depth: `0.0000 m`;
- peak raw `|Fz|`: `691.37 N`;
- peak raw force norm: `703.09 N`.

The previous unsafe behavior where a degraded approach could enter local `SEARCH` has been removed. In the corrected run, observer summaries contain no `SEARCH` rows. `SEARCH` is now limited to a completed approach at force-safe Z with residual XY within the bounded search radius.

## 2026-06-02 Slow Approach Descent Experiment

Milestone: `research_baseline_slow_approach_descent_v1`

Evidence: `diagnostics/research_baseline_slow_approach_descent_v1/summary.md`

A timing-only approach experiment increased the descent command duration from the default 15 s minimum to `41.662 s`. It was rejected and reverted.

Result:

- `MOVING_TO_START`: success after `95.6 s`, initial XY error `0.0015 m`;
- `APPROACH`: failed at 90 s with `cart_err=0.070 m`, `joint_err=0.108 rad`;
- insertion depth: `0.0000 m`;
- peak raw `|Fz|`: `570.07 N`;
- peak force norm: `627.03 N`;
- no `SEARCH` phase was entered.

The slower trajectory reduced neither the blocking joint error nor the missing Z descent enough to matter. The next investigation should focus on why the approach target command leaves `joint_2` roughly `0.108 rad` away from target under Gazebo/`gz_ros2_control`, not on further timing-only changes.

## 2026-06-02 Approach Gain 3000 Diagnostic

Milestone: `research_baseline_approach_gain_3000_v1`

Evidence: `diagnostics/research_baseline_approach_gain_3000_v1/summary.md`

A high-gain Gazebo position-controller diagnostic ran with `position_gain:=3000`. It was rejected and not retained as the canonical launch setting.

Result:

- Gazebo confirmed `position_proportional_gain=3000`;
- `MOVING_TO_START`: success after `92.6 s`, initial XY error `0.0014 m`;
- `APPROACH`: failed at 90 s with `cart_err=0.073 m`, `joint_err=0.110 rad`;
- insertion depth: `0.0000 m`;
- peak raw `|Fz|`: `842.85 N`;
- peak force norm: `890.27 N`;
- no `SEARCH` phase was entered.

The higher gain slightly reduced time to the above-hole gate, but it worsened approach tracking and increased peak wrench. The next milestone remains a dynamics/controller investigation around `joint_2`, not simple gain increase.

## 2026-06-02 Joint 2 Approach Tracking Diagnostic

Milestone: `research_baseline_joint2_approach_tracking_diagnostic`

Evidence:

- `diagnostics/research_baseline_search_fail_closed_v2/approach_tracking_analysis.md`
- `diagnostics/research_baseline_slow_approach_descent_v1/approach_tracking_analysis.md`
- `diagnostics/research_baseline_approach_gain_3000_v1/approach_tracking_analysis.md`

Added `thesis_bringup.approach_tracking_analyzer`, an offline analyzer for the passive trajectory observer CSVs. It uses named joints from `trajectory_commands.csv` and `trajectory_tracking_samples.csv`, computes per-joint approach error statistics, and maps final feedback through the local KUKA LBR iisy6 R1300 peg-tip kinematics.

Validation passed:

- `python3 -m py_compile src/thesis_bringup/thesis_bringup/approach_tracking_analyzer.py`;
- targeted `colcon build --symlink-install --packages-select thesis_bringup`;
- analyzer runs over the three recent approach-failure diagnostic directories.

Cross-run result:

| Run | p95 joint_2 abs error | final joint_2 error | final Cartesian error | missing descent |
|---|---:|---:|---:|---:|
| `research_baseline_search_fail_closed_v2` | `0.108733 rad` | `0.107360 rad` | `0.072220 m` | `-0.069840 m` |
| `research_baseline_slow_approach_descent_v1` | `0.106741 rad` | `0.108545 rad` | `0.070156 m` | `-0.067465 m` |
| `research_baseline_approach_gain_3000_v1` | `0.110990 rad` | `0.110880 rad` | `0.072500 m` | `-0.069807 m` |

The approach command target is consistently the correct peg-tip touch pose near `0.520, -0.200, 0.830 m`. Runtime feedback remains near `z=0.897-0.900 m`, so the blocked descent is a controller/physics/joint-authority issue dominated by `joint_2`, not an unreachable or wrongly computed Cartesian target.

## 2026-06-02 Joint Damping Scale 0.2 Diagnostic

Milestone: `research_baseline_joint_damping_scale_0p2_v1`

Evidence: `diagnostics/research_baseline_joint_damping_scale_0p2_v1/summary.md`

`spawn_robot_sdf` now supports diagnostic-only launch-time scaling of converted SDF joint damping and effort limits. Defaults remain `joint_damping_scale:=1.0` and `joint_effort_scale:=1.0`, preserving canonical robot dynamics unless a run explicitly overrides them.

The first dynamics diagnostic used `joint_damping_scale:=0.2`, leaving position gain, effort limits, and all task safety gates unchanged. The converted SDF damping overrides were:

- arm joints: `30/30/20/10/10/5 -> 6/6/4/2/2/1`;
- `ft_sensor_joint`: `1 -> 0.2`.

Result: rejected.

- outcome: `ABORTED`;
- reason: hard-force abort in `MOVING_TO_START`, `|Fz|=1181.0 N`, `|F|=1272.7 N`;
- insertion depth: `0.0000 m`;
- phase Cartesian error at abort: `0.272028 m`;
- trajectory tracking p95 max joint error: `0.113972 rad`;
- contact observer recorded no bridged contact samples.

Broad damping reduction did not reach the no-contact gate or approach phase. It is useful diagnostic evidence, but not a canonical fix. The next controller/physics investigation should be more targeted than global damping reduction.

## 2026-06-02 Joint Effort Scale 2.0 Diagnostic

Milestone: `research_baseline_joint_effort_scale_2p0_v1`

Evidence: `diagnostics/research_baseline_joint_effort_scale_2p0_v1/summary.md`

The second dynamics diagnostic used `joint_effort_scale:=2.0`, leaving damping, position gain, and all task safety gates unchanged. The converted SDF doubled all arm-joint effort limits; `joint_2` changed from about `199.605 Nm` to `399.21 Nm`.

Result: rejected.

- outcome: `ABORTED`;
- `MOVING_TO_START`: success after `81.4 s`, initial XY error `0.0007 m`, Cartesian error `0.006955 m`;
- `APPROACH`: hard-force abort after `0.5 s`, `|Fz|=968.4 N`, force norm `1009.7 N`;
- total max raw `|Fz|`: `1020.10 N`;
- total max raw force norm: `1043.00 N`;
- insertion depth: `0.0000 m`;
- trajectory tracking p95 max joint error: `0.048923 rad`;
- command-index 1 approach analysis: peg still at `z=0.890982 m` against the `z=0.830000 m` target at abort, with final target-minus-feedback `joint_2` error `0.099473 rad`;
- contact observer recorded target-source contact rows in `MOVING_TO_START`, `APPROACH`, and `ABORT`, with max target contact force `9925.518339 N`.

This diagnostic shows effort authority is involved, but doubled effort is unsafe and not a fix. The next investigation should localize why force/contact evidence appears immediately at approach start when XY is valid and the command target is a short vertical descent.

## 2026-06-02 Contact Pair Attribution

Milestone: `research_baseline_contact_pair_attribution_v1`

Evidence: `diagnostics/research_baseline_contact_pair_attribution_v1/summary.md`

`contact_state_observer` now records exact Gazebo collision pairs in its CSV and compact summary. The observer remains passive and does not alter controller behavior.

Runtime result with `joint_effort_scale:=2.0`:

- outcome: `ABORTED`;
- state at abort: `MOVING_TO_START`;
- reason: hard-force abort at `|Fz|=1018.9 N`, force norm `1195.8 N`;
- insertion depth: `0.0000 m`;
- attributed collision pair: `lbr_iisy6_r1300::link_5::link_5_collision <-> target_plate::plate_link::target_plate_collision`;
- no peg-source or hole-source contact rows were recorded.

This shows at least one unsafe high-force path is caused by robot-link clearance contact with the target plate before descent. It is not valid insertion contact and must not be counted as progress. The next safety-critical milestone is clearance-aware `MOVING_TO_START` geometry/path validation before further approach or insertion tuning.

## 2026-06-02 Tool Tip Frame Correction

Milestone: `research_baseline_tool_tip_frame_correction_v1`

Evidence: `diagnostics/research_baseline_tool_tip_frame_correction_v1/summary.md`

The research gripper `peg_tip` frame was corrected from the near-palm end of the 110 mm peg to the protruding negative local tool-Z end. The URDF now places the fingers at `z=-0.055 m`, the peg center at `z=-0.075 m`, and both `gripper_tcp` and `peg_tip` at `z=-0.130 m`. `RobotKinematics` now uses the matching `link_6 -> peg_tip` offset.

Validation passed for Python syntax, xacro expansion, targeted package build, offline clearance analysis, and a canonical 240 s headless launch.

Runtime result:

- outcome: `ABORTED`;
- reason: `MOVING_TO_START timeout/failure (120.0s)`;
- final phase Cartesian error: `0.015238 m`;
- final phase joint error: `0.022126 rad`;
- final logged XY error: about `0.014 m`;
- insertion depth: `0.0000 m`;
- max raw force norm: `270.82 N`;
- contact observer samples: `0`.

The clearance analyzer reported `0/201` planned and `0/1338` runtime-feedback `link_5` target-plate intersections, with closest sampled runtime-feedback clearance `0.117041 m`. The previous clearance collision is therefore resolved for this run. The next blocker is final above-hole XY stabilization/settling, not contact search or insertion.

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

## 2026-06-02 Slow Move-To-Start Timing Rejected

Milestone: `research_baseline_slow_move_to_start_rejected`

Evidence: `diagnostics/research_baseline_slow_move_to_start_tracking/summary.md`

A temporary slower and denser no-contact `MOVING_TO_START` trajectory was tested and then reverted. The tested command used `duration=45.6s`, `waypoints=17`, and `dist=0.6517`; it delayed arrival near the above-hole target and still failed the strict stability gate.

Runtime result:

- `Outcome: ABORTED`;
- `Reason: MOVING_TO_START timeout/failure (90.0s)`;
- `cart_err=0.012 m`;
- `xy_err=0.011 m`;
- `joint_err=0.015 rad`;
- `stable=0/5`;
- `Depth: 0.0000 m`;
- `Max Fz: 169.4 N`.

Decision: rejected and reverted. The retained code keeps the prior move-to-start timing. The next milestone should target final hold/stabilization near the above-hole pose or controller/physics parameters, not a globally slower no-contact approach.

## 2026-06-02 Move-To-Start Hold Correction Rejected

Milestone: `research_baseline_move_to_start_hold_correction`

Evidence: `diagnostics/research_baseline_move_to_start_hold_correction/summary.md`

A temporary bounded final hold correction was tested and then removed. The
experiment allowed up to three same-target hold commands after the original
`MOVING_TO_START` trajectory if the peg was already within a 30 mm XY window.

Runtime result:

- `Outcome: ABORTED`;
- `Reason: MOVING_TO_START timeout/failure (90.0s)`;
- `cart_err=0.016 m`;
- `xy_err=0.011 m`;
- `joint_err=0.014 rad`;
- `stable=0/5`;
- `Depth: 0.0000 m`;
- `Max Fz: 170.8 N`.

The hold commands briefly reduced XY error to `0.0028 m` and `0.0008 m`, but
the pose did not remain stable for the required consecutive samples and drifted
back outside the strict 2 mm no-contact gate. Passive tracking evidence showed
p95 max joint error `0.026865 rad` and final max joint error `0.030123 rad`.

Decision: rejected and reverted. The strict above-hole stability gate remains
unchanged. The next blocker is runtime tracking/physics and high free-space
F/T behavior near the above-hole target, not repeated same-target commands or
weaker descent criteria.

## 2026-06-02 Raw Wrench Abort Instrumentation

Milestone: `research_baseline_raw_wrench_abort`

Evidence: `diagnostics/research_baseline_raw_wrench_abort/summary.md`

The canonical launch now starts a passive `wrench_state_observer` by default.
It records `/ft_sensor_wrench` grouped by `/insertion_state` and peg pose. The
task controller also now tracks raw wrench peaks in the wrench callback and
latches hard-force aborts on raw `|Fz|` or force norm above `1000 N` in active
task states, including `MOVING_TO_START`.

Validation passed for Python syntax, targeted `colcon build`, and a 150 s
headless launch. Runtime result:

- `Outcome: ABORTED`;
- `Reason: Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START`;
- `Max |Fz|=1943.3 N`;
- `Max |F|=2936.5 N`;
- `Depth: 0.0000 m`.

The passive observer independently recorded `MOVING_TO_START` max abs Fz
`1943.293077 N` and max force norm `2936.479541 N`. This is a safety improvement
and a clearer failure mode, not task success.

The next blocker is to determine whether these high free-space raw wrench
spikes are hidden contact, force-torque sensor semantics, inertial dynamics from
the free-space trajectory, or Gazebo/controller physics. Descent, search,
insertion, and learning should remain blocked until this is understood or
bounded by evidence.

## 2026-06-02 Contact-Wrench Correlation

Milestone: `research_baseline_contact_wrench_correlation`

Evidence: `diagnostics/research_baseline_contact_wrench_correlation/summary.md`

The canonical launch now also starts a passive `contact_state_observer` by
default. It subscribes to `/gazebo/contacts/peg`, `/gazebo/contacts/hole`, and
`/gazebo/contacts/target`, groups messages by `/insertion_state`, and writes
compact contact summaries.

Validation passed for Python syntax, targeted `colcon build`, and a 150 s
headless launch. Runtime result:

- `Outcome: ABORTED`;
- `Reason: Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START`;
- `Max |Fz|=1396.8 N`;
- `Max |F|=2624.1 N`;
- `Depth: 0.0000 m`.

Contact observer result:

- samples: `0`;
- positive contact samples: `0`;
- max contact force from contact topics: `0.000000 N`.

This does not prove every possible collision pair was contact-free, but it does
show that the canonical peg/hole/target contact topics did not provide positive
contact evidence for the raw wrench spike. The next investigation should focus
on FT sensor semantics, inertial/dynamic loads from the free-space trajectory,
uninstrumented collision pairs, or Gazebo/controller physics.

## 2026-06-02 F/T Mount Effort-Limit Validation

Milestone: `research_baseline_ft_mount_effort_limit`

Evidence: `diagnostics/research_baseline_ft_mount_effort_limit/summary.md`

The F/T mount remains a zero-range revolute joint because URDF fixed joints are
collapsed by `gz sdf -p`, which removes the named joint needed by the
joint-level Gazebo force-torque sensor. The previous measurement-joint limit of
`effort=1`, `velocity=0` was corrected to `effort=10000`, `velocity=100` while
preserving lower/upper limits at `0`.

Validation passed for Python syntax, xacro expansion, URDF-to-SDF conversion,
targeted `colcon build`, and a 150 s headless launch. Runtime result:

- `Outcome: ABORTED`;
- `Reason: Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START`;
- `Max |Fz|=612.25 N`;
- `Max |F|=1765.41 N`;
- `Depth: 0.0000 m`.

Observer result:

- wrench samples: `7310`;
- contact-topic samples: `0`;
- trajectory max absolute joint-position error: `0.048581 rad`;
- trajectory p95 max absolute joint-position error: `0.028216 rad`.

This reduced the raw wrench spike compared with the preceding
contact-correlation run (`Max |Fz|=1396.8 N`, `Max |F|=2624.1 N`), but did not
produce insertion or clear the hard-force safety gate. The next blocker is to
localize late `MOVING_TO_START` force norm spikes from uninstrumented collision
pairs, tool/peg/table geometry proximity, F/T joint semantics, or
controller/physics dynamics near the above-hole target.

## 2026-06-02 Full-Path Contact Bridge Validation

Milestone: `research_baseline_contact_bridge_full_paths`

Evidence: `diagnostics/research_baseline_contact_bridge_full_paths/summary.md`

The canonical contact bridge now maps fully scoped Gazebo sensor topics to the
stable ROS topics consumed by `contact_state_observer`. The active robot-mounted
peg is also instrumented by injecting `peg_contact_sensor` onto the converted
`ft_sensor_link`, referencing the lumped grasped-peg collision.

Validation passed for Python syntax, generated-SDF inspection, targeted
`colcon build`, and a 150 s headless launch. Runtime logs showed all three
contact bridges created and Gazebo publishing all three contact sensors.

The task outcome remained a bounded safety failure:

- `Outcome: ABORTED`;
- `Reason: Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START`;
- `Max |Fz|=939.36 N`;
- `Max |F|=1748.50 N`;
- `Depth: 0.0000 m`.

Contact observer result:

- contact samples: `50`;
- positive contact samples: `50`;
- `MOVING_TO_START` peg max contact force: `2427.307742 N`;
- `MOVING_TO_START` target max contact force: `2427.307742 N`;
- no hole contact samples were recorded.

The previous zero-contact conclusion was therefore an observability gap. The
current blocker is no-contact start-pose geometry: before descent, the peg can
contact the target plate while hovering near the above-hole target. The next
milestone should correct the free-space start pose or clearance geometry without
loosening the hard-force abort or strict no-contact stability gate.

## 2026-06-02 Axis-Aligned Start Pose

Milestone: `research_baseline_axis_aligned_start_pose`

Evidence: `diagnostics/research_baseline_axis_aligned_start_pose/summary.md`

The previous above-hole target used position-only IK. Offline FK showed the peg
tip at the target but the peg body tilted strongly into the target area. The
task controller now uses joint-limit-aware axis-aligned IK for Cartesian task
targets, constraining peg local +Z to world +Z.

Validation passed for Python syntax, offline IK checks, targeted `colcon build`,
and a 150 s headless launch. Runtime result:

- `Outcome: ABORTED`;
- `Reason: MOVING_TO_START timeout/failure (90.0s)`;
- final phase `cart_err=0.011498 m`;
- final logged `xy_err=0.006 m`;
- final `joint_err=0.026354 rad`;
- `Max |Fz|=554.24 N`;
- `Max |F|=628.61 N`;
- `Depth: 0.0000 m`.

This is a safety improvement, not task success. The previous raw hard-force
abort did not occur, and the contact observer recorded no peg-source contact
rows. The target-source contact rows are not sufficient evidence of peg contact
because the target plate also has support/fixture contacts.

The new blocker is convergence of the larger axis-aligned no-contact move. It
requires a `2.4145 rad` joint-space move and did not satisfy the strict 2 mm XY
stability gate before timeout. The next milestone should improve start-pose
trajectory timing/settling or split the move through a clear staging posture,
while preserving the hard-force abort and no-contact gate.
