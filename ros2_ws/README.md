# ROS 2 Jazzy / Gazebo Peg-in-Hole Research Workspace

Current status as of 2026-06-03: this is an active ROS 2 Jazzy workspace for a Gazebo-based KUKA LBR iisy 6 R1300 peg-in-hole research baseline. The project has a working robot spawn path, active ros2_control controllers, a fixed grasped peg model, a fixed hole fixture, force/torque bridge plumbing, contact observability, and an admittance-style insertion controller.

The strongest historical iisy6 evidence is a controller-driven simulated insertion-depth event from `diagnostics/research_baseline_insert_sim_time_completion_v4`: final outcome `SUCCESS` under older depth/contact criteria, insertion depth `0.0191 m`, task F/T insert-contact evidence `60.1 N`, max raw `|Fz|=133.33 N`, and no safety abort or invalid timeout. That claim is now superseded by a stricter physical-clearance gate: `diagnostics/research_baseline_insert_physical_xy_gate_v1` reached depth `0.0177 m` and insert contact `55.4 N`, but correctly reported `DEGRADED` because final insertion XY error `0.0030 m` exceeds the 25 mm peg / 27 mm hole radial clearance of `0.0010 m`. Current status: no validated physical insertion success under the latest criteria.

Latest timing evidence shows the prior failed insert was partly a clock-domain bug: the task advanced to RETREAT after about `10.6 s` of controller-state INSERT time despite commanding a `20 s` trajectory. The current task node uses ROS/Gazebo time for INSERT completion and now checks final insertion XY against physical hole clearance. The latest runtime gate aborts INSERT before meaningful depth when no-contact XY feedback exceeds the `0.001 m` physical radial clearance, so future work should reduce or constrain single-point INSERT path drift; do not treat depth/contact alone as robust autonomous peg-in-hole performance.

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
| research_baseline_start_slow_settle_after_tool_fix_v1 | Rejected: one 20 s same-target settle did not satisfy the strict above-hole stability gate |
| research_baseline_start_gain_2000_after_tool_fix_v1 | Rejected: gain 2000 improved final error but did not hold the 2 mm gate |
| research_baseline_start_gain_3000_after_tool_fix_v1 | Rejected: gain 3000 was worse than gain 2000 for strict-gate stability |
| research_baseline_zero_derivative_trajectory_hold_v1 | Rejected: explicit zero velocity/acceleration trajectory points still failed above-hole hold |
| research_baseline_above_hole_hold_analyzer | Completed: reusable offline analyzer confirms recent post-tool runs only cross the strict gate transiently |
| research_baseline_moving_to_start_tracking_analyzer | Completed: reusable selector-based analyzer localizes post-tool start hold drift without false retreat-command attribution |
| research_baseline_start_endpoint_correction_v1 | Rejected: bounded endpoint corrections improved final XY but still failed the five-tick strict hold gate |
| research_baseline_joint_damping_scale_2p0_v1 | Rejected: 2x damping improved tracking and best hold to two ticks, but still failed the strict gate |
| research_baseline_damping_2p0_gain_1500_v1 | Rejected: 2x damping plus gain 1500 still failed the strict hold gate and only captured retreat-command tracking |
| research_baseline_trajectory_command_capture_v1 | Completed: bounded first-command discovery wait restores command-attributed MOVING_TO_START tracking logs |
| research_baseline_canonical_after_command_capture_v1 | Failed safely: canonical run captures MOVING_TO_START command but still fails the strict above-hole hold gate |
| research_baseline_moving_to_start_xy_distribution_analyzer_v1 | Completed: command-attributed analyzer now reports XY distribution and final-window oscillation |
| research_baseline_jtc_controller_state_observer_v1 | Completed: trajectory observer now subscribes to JTC `controller_state` and records nonzero state samples |
| research_baseline_controller_state_tracking_v2 | Failed safely: canonical run records JTC controller-state tracking but still times out before descent |
| research_baseline_endpoint_hold_dynamics_analyzer_v1 | Completed: endpoint hold analyzer shows multi-centimeter hold oscillation and zero strict 10 Hz bins |
| research_baseline_joint_damping_scale_5p0_v1 | Improved but failed safely: 5x damping reaches APPROACH but INSERT remains blocked by Z precondition |
| research_baseline_approach_z_precondition_gate_v1 | Completed: APPROACH must satisfy INSERT Z precondition before SEARCH/INSERT |
| research_baseline_insert_retreat_contact_analyzer_v1 | Completed: failed INSERT depth and RETREAT collision attribution analyzer added |
| research_baseline_retreat_clearance_lift_v1 | Completed: failed-insert RETREAT contact reduced; INSERT still failed |
| research_baseline_insert_sim_time_completion_v4 | Superseded: depth/contact success under older criteria; physical XY gate added later |
| research_baseline_staged_withdrawal_v1 | Rejected: staged lift/home preserved success but worsened RETREAT contact |
| research_baseline_withdrawal_contact_timing_v1 | Completed: RETREAT contact occurs during first extraction command before home motion |
| research_baseline_insert_physical_xy_gate_v1 | Completed: depth/contact event correctly downgraded because final inserted XY exceeds physical clearance |
| research_baseline_insert_sideload_abort_v1 | Completed: INSERT aborts safely when inserted-depth XY exceeds physical clearance |
| research_baseline_insert_xy_drift_diagnostic_v1 | Completed: analyzer shows INSERT XY can violate physical clearance before or during early descent |
| research_baseline_insert_precontact_clearance_gate_v1 | Completed: INSERT aborts before meaningful depth when no-contact XY exceeds physical clearance |
| research_baseline_insert_cartesian_descent_v1 | Rejected: multi-waypoint Cartesian INSERT still violated clearance immediately; source reverted |
| research_baseline_insert_handoff_reference_v1 | Completed: analyzer shows centered INSERT reference but feedback leaves physical clearance within 5 ms |
| research_baseline_insert_handoff_settle_v1 | Completed safety gate: final INSERT descent is withheld unless handoff feedback is stable; validation still aborted before depth |
| research_baseline_search_sustained_clearance_v1 | Completed safety gate: SEARCH now requires sustained physical clearance; validation fails closed in SEARCH |
| research_baseline_search_centered_hold_v1 | Rejected: centered SEARCH hold remained unstable and source was reverted |
| research_baseline_xy_stability_analyzer_v1 | Completed: per-state passive-log analyzer confirms recent SEARCH runs only sustain 1 mm clearance for two estimated control ticks |
| research_baseline_search_recenter_on_coarse_band_v1 | Improved but failed safely: SEARCH recenters inside 2 mm coarse band; best 1 mm window improved to four ticks but INSERT remains blocked |
| research_baseline_search_recenter_4mm_v1 | Improved but failed safely: bounded 4 mm recenter reduces final SEARCH XY to 1.7 mm but still does not satisfy the 1 mm sustained gate |
| research_baseline_hold_window_reference_analyzer_v1 | Completed: command-window analyzer shows centered hold references but feedback still fails sustained 1 mm clearance |

## 2026-06-03 Hold Window Reference Analyzer

Milestone: `research_baseline_hold_window_reference_analyzer_v1`

Evidence: `diagnostics/research_baseline_hold_window_reference_analyzer_v1/summary.md`

The workspace now includes `hold_window_reference_analyzer`, an offline
diagnostic that groups `trajectory_controller_state_samples.csv` by
`trajectory_commands.csv` command windows and reconstructs peg-tip Cartesian
reference/feedback for each command. Single-point trajectories lasting at least
1 s are reported as hold-like command windows.

Validation passed Python syntax, targeted `colcon build --packages-select
thesis_bringup`, and analyzer runs over the two 4 mm recenter diagnostics:

- `diagnostics/research_baseline_search_recenter_4mm_v1/hold_window_reference_analysis.md`;
- `diagnostics/research_baseline_search_recenter_4mm_v1_repeat2/hold_window_reference_analysis.md`.

Findings:

- the direct INSERT handoff hold in `research_baseline_search_recenter_4mm_v1`
  had a centered target but only `3` estimated feedback ticks inside
  `0.0010 m`;
- the repeated SEARCH recenter run observed `7` hold-like commands, but the
  best feedback hold was only `2` estimated ticks inside `0.0010 m`;
- centered references frequently stayed inside clearance longer than feedback,
  so the remaining blocker is feedback stabilization/command sequencing, not a
  reason to loosen the physical gate.

No insertion success is claimed from this milestone.

## 2026-06-03 Controller-State Tracking V2

Milestone: `research_baseline_controller_state_tracking_v2`

Evidence: `diagnostics/research_baseline_controller_state_tracking_v2/summary.md`

The passive trajectory observer now writes
`trajectory_controller_state_samples.csv` from
`/joint_trajectory_controller/controller_state`, and
`moving_to_start_tracking_analyzer` prefers that controller-state source when
present. This removes the remaining ambiguity between locally interpolated
command reference and the JTC's own reference/feedback/error stream.

Validation passed Python syntax and targeted `colcon build --packages-select
thesis_bringup`. A canonical 190 s headless run reached the task node's own
final outcome before the wrapper timeout:

- `Outcome: ABORTED`;
- `Reason: MOVING_TO_START timeout/failure (120.0s). cart_err=0.018m, xy_err=0.013m, joint_err=0.017rad, stable=0/5`;
- insertion depth `0.0000 m`;
- JTC controller-state records `23878`;
- analyzer tracking source `trajectory_controller_state_samples.csv`;
- strict XY samples `143 / 15930`;
- final 1 s XY range `0.000589-0.023077 m`;
- contact-topic samples `0`;
- max raw force norm `272.806449 N`.

The instrumentation is validated, but the canonical baseline remains blocked
before descent. Do not claim insertion success from this run and do not loosen
the strict no-contact gate.

## 2026-06-03 Endpoint Hold Dynamics Analyzer

Milestone: `research_baseline_endpoint_hold_dynamics_analyzer_v1`

Evidence: `diagnostics/research_baseline_endpoint_hold_dynamics_analyzer_v1/summary.md`

The workspace now includes `endpoint_hold_dynamics_analyzer`, an offline
diagnostic that isolates the post-axis-align hold window from passive
trajectory logs. It prefers JTC controller-state rows when present and reports
Cartesian hold ranges, strict-gate occupancy, estimated 10 Hz strict bins, and
per-joint feedback ranges.

Running it on `research_baseline_controller_state_tracking_v2` showed that the
post-command hold is not a small static offset:

- hold duration `23.716 s`;
- XY error mean `0.009221 m`;
- XY error p95 `0.015782 m`;
- X/Y/Z ranges `0.041273 / 0.035843 / 0.033742 m`;
- strict XY samples `140 / 5930`;
- strict 10 Hz bins `0`;
- largest joint feedback range `joint_1`, `0.057121 rad`.

This supports endpoint hold dynamics or damping-authority work as the next
technical step. It does not support relaxing the 2 mm safety gate.

## 2026-06-03 Joint Damping Scale 5.0 Diagnostic

Milestone: `research_baseline_joint_damping_scale_5p0_v1`

Evidence: `diagnostics/research_baseline_joint_damping_scale_5p0_v1/summary.md`

The 5x SDF joint damping diagnostic was run because endpoint-hold evidence
showed multi-centimeter oscillation and prior 2x damping improved but did not
clear the strict gate. This diagnostic keeps the canonical targets, safety
gates, force thresholds, and success criteria unchanged.

Runtime result:

- `Outcome: ABORTED`;
- `Reason: INSERT blocked: peg_z 0.8534m is above force-safe precondition 0.8450m`;
- MOVING_TO_START reached strict no-contact stability and transitioned to APPROACH;
- insertion depth `0.0000 m`;
- contact-topic samples `0`;
- max raw force norm `209.198759 N`;
- approach final feedback `z=0.849622 m` for target `z=0.830000 m`;
- approach missing descent `0.019622 m`.

This is a meaningful stabilization improvement, not physical insertion success.
The next blocker has moved from start hold to approach depth realization before
INSERT.

## 2026-06-03 Insert Sim-Time Completion

Milestone: `research_baseline_insert_sim_time_completion_v4`

Evidence: `diagnostics/research_baseline_insert_sim_time_completion_v4/summary.md`

The INSERT phase now waits on ROS/Gazebo time from the moment the INSERT
trajectory is published. `research_baseline.launch.py` passes `use_sim_time` to
`admittance_insertion_node`, and the final success contract now requires
`max_insert_contact_force_N` so RETREAT contact cannot create a false insertion
success.

Validation passed Python syntax, targeted `colcon build --packages-select
kuka_task_control thesis_bringup`, a headless launch with
`joint_damping_scale:=5.0`, and the passive analyzers.

Runtime result:

- final outcome `SUCCESS`;
- reason `Full cycle completed. Insertion depth 0.019m, contact 60.1N during INSERT`;
- final insertion depth `0.0191 m`;
- max task-side insert contact `60.1 N`;
- max raw `|Fz|=133.33 N`;
- max raw force norm `211.14 N`;
- pre-insertion XY error `0.0006 m`;
- INSERT command observed for `23.111 s` after a `20.000 s` command, fixing the previous about `10.6 s` premature RETREAT transition.

This was a credible simulated insertion-depth/contact event under the older
criteria, not a robustness claim. It is now superseded by
`research_baseline_insert_physical_xy_gate_v1`, which requires final inserted
XY error within the physical 1 mm radial clearance. The passive Gazebo contact
observer still recorded contact-topic rows only in `RETREAT` for this run, and
RETREAT contact reached `249.593329 N`.

## 2026-06-03 Staged Withdrawal Diagnostic

Milestone: `research_baseline_staged_withdrawal_v1`

Evidence: `diagnostics/research_baseline_staged_withdrawal_v1/summary.md`

A staged vertical-lift-then-home RETREAT was tested after the v4 success
because v4 still produced RETREAT contact. The experiment preserved insertion
success (`0.0208 m` depth, `59.5 N` insert contact evidence), but worsened
withdrawal contact:

- v4 RETREAT contact rows: `41`;
- staged v1 RETREAT contact rows: `307`;
- v4 RETREAT max contact force: `249.593329 N`;
- staged v1 RETREAT max contact force: `486.746287 N`;
- v4 max raw force norm: `211.14 N`;
- staged v1 max raw force norm: `285.8 N`.

The staged withdrawal source change was removed. Future withdrawal work should
diagnose fixture/hole contact during vertical extraction rather than simply
splitting lift and home trajectories.

## 2026-06-03 Withdrawal Contact Timing

Milestone: `research_baseline_withdrawal_contact_timing_v1`

Evidence: `diagnostics/research_baseline_withdrawal_contact_timing_v1/summary.md`

The new `withdrawal_contact_timing_analyzer` correlates recorded contact rows
with active trajectory-command windows and controller-state peg-tip feedback.
It was run on both the v4 insertion success and the rejected staged-withdrawal
run.

Result:

- v4: all `41` positive contact samples occurred in `RETREAT_1`; first contact
  was `0.554 s` after the RETREAT command while the peg was still inserted
  `0.022622 m`;
- staged v1: all `307` positive contact samples occurred in the vertical
  `RETREAT_1` lift; none were attributed to the later `RETREAT_2` home command;
- staged v1 highest contact was `486.746287 N` at depth `0.019732 m` with
  `0.005979 m` XY error.

Interpretation: the current withdrawal blocker is side-loaded extraction while
the peg is still inside or near the hole, not late lateral home motion. The next
motion change should reduce initial extraction contact or fixture collision
geometry before repeated-validation claims.

## 2026-06-03 Insert Physical XY Gate

Milestone: `research_baseline_insert_physical_xy_gate_v1`

Evidence: `diagnostics/research_baseline_insert_physical_xy_gate_v1/summary.md`

The success contract now requires final INSERT XY error to fit within the
physical radial clearance of the task: 25 mm peg, 27 mm hole, so `0.0010 m`.
Runtime validation with `joint_damping_scale:=5.0` reached depth and contact
but correctly failed the physical-success gate:

- final outcome `DEGRADED`;
- insertion depth `0.0177 m`;
- max task-side INSERT contact `55.4 N`;
- final insertion XY error `0.0030 m`;
- final INSERT XY tolerance `0.0010 m`;
- max raw force norm `234.96 N`;
- RETREAT contact max `589.942680 N`.

This supersedes the earlier v4 success wording. The current baseline has an
insertion-depth/contact event, not validated physical insertion success. The
next implementation should reduce inserted-depth XY drift and side-loaded
extraction contact.

## 2026-06-03 Insert Sideload Abort

Milestone: `research_baseline_insert_sideload_abort_v1`

Evidence: `diagnostics/research_baseline_insert_sideload_abort_v1/summary.md`

The controller now aborts INSERT when the peg is at least `0.0010 m` below the
hole top and XY error exceeds the physical radial clearance `0.0010 m` for
three consecutive control ticks. Validation produced an honest safety abort:

- final outcome `ABORTED`;
- reason `INSERT aborted: side-loaded peg at depth 0.0011m with XY error 0.0032m`;
- max raw force norm `204.14 N`;
- contact-topic rows `2`;
- max contact-topic force `0.000000 N`.

Compared with `research_baseline_insert_physical_xy_gate_v1`, passive contact
rows dropped from `1293` to `2` and RETREAT max contact force dropped from
`589.942680 N` to `0.000000 N`. This is not task success; it is a safer
fail-closed behavior. The next step is reducing the inserted-depth XY drift
that triggers this abort.

## 2026-06-03 Insert XY Drift Diagnostic

Milestone: `research_baseline_insert_xy_drift_diagnostic_v1`

Evidence: `diagnostics/research_baseline_insert_xy_drift_diagnostic_v1/summary.md`

Added `insert_xy_drift_analyzer`, an offline analyzer over
`trajectory_commands.csv` plus controller-state tracking samples. It reconstructs
peg-tip Cartesian feedback through the project kinematics and reports
pre-command boundary XY, the first physical-clearance violation, the first
meaningful insertion depth, and the first side-loaded depth event.

Validation passed Python syntax and targeted `colcon build --packages-select
thesis_bringup`. The analyzer was run on both
`research_baseline_insert_sideload_abort_v1` and
`research_baseline_insert_physical_xy_gate_v1`.

Result: current side-load abort evidence shows pre-command final XY
`0.001569 m`, command-window initial XY `0.002539 m`, and first meaningful
depth `0.001316 m` at XY `0.002488 m`. The prior physical-XY-gate run shows a
similar early clearance risk and later side-load: first meaningful depth
`0.001103 m` at XY `0.000323 m`, then side-load at depth `0.001274 m` with XY
`0.002153 m`.

This confirms the next control change must preserve the side-load abort and
gate no-contact INSERT motion against the physical clearance before attempting
deeper descent or learning.

## 2026-06-03 Insert Pre-Contact Clearance Gate

Milestone: `research_baseline_insert_precontact_clearance_gate_v1`

Evidence: `diagnostics/research_baseline_insert_precontact_clearance_gate_v1/summary.md`

The task now requires direct INSERT entry and SEARCH convergence to satisfy the
physical radial clearance `0.0010 m`. During INSERT it aborts before meaningful
depth if no-contact XY error exceeds that clearance for three control ticks.
The existing inserted-depth side-load and hard-force aborts remain active.

Validation passed Python syntax and targeted `colcon build --packages-select
kuka_task_control thesis_bringup`. The first sandboxed launch failed before
spawn because DDS/Gazebo could not create local sockets; the same command was
rerun with escalated permissions and reached task `DONE`.

Runtime result:

- final outcome `ABORTED`;
- reason `INSERT aborted: no-contact XY error 0.0027m exceeds physical clearance 0.0010m before meaningful insertion depth 0.0010m for 3 ticks.`;
- insertion depth `0.0000 m`;
- SEARCH converged at pre-insertion XY `0.0006 m`;
- max physical depth from analyzer `0.000000 m`;
- positive contact-topic samples `0`;
- max raw force norm `205.09 N`.

This is a safer fail-closed behavior, not task success. The remaining blocker is
the one-point INSERT command drifting outside physical clearance almost
immediately after SEARCH has centered the peg.

## 2026-06-03 Insert Cartesian Descent Diagnostic

Milestone: `research_baseline_insert_cartesian_descent_v1`

Evidence: `diagnostics/research_baseline_insert_cartesian_descent_v1/summary.md`

Status: rejected; source reverted.

A centered, axis-aligned, multi-waypoint Cartesian INSERT descent was tested
with a `20 s` duration and the pre-contact clearance gate preserved. The
command published `6` points, but the run still aborted before meaningful
depth:

- final outcome `ABORTED`;
- reason `INSERT aborted: no-contact XY error 0.0026m exceeds physical clearance 0.0010m before meaningful insertion depth 0.0010m for 3 ticks.`;
- first clearance violation `0.005 s` after INSERT command receipt;
- max physical depth `0.000000 m`;
- positive contact-topic samples `0`.

The source was reverted to the validated pre-contact clearance gate state. The
next implementation should address INSERT command handoff/hold dynamics rather
than simply adding more waypoints.

## 2026-06-03 Insert Handoff Reference Diagnostic

Milestone: `research_baseline_insert_handoff_reference_v1`

Evidence: `diagnostics/research_baseline_insert_handoff_reference_v1/summary.md`

The workspace now includes `insert_handoff_reference_analyzer`, an offline
diagnostic that compares JTC Cartesian reference and feedback at the selected
INSERT command. It uses `trajectory_controller_state_samples.csv` and writes
Markdown/JSON evidence without publishing commands or changing safety gates.

Validation passed Python syntax, installed `ros2 run` execution, and targeted
`colcon build --packages-select thesis_bringup`.

Key result from the rejected multi-waypoint Cartesian descent run:

- INSERT command point count `6`;
- pre-command final reference XY error `0.000000 m`;
- pre-command final feedback XY error `0.001660 m`;
- initial reference and feedback XY error `0.000458 m`;
- reference stayed inside the `0.0010 m` physical clearance for the first `0.5 s`;
- feedback violated clearance `0.005 s` after INSERT command receipt;
- max reference XY error `0.000510 m`;
- max feedback XY error `0.004264 m`;
- max Cartesian reference-feedback error `0.004571 m`.

Interpretation: the rejected waypoint-only INSERT command was centered at the
JTC reference level, but the simulated plant/controller feedback drifted out
of clearance immediately. The next implementation should target INSERT
handoff/feedback stabilization or bounded pre-insert settling, not another
waypoint-count change or a looser clearance gate.

## 2026-06-03 Insert Handoff Settle

Milestone: `research_baseline_insert_handoff_settle_v1`

Evidence: `diagnostics/research_baseline_insert_handoff_settle_v1/summary.md`

The INSERT state now runs a bounded non-descending handoff hold before
publishing the final descent command. It requires `8` stable ticks inside the
`0.0010 m` physical clearance after a `2.0 s` hold and preserves the
pre-contact clearance abort.

Validation passed Python syntax, targeted `colcon build --packages-select
kuka_task_control thesis_bringup`, and a headless runtime run with
`joint_damping_scale:=5.0`.

Runtime result:

- final outcome `ABORTED`;
- reason `INSERT aborted: no-contact XY error 0.0024m exceeds physical clearance 0.0010m before meaningful insertion depth 0.0010m for 3 ticks.`;
- insertion depth `0.0000 m`;
- no final descent-to-`z=0.790 m` INSERT command was published;
- contact-topic samples `0`;
- max raw `|Fz|=127.6 N`;
- max raw force norm `213.6 N`.

Interpretation: this is a safety improvement, not insertion success. The
handoff hold prevents descent when feedback is already outside clearance. The
next blocker is SEARCH convergence quality: SEARCH accepted a transient
inside-clearance sample, but feedback was `0.002773 m` off center by the
handoff command boundary.

## 2026-06-03 Sustained SEARCH Clearance

Milestone: `research_baseline_search_sustained_clearance_v1`

Evidence: `diagnostics/research_baseline_search_sustained_clearance_v1/summary.md`

SEARCH now requires `8` consecutive control ticks inside the `0.0010 m`
physical clearance before it may enter INSERT. This prevents a transient
inside-clearance sample from triggering the INSERT handoff.

Validation passed syntax, targeted build, and a headless runtime run with
`joint_damping_scale:=5.0`.

Runtime result:

- final outcome `ABORTED`;
- reason `SEARCH timeout (45s). XY error 0.0028m remains above tolerance.`;
- no INSERT phase was entered;
- insertion depth `0.0000 m`;
- contact-topic samples `0`;
- max raw `|Fz|=129.1 N`;
- max raw force norm `211.4 N`;
- SEARCH samples inside `0.0010 m`: `308/4500`;
- longest consecutive inside-clearance run: `2` observer samples.

Interpretation: the new gate is stricter and safer. The next blocker is not
INSERT command generation; the controller must hold no-contact XY alignment
inside the physical clearance long enough for a credible handoff.

## 2026-06-03 Rejected SEARCH Centered Hold

Milestone: `research_baseline_search_centered_hold_v1`

Evidence: `diagnostics/research_baseline_search_centered_hold_v1/summary.md`

A no-contact centered hold at current SEARCH Z was tested before spiral search
offsets. It preserved the sustained `8` tick clearance gate and all INSERT
safety gates.

Runtime result:

- final outcome `ABORTED`;
- reason `SEARCH timeout (45s). XY error 0.0048m remains above tolerance.`;
- no INSERT phase was entered;
- insertion depth `0.0000 m`;
- contact-topic samples `0`;
- max raw `|Fz|=130.8 N`;
- max raw force norm `208.5 N`;
- SEARCH samples inside `0.0010 m`: `365/4500`;
- longest consecutive inside-clearance run: `4` observer samples.

Decision: rejected; source reverted. The centered hold was safe but did not
meet the sustained clearance gate and ended with worse final SEARCH XY than the
previous sustained-clearance run.

## 2026-06-03 XY Stability Analyzer

Milestone: `research_baseline_xy_stability_analyzer_v1`

Evidence:

- `diagnostics/research_baseline_search_sustained_clearance_v1/xy_stability_analysis.md`
- `diagnostics/research_baseline_search_centered_hold_v1/xy_stability_analysis.md`

The workspace now includes `xy_stability_analyzer`, an offline passive-log
diagnostic for `wrench_state_samples.csv`. It groups samples by task state and
estimates longest clearance windows at the task controller cadence (`10 Hz`) for
both the physical `0.0010 m` peg/hole radial clearance and the older `0.0020 m`
pre-contact alignment band.

Validation commands:

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/xy_stability_analyzer.py
colcon build --symlink-install --packages-select thesis_bringup
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_sustained_clearance_v1
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_centered_hold_v1
```

Result:

| Run | SEARCH min XY m | SEARCH mean XY m | SEARCH final XY m | Best SEARCH 1 mm ticks | Best SEARCH 2 mm ticks |
| --- | ---: | ---: | ---: | ---: | ---: |
| sustained-clearance | 0.000037 | 0.003108 | 0.004354 | 2 | 6 |
| centered-hold | 0.000062 | 0.003157 | 0.004772 | 2 | 6 |

Interpretation: both recent runs hit sub-millimeter samples, but neither holds
physical clearance beyond two estimated 10 Hz control ticks in SEARCH. This
supports preserving the sustained 1 mm gate and targeting feedback/control
stability before attempting INSERT again.

## 2026-06-03 SEARCH Recenter On Coarse Band

Milestone: `research_baseline_search_recenter_on_coarse_band_v1`

Evidence:

- `diagnostics/research_baseline_search_recenter_on_coarse_band_v1/summary.md`
- `diagnostics/research_baseline_search_recenter_on_coarse_band_v1/xy_stability_analysis.md`

SEARCH now recenters at the current peg Z whenever feedback is inside the older
`0.0020 m` pre-contact band but has not sustained the physical `0.0010 m`
clearance gate. Spiral offsets, timeout, hard-force abort, and INSERT gates
remain bounded and unchanged.

Validation passed syntax, targeted build, and a headless run with
`joint_damping_scale:=5.0`.

Runtime result:

- final outcome `ABORTED`;
- reason `SEARCH timeout (45s). XY error 0.0037m remains above tolerance.`;
- no INSERT phase was entered;
- insertion depth `0.0000 m`;
- contact-topic samples `0`;
- max raw `|Fz|=128.6 N`;
- max raw force norm `209.9 N`;
- observed SEARCH recenter attempts: `2`;
- best estimated SEARCH `0.0010 m` clearance window: `4` controller ticks;
- best estimated SEARCH `0.0020 m` window: `9` controller ticks.

Decision: keep as an improvement but do not claim success. The change doubled
the best 1 mm SEARCH stability window compared with the previous two ticks, but
still did not satisfy the required eight ticks for a credible INSERT handoff.

## 2026-06-03 SEARCH Recenter 4 mm

Milestone: `research_baseline_search_recenter_4mm_v1`

Evidence:

- `diagnostics/research_baseline_search_recenter_4mm_v1/summary.md`
- `diagnostics/research_baseline_search_recenter_4mm_v1_repeat2/summary.md`
- `diagnostics/research_baseline_search_recenter_4mm_v1_repeat2/xy_stability_analysis.md`

The SEARCH recenter trigger was widened to a bounded `0.0040 m` near-center
band. This only affects command selection inside SEARCH; it does not change the
physical `0.0010 m` clearance required for INSERT.

Two validations were run with `joint_damping_scale:=5.0`:

- first run bypassed SEARCH and aborted safely in INSERT handoff after feedback
  drifted from a direct-entry `0.0009 m` XY condition to `0.0035 m`;
- second run exercised SEARCH, made six recenter attempts, and timed out safely
  at `0.0017 m` final SEARCH XY.

Second-run observer result:

- contact-topic samples `0`;
- SEARCH mean XY `0.002402 m`;
- SEARCH final XY `0.001682 m`;
- best estimated SEARCH `0.0010 m` clearance window `4` controller ticks;
- best estimated SEARCH `0.0020 m` window `8` controller ticks.

Decision: keep as an incremental improvement. It improves mean/final SEARCH XY
relative to the 2 mm recenter run, but still does not meet the eight-tick
physical-clearance gate, so INSERT remains correctly blocked.

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

## 2026-06-02 Slow Start Settle After Tool-Tip Fix

Milestone: `research_baseline_start_slow_settle_after_tool_fix_v1`

After correcting the gripper peg-tip frame, a bounded one-shot settle
experiment tested whether another 20 s same-target `MOVING_TO_START` trajectory
from current feedback to the same axis-aligned start joint target would hold
the peg inside the strict 2 mm no-contact gate. The experiment was rejected and
the source change was removed.

Validation command:

```bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  tracking_log_dir:=diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1
```

Evidence:

- `diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1/summary.md`
- `diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1/trial_outcome.json`
- `diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1/clearance_path_analysis.md`

Result: `ABORTED` in `MOVING_TO_START`, zero insertion depth, zero contact-topic
samples, max raw `|Fz|=171.25 N`, and max raw force norm `271.34 N`. The final
phase timeout reported `cart_err=0.012 m`, `xy_err=0.011 m`, `joint_err=0.025
rad`, and `stable=0/5`. Offline replay showed the corrected peg tip crossed
the 2 mm XY gate only transiently, with a minimum replayed XY error
`0.000072 m` but only two consecutive strict observer samples. Clearance
analysis still found `0/201` planned and `0/1884` runtime-feedback `link_5`
target-plate intersections.

Current conclusion: same-target settle publication is not a credible fix. The
next milestone remains above-hole pose hold/tracking stabilization from
measured controller and physics behavior, with the strict no-contact gate and
hard-force abort preserved.

## 2026-06-02 Post-Tool Start Gain Diagnostics

Milestones:

- `research_baseline_start_gain_2000_after_tool_fix_v1`
- `research_baseline_start_gain_3000_after_tool_fix_v1`

After the tool-tip frame correction, global `position_gain` values 2000 and
3000 were retested with unchanged task gates and full observers. Both runs were
safe bounded failures: no descent, zero insertion depth, zero contact-topic
samples, and no `link_5` target-plate clearance intersections.

Evidence:

- `diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1/summary.md`
- `diagnostics/research_baseline_start_gain_3000_after_tool_fix_v1/summary.md`

Result: gain 2000 was the better of the two but still failed. It timed out in
`MOVING_TO_START` with final `cart_err=0.006 m`, `xy_err=0.002 m`,
`joint_err=0.009 rad`, and `stable=0/5`; replay showed a best strict-gate
streak of only three observer samples. Gain 3000 timed out with final
`xy_err=0.007 m` and only two consecutive strict observer samples. The
canonical gain therefore remains unchanged.

Current conclusion: global gain increase alone is not a credible fix. The next
milestone should implement and validate explicit endpoint hold/tracking
behavior or trajectory timing changes, still preserving the strict 2 mm
no-contact gate.

## 2026-06-02 Zero-Derivative Trajectory Hold Diagnostic

Milestone: `research_baseline_zero_derivative_trajectory_hold_v1`

A small trajectory-publisher experiment populated every
`JointTrajectoryPoint` with zero velocities and accelerations to test whether
the spline controller needed explicit stop conditions at the endpoint. The
change was rejected and removed.

Validation command:

```bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  tracking_log_dir:=diagnostics/research_baseline_zero_derivative_trajectory_hold_v1
```

Evidence: `diagnostics/research_baseline_zero_derivative_trajectory_hold_v1/summary.md`

Result: `ABORTED` in `MOVING_TO_START`, zero insertion depth, zero contact-topic
samples, max raw `|Fz|=176.57 N`, and max raw force norm `272.23 N`. Clearance
analysis found `0/201` planned and `0/2647` runtime-feedback `link_5`
target-plate intersections. Corrected peg-tip replay showed a very low
minimum XY error of `0.000014 m`, but the best strict-gate streak was only four
observer samples and the final timeout regressed to `xy_err=0.014 m`.

Current conclusion: explicit zero derivatives alone are not a credible fix.
The next milestone should target endpoint hold observability/control more
directly, not relax the 2 mm gate.

## 2026-06-02 Above-Hole Hold Analyzer

Milestone: `research_baseline_above_hole_hold_analyzer`

The workspace now includes `above_hole_hold_analyzer`, an offline diagnostic
tool for `wrench_state_samples.csv`. It estimates whether passive observer
data would satisfy the task controller's preserved strict 2 mm no-contact XY
gate for five consecutive 10 Hz state-machine ticks. It does not publish
commands or alter controller safety behavior.

Validation commands:

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/above_hole_hold_analyzer.py
colcon build --symlink-install --packages-select thesis_bringup
ros2 run thesis_bringup above_hole_hold_analyzer diagnostics/research_baseline_tool_tip_frame_correction_v1
ros2 run thesis_bringup above_hole_hold_analyzer diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1
ros2 run thesis_bringup above_hole_hold_analyzer diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1
ros2 run thesis_bringup above_hole_hold_analyzer diagnostics/research_baseline_start_gain_3000_after_tool_fix_v1
ros2 run thesis_bringup above_hole_hold_analyzer diagnostics/research_baseline_zero_derivative_trajectory_hold_v1
```

Evidence:

- `diagnostics/research_baseline_tool_tip_frame_correction_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_start_gain_3000_after_tool_fix_v1/above_hole_hold_analysis.md`
- `diagnostics/research_baseline_zero_derivative_trajectory_hold_v1/above_hole_hold_analysis.md`

Result: all five post-tool runs failed the estimated five-tick gate. The
corrected tool-tip run reached minimum XY `0.000037 m`, gain 2000 reached
`0.000455 m`, gain 3000 reached `0.000034 m`, and the zero-derivative run
reached `0.000257 m`, but none held the strict gate for more than one
estimated 10 Hz state-loop tick. This confirms the blocker is controlled
endpoint hold, not merely reaching the target once.

## 2026-06-02 MOVING_TO_START Tracking Analyzer

Milestone: `research_baseline_moving_to_start_tracking_analyzer`

The workspace now includes `moving_to_start_tracking_analyzer`, an offline
diagnostic that selects the MOVING_TO_START command by FK target near the
canonical axis-align peg-tip pose instead of assuming command index 0. This is
needed because some diagnostics only captured the abort-retreat command in
`trajectory_commands.csv`; those runs are now reported as missing axis-align
command evidence instead of being misattributed.

Validation commands:

```bash
python3 -m py_compile src/thesis_bringup/thesis_bringup/moving_to_start_tracking_analyzer.py
colcon build --symlink-install --packages-select thesis_bringup
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_tool_tip_frame_correction_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_start_gain_3000_after_tool_fix_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_zero_derivative_trajectory_hold_v1
```

Evidence: `moving_to_start_tracking_analysis.md/json` in the same five
diagnostic directories.

Result: the tool-tip and gain-3000 diagnostics did not capture the axis-align
command, only retreat, so they cannot be used for MOVING_TO_START joint
attribution. The slow-settle, gain-2000, and zero-derivative runs did capture a
valid axis-align command. They show distributed joint tracking error rather
than a single dominant joint: worst p95 joints were `joint_4` at `0.020421 rad`
for slow settle, `joint_3` at `0.022163 rad` for gain 2000, and `joint_5` at
`0.024759 rad` for zero-derivative hold. Final XY drift remained `0.012767 m`,
`0.003992 m`, and `0.011025 m` respectively, so the next implementation should
target closed endpoint hold/correction rather than another broad gain or
single-joint dynamics change.

## 2026-06-02 Start Endpoint Correction Diagnostic

Milestone: `research_baseline_start_endpoint_correction_v1`

A bounded endpoint-correction experiment was tested in `MOVING_TO_START`.
Corrections were allowed only after the original 40 s start trajectory had
finished, only above the workpiece, only at low force, only when XY error was
within 20 mm, and only if the IK correction required at most 0.05 rad of joint
motion. The experiment did not relax the 2 mm no-contact gate and did not allow
descent without five strict stable ticks.

Evidence: `diagnostics/research_baseline_start_endpoint_correction_v1/summary.md`

Result: rejected and source change removed. The run remained safe and recorded
zero contact-topic samples. It rejected one large correction and accepted three
small corrections, but still aborted in `MOVING_TO_START` with final
`cart_err=0.010 m`, `xy_err=0.004 m`, `joint_err=0.018 rad`, and `stable=0/5`.
Offline hold analysis still found only one estimated 10 Hz stable tick. This is
not a credible canonical fix.

## 2026-06-03 Joint Damping Scale 2.0 Diagnostic

Milestone: `research_baseline_joint_damping_scale_2p0_v1`

A diagnostic run doubled converted SDF joint damping while preserving the
canonical position gain and all task safety gates.

Validation command:

```bash
timeout 200s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  joint_damping_scale:=2.0 \
  tracking_log_dir:=diagnostics/research_baseline_joint_damping_scale_2p0_v1
```

Evidence: `diagnostics/research_baseline_joint_damping_scale_2p0_v1/summary.md`

Result: rejected as a canonical change. The run stayed safe, recorded zero
contact-topic samples, and reduced p95 max joint error to about `0.0158 rad`.
It also improved the estimated strict-gate hold to `2/5` ticks, but still
aborted in `MOVING_TO_START` with final `xy_err=0.007 m`, `joint_err=0.013
rad`, and `stable=0/5`. Canonical damping remains unchanged.

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

Historical note: this older proposal-era next milestone is superseded by the
current status section near the top of this README and by
`docs/CURRENT_PROJECT_STATUS.md`.

Superseded milestone: `proposal_simulation_cell_v2_17_contact_gated_moving_to_start_transition`

- Why the admittance node stays in MOVING_TO_START after trajectory completion
- Debug the contact detection threshold (5.0 N) vs. observed contact wrench (~2.7 N, below threshold)
- Verify the FT sensor bridge remapping from `/world/peg_in_hole_world/model/lbr_iisy6_r1300/joint/ft_sensor_joint/sensor/ft_sensor/forcetorque` → `/ft_sensor_wrench`
- Confirm the automaton state machine logic checks for contact after trajectory complete

## admittance_controller_v2_honest_tracking_and_contact_estimation

Status: `historical_insertion_depth_event_observed_physical_xy_gate_now_required`

The v2 controller has produced simulated insertion-depth/contact events with
measured insertion depth, but these runs are not enough to claim robust
autonomous peg-in-hole success. The current physical-clearance gate requires
final inserted XY error within the 25 mm peg / 27 mm hole radial clearance.
The correct wording is: **controller-driven insertion-depth/contact event, not
validated physical success**.

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

Physical success is counted only when `trial_outcome == SUCCESS`, insertion depth is at least 0.010 m, INSERT contact force exceeds the configured threshold, final inserted XY error is within the physical peg/hole radial clearance, and no safety abort occurs.

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
