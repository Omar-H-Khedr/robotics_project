# Project Context

Last reviewed: 2026-06-02

This workspace is the active ROS 2 Jazzy / Gazebo implementation for the PhD topic:

**Visuomotor Context-Based Meta-Reinforcement Learning with Virtual-Force Safety for Adaptable Peg-in-Hole Assembly in Smart Manufacturing.**

## Target Platform

- Robot: KUKA LBR iisy 6 R1300
- Axes: 6
- Rated payload: 6 kg
- Maximum payload: 6.9 kg
- Reach: 1300 mm
- Repeatability: +/-0.05 mm
- Footprint: 275 mm x 275 mm
- Approximate mass: 46.3 kg
- Controller family: KR C5 micro / KR C5 micro-2
- Current simulation stack: ROS 2 Jazzy, Gazebo Sim, `gz_ros2_control`

## Current Implementation Status

The current workspace contains a Gazebo workcell with:

- project-local KUKA LBR iisy 6 R1300 model adaptation;
- pedestal-mounted robot spawn at SAFE_HOME;
- fixed gripper and fixed grasped cylindrical peg;
- fixed work table, target plate, and hole fixture;
- `joint_state_broadcaster` and `joint_trajectory_controller`;
- a canonical research baseline bridge that intentionally does not bridge Gazebo `/joint_states`;
- FT sensor injection and ROS bridge to `/ft_sensor_wrench`;
- RGB-D D405 camera model in the world, with perception config aligned to `/d405/*` topics;
- task-level admittance insertion node with phase logging;
- dry-run experiment/context scaffolds from earlier proposal milestones.

The strongest historical single-run insertion evidence so far is one simulated insertion-depth event:

- insertion depth: about 0.011 m;
- sustained contact: about 142.9 N;
- final phase sequence reached RETREAT/DONE in that run.

Repeated validation on 2026-06-01 produced 0/3 physical successes:

- one DEGRADED INSERT with only 0.0037 m depth and a 1237.45 N peak raw Fz spike;
- one ABORTED INSERT with a 3716.2 N peak raw Fz spike;
- one SEARCH timeout/no-outcome before final insertion evaluation.

This is not robust autonomous peg-in-hole success. The honest claim remains:

**first simulated insertion event with measured insertion depth.**

## Known Open Risks

- Repeat validation is complete for the current controller revision and failed: 0/3 physical successes.
- MOVING_TO_START is non-deterministic, with roughly 1-in-3 failures reported.
- MOVING_TO_START and APPROACH can have large Cartesian tracking errors.
- Peak raw Fz spikes around 1237 N and 3716 N have now been observed in repeated validation.
- Multi-point INSERT trajectory behavior is broken; current INSERT uses a single-point trajectory.
- Contact/gravity estimation depends on median Fz baseline validity and needs more validation.
- Gazebo contact physics are adequate for early simulation evidence but not final safety fidelity.
- Some older docs still describe stale iisy3 state and must not be used as current truth.
- A 2026-06-02 source-integrity run confirmed `joint_state_broadcaster` as the intended `/joint_states` publisher, but the same run still timed out in `MOVING_TO_START` with large XY error.
- A 2026-06-02 tracking audit confirmed the above-hole target is reachable in offline IK, but runtime Gazebo/controller tracking remains underdamped or unstable. Gain 250 and repeated bounded refinements were rejected.
- A 2026-06-02 cell-model consistency audit validated iisy6 naming, target Z convention, D405 topics, fixed peg geometry, deprecated cylinder marking, and standalone world/robot SDF checks. The same headless launch still failed to satisfy the preserved 0.002 m no-contact gate, with best observed XY error about 0.027 m.
- A 2026-06-02 primitive-collision audit replaced canonical Gazebo KUKA arm mesh collisions with simple DART-loadable primitive collisions while keeping mesh visuals. The prior KUKA arm mesh-collision rejection messages were not observed, and one run reached the strict above-hole XY gate before timing out in `APPROACH`.
- A 2026-06-02 strict-stability audit removed descent from transient above-hole crossings. The latest validation aborted safely in `MOVING_TO_START` with `xy_err=0.018 m`, `stable=0/5`, zero insertion depth, and a high no-contact F/T spike.
- A 2026-06-02 controller-config audit made the canonical baseline use a project-local 250 Hz `research_baseline_ros2_control.yaml`. The config loaded correctly and reduced controller-configuration ambiguity, but the task still aborted safely in `MOVING_TO_START` with `xy_err=0.011 m`, `stable=0/5`, and zero insertion depth.
- A 2026-06-02 trajectory-tracking audit added a passive observer for task-command-versus-`/joint_states` tracking. The latest validation still aborted safely in `MOVING_TO_START` with `xy_err=0.010 m`, while the observer recorded p95 max joint error `0.024368 rad` and final max joint error `0.014106 rad`.
- A 2026-06-02 slower move-to-start timing experiment was rejected and reverted. It delayed arrival near the above-hole pose and still aborted safely with `xy_err=0.011 m`, `stable=0/5`, and zero insertion depth.
- A 2026-06-02 bounded same-target hold-correction experiment was rejected and reverted. It produced transient XY errors as low as `0.0008 m`, but did not satisfy the consecutive strict stability gate and timed out safely with final logged `xy_err=0.011 m`, `stable=0/5`, and zero insertion depth.
- A 2026-06-02 raw-wrench instrumentation milestone added passive wrench-by-state logging and callback-level hard-force latching. The latest validation aborted in `MOVING_TO_START` with `max_abs_fz_N=1943.29`, `max_force_norm_N=2936.48`, and zero insertion depth.
- A 2026-06-02 contact-wrench correlation milestone added passive contact-topic logging. The latest validation aborted in `MOVING_TO_START` with `max_abs_fz_N=1396.75` and `max_force_norm_N=2624.11`; the canonical peg/hole/target contact topics produced zero messages in that run.
- A 2026-06-02 F/T mount effort-limit validation corrected the preserved zero-range measurement joint from `effort=1`, `velocity=0` to `effort=10000`, `velocity=100`. The latest validation still aborted safely in `MOVING_TO_START`, but peak raw wrench dropped to `max_abs_fz_N=612.25` and `max_force_norm_N=1765.41`; canonical contact topics still produced zero messages.
- A 2026-06-02 full-path contact bridge validation fixed the contact observability gap and added a robot-mounted peg contact sensor. The latest validation still aborted safely in `MOVING_TO_START`, and contact evidence now shows peg-target contact before descent (`MOVING_TO_START` max contact force `2427.31 N`).
- A 2026-06-02 axis-aligned start-pose validation replaced position-only Cartesian IK with joint-limit-aware peg-axis-constrained IK. The latest validation no longer hard-aborted on raw wrench and recorded no peg-source contact rows, but it still timed out in `MOVING_TO_START` with final `xy_err=0.006 m`, zero insertion depth, and no descent.
- A 2026-06-02 search fail-closed validation gave the safer axis-aligned start posture a scoped 120 s timeout. The run reached the strict 2 mm no-contact XY gate in `MOVING_TO_START` after 96.3 s, then failed honestly in `APPROACH` after 90 s because the 67 mm descent was not tracked. The state machine transitioned directly to `ABORT`; no `SEARCH` rows were recorded.
- A 2026-06-02 slow-approach timing experiment was rejected and reverted. Increasing the approach command duration to 41.7 s still failed with `cart_err=0.070 m`, `joint_err=0.108 rad`, and zero insertion depth.
- A 2026-06-02 high-gain approach diagnostic was rejected. `position_gain:=3000` slightly reduced time to the above-hole gate but worsened approach failure to `cart_err=0.073 m`, `joint_err=0.110 rad`, with peak raw force norm `890.27 N`.
- A 2026-06-02 joint-level approach tracking diagnostic added a reusable analyzer and confirmed that `joint_2` dominates the missing descent across normal, slow-descent, and high-gain runs. The command target remains the correct `z=0.830 m` touch pose, while final feedback remains near `z=0.897-0.900 m`.
- A 2026-06-02 broad damping-reduction diagnostic was rejected. `joint_damping_scale:=0.2` preserved safety gates but hard-aborted in `MOVING_TO_START` at raw `|Fz|=1181.0 N` before reaching the no-contact gate or approach phase.
- A 2026-06-02 effort-authority diagnostic was rejected. `joint_effort_scale:=2.0` reached the no-contact gate faster and entered `APPROACH`, but hard-aborted after 0.5 s with force norm `1009.7 N` and target-source contact rows while the peg was still at `z=0.890982 m` against the `z=0.830000 m` target.
- A 2026-06-02 contact-pair attribution diagnostic added exact collision-pair logging to the passive contact observer. A reproduced doubled-effort run aborted in `MOVING_TO_START` with raw `|Fz|=1018.9 N` and showed target-source contact from `lbr_iisy6_r1300::link_5::link_5_collision <-> target_plate::plate_link::target_plate_collision`. This is invalid robot-link clearance contact, not peg insertion contact.
- A 2026-06-02 tool-tip frame correction moved the modeled `peg_tip` from the near-palm end of the 110 mm peg to the protruding negative local tool-Z end and updated `RobotKinematics` to match. Offline clearance analysis and runtime feedback then showed zero `link_5` target-plate intersections, zero contact-topic samples, and max raw force norm `270.82 N`. The validation still failed honestly in `MOVING_TO_START` with final XY about `0.014 m`.
- A 2026-06-02 slow same-target settle after the tool-tip correction was rejected and removed. It aborted safely in `MOVING_TO_START` with final `xy_err=0.011 m`, `stable=0/5`, zero contact-topic samples, zero insertion depth, and no planned or runtime-feedback `link_5` target-plate intersections. Offline replay showed the corrected peg tip crossed the strict 2 mm XY gate only transiently, with minimum replayed XY `0.000072 m` but only two consecutive strict observer samples.
- 2026-06-02 post-tool global gain diagnostics at `position_gain:=2000` and `position_gain:=3000` were rejected. Both preserved zero contact-topic samples and zero `link_5` target-plate intersections, but neither held the strict gate. Gain 2000 was closest with final `xy_err=0.002 m` and a best strict replay streak of three observer samples; gain 3000 ended at `xy_err=0.007 m` with a best streak of two samples.
- A 2026-06-02 zero-derivative trajectory-point experiment was rejected and removed. It explicitly filled trajectory velocities and accelerations with zeros, remained safe and clearance-clean, but timed out at final `xy_err=0.014 m` after reaching only four consecutive strict observer samples.
- A 2026-06-02 above-hole hold analyzer milestone added a reusable offline diagnostic for `wrench_state_samples.csv`. Re-analysis of five post-tool runs showed none satisfied the estimated five 10 Hz stable ticks required by the preserved 2 mm no-contact gate. Several runs reached sub-millimetre XY error transiently, but the best estimated state-loop hold was only one tick.
- A 2026-06-02 MOVING_TO_START tracking analyzer milestone added selector-based command attribution for the axis-align command. It prevents retreat-only command logs from being misread as start tracking and showed the usable post-tool start runs have distributed joint error with persistent Cartesian XY drift, not one dominant joint comparable to the approach `joint_2` failure.
- A 2026-06-02 bounded endpoint-correction experiment was rejected and removed. It accepted three small no-contact corrections and improved final timeout XY to about `0.004 m`, but still failed the five-tick strict hold gate and aborted in `MOVING_TO_START` with zero contact-topic samples.
- A 2026-06-03 2x joint-damping diagnostic was rejected as a canonical change. It reduced p95 max joint tracking error to about `0.0158 rad` and improved the estimated strict hold to two ticks, but still failed `MOVING_TO_START` with final XY about `0.007 m` and `stable=0/5`.
- A 2026-06-03 2x damping plus `position_gain:=1500` diagnostic was rejected as a canonical change. It reached instantaneous XY error as low as `0.000052 m`, but the estimated strict hold was still only two 10 Hz ticks, final `MOVING_TO_START` XY was about `0.004 m`, contact-topic samples were zero, and the run aborted safely before descent.
- A 2026-06-03 trajectory command-capture fix added a bounded first-command discovery wait before the task publishes its first joint trajectory. A short validation run captured the 20-point `MOVING_TO_START` command and the selector-based tracking analyzer attributed it to the canonical axis-align target. This is an instrumentation/reproducibility fix, not insertion evidence.
- A 2026-06-03 canonical post-command-capture validation failed safely in `MOVING_TO_START`. It captured both the start and abort-retreat commands, attributed command index 0 to the canonical axis-align target, and showed final command-attributed XY error about `0.0036 m`. The strict 2 mm hold gate still failed with only one estimated stable tick, zero contact-topic samples, and zero insertion depth.
- A 2026-06-03 MOVING_TO_START XY-distribution analyzer enhancement showed why final/minimum samples are insufficient: in the canonical post-command-capture run, command-window minimum XY was `0.000115 m`, but only `138/15524` samples were inside the strict 2 mm band and the final one-second XY range was `0.000575-0.018345 m`.

## Current Success Criteria

A state machine reaching DONE is insufficient. A physical success trial requires:

- final trial outcome `SUCCESS`;
- measured insertion depth at least 0.010 m;
- contact force above the configured insertion/contact threshold;
- no safety abort;
- no unresolved timeout that invalidates task execution;
- recorded peak raw Fz and Cartesian-error metrics.

Robust success requires repeated validation with a documented success rate and failure modes.

## Next Technical Milestone

`research_baseline_above_hole_hold_tracking_stabilization`

Reason: after the peg-tip frame correction, the current canonical blocker is
again stable above-hole holding. The corrected tool frame removed the
reproduced `link_5` target-plate collision, but the controller does not hold
the peg inside the strict 2 mm no-contact XY gate for the required consecutive
state-machine ticks. A post-correction slow settle crossed the gate only
transiently and was rejected.

Post-correction global gain increases were also rejected. Gain 2000 improved
the final error but still did not meet the consecutive strict-gate requirement;
gain 3000 was worse. The next implementation should focus on explicit
endpoint hold/tracking behavior or trajectory timing rather than another
global gain increase.

Explicit zero velocity/acceleration trajectory points were also rejected. They
did not create a stable hold and regressed the final timeout error, so the
remaining work should instrument and control the endpoint hold window more
directly.

The new `above_hole_hold_analyzer` should be used for that next work. It
converts passive `MOVING_TO_START` observer rows into estimated 10 Hz
state-loop hold windows and confirmed that recent post-tool diagnostics cross
the strict 2 mm gate only briefly. The next implementation should therefore
target sustained endpoint hold/control, not gate relaxation.

Use `moving_to_start_tracking_analyzer` alongside it when checking trajectory
logs. It selects the axis-align command by FK target pose, so diagnostics that
only captured abort-retreat are not falsely treated as MOVING_TO_START
evidence. Current usable post-tool runs show final XY drift despite small,
distributed joint errors, which makes endpoint correction/hold behavior the
next target.

The first-command capture race has been reduced by a bounded discovery wait in
the task node. Future canonical runs should normally capture the initial
axis-align trajectory; if they do not, treat that as an instrumentation failure
before drawing controller-tracking conclusions.

The first full canonical run after that fix confirms the same physical blocker
with better evidence: the axis-align command is captured and nearly reached,
but the endpoint is not held inside the strict 2 mm XY gate for the required
five consecutive 10 Hz ticks. Continue focusing on sustained endpoint hold,
controller/physics dynamics, or target-frame feedback consistency before
approach or learning work.

Use the enhanced MOVING_TO_START tracking analyzer to judge command-attributed
XY stability by distribution and final-window range. A single minimum or final
sample is not sufficient evidence for safe no-contact alignment.

A first bounded endpoint-correction implementation was tested and rejected. It
was safe, but it did not hold the strict gate, so the source was removed. The
next implementation should not merely republish small IK corrections; it should
address why the endpoint continues to oscillate or drift across the strict
2 mm window.

The 2x damping diagnostic suggests damping is relevant but insufficient alone.
It improved joint tracking and the strict-gate hold window without contact
regression, but it still failed before descent. Treat it as evidence for a
targeted controller/physics stabilization path, not as a canonical robot model
change.

Combining 2x damping with `position_gain:=1500` also failed the strict hold gate.
It produced a near-zero instantaneous XY sample but still only achieved an
estimated two stable state-loop ticks and did not capture a reliable
axis-align command for command-attributed tracking analysis. Do not adopt it as
the canonical setting.

A same-target refresh experiment was tested and rejected: repeated MOVING_TO_START target publication produced hard-force aborts and did not improve XY gate convergence.

The next step remains tracking stabilization. The joint-state source integrity milestone removed one measurement ambiguity; it did not solve the large no-contact XY error.

Tracking stabilization should now focus on final above-hole XY settling with
the corrected tool frame, then approach IK trajectory realization, `joint_2`
tracking authority, final-pose damping, and high free-space F/T behavior using
the command-vs-feedback, wrench-by-state, and contact-by-pair evidence. A
globally slower move-to-start trajectory, repeated same-target hold
corrections, slower approach timing, higher plugin position gain, broad damping
reduction, doubled effort limits, one slow post-tool-fix same-target settle,
post-tool global gains 2000/3000, and zero-derivative trajectory points were
tested and rejected. The weak F/T
measurement-joint limit has been
corrected but did not eliminate all force spikes. Full-path contact evidence
showed that the earlier abort could coincide with peg-target contact before
descent, axis-aligned IK removed that tilted-peg failure mode,
collision-pair attribution showed `link_5` could hit the target plate, and the
tool-tip correction removed that reproduced clearance collision. Do not loosen
the no-contact XY gate, approach Z preconditions, or hard-force abort to hide
the remaining tracking failure.
