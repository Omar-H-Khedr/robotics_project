# Project Context

Last reviewed: 2026-06-03

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

The strongest historical single-run iisy6 insertion-depth evidence is
`diagnostics/research_baseline_insert_sim_time_completion_v4`, which passed the
older depth/contact criteria:

- final outcome under older criteria: `SUCCESS`;
- insertion depth: `0.0191 m`;
- task-side insert-contact evidence: `60.1 N`;
- max raw `|Fz|`: `133.33 N`;
- final phase sequence: MOVING_TO_START, APPROACH, SEARCH, INSERT, RETREAT all OK.

The current stricter validation is
`diagnostics/research_baseline_insert_physical_xy_gate_v1`:

- final outcome: `DEGRADED`;
- insertion depth: `0.0177 m`;
- task-side insert-contact evidence: `55.4 N`;
- final insertion XY error: `0.0030 m`;
- physical radial clearance: `0.0010 m`;
- reason: side-loaded insertion must not count as physical success.

The latest safety validation is
`diagnostics/research_baseline_insert_sideload_abort_v1`:

- final outcome: `ABORTED`;
- abort depth: `0.0011 m`;
- abort XY error: `0.0032 m`;
- passive contact-topic rows: `2`;
- max passive contact-topic force: `0.000000 N`;
- reason: fail closed on side-loaded INSERT before deeper invalid extraction.

The latest INSERT drift diagnostic is
`diagnostics/research_baseline_insert_xy_drift_diagnostic_v1`:

- side-load abort run pre-command final XY error: `0.001569 m`;
- side-load abort run first meaningful depth: `0.001316 m` at XY error `0.002488 m`;
- prior physical-XY-gate run first side-load: `0.001274 m` depth at XY error `0.002153 m`;
- reason: controller-state feedback can violate the `0.0010 m` physical radial clearance before or during early INSERT.

The latest runtime safety validation is
`diagnostics/research_baseline_insert_precontact_clearance_gate_v1`:

- final outcome: `ABORTED`;
- reason: no-contact INSERT XY error `0.0027 m` exceeded physical clearance before meaningful depth;
- insertion depth: `0.0000 m`;
- pre-insertion XY after SEARCH: `0.0006 m`;
- positive contact-topic samples: `0`;
- reason: fail closed before descending into the hole when XY feedback drifts outside physical clearance.

The latest rejected INSERT-path diagnostic is
`diagnostics/research_baseline_insert_cartesian_descent_v1`:

- tested change: centered, axis-aligned, multi-waypoint Cartesian INSERT descent;
- final outcome: `ABORTED`;
- first clearance violation: `0.005 s` after INSERT command receipt;
- max physical depth: `0.0000 m`;
- decision: source reverted because waypoint-only INSERT did not preserve physical clearance.

The latest INSERT handoff reference diagnostic is
`diagnostics/research_baseline_insert_handoff_reference_v1`:

- rejected Cartesian descent reference stayed inside physical clearance for the analyzed first `0.5 s`;
- feedback violated physical clearance `0.005 s` after INSERT command receipt;
- max reference XY error: `0.000510 m`;
- max feedback XY error: `0.004264 m`;
- decision: the next implementation should stabilize INSERT handoff feedback or add bounded pre-insert settling, not retry waypoint-only descent or loosen clearance.

The latest INSERT handoff settle validation is
`diagnostics/research_baseline_insert_handoff_settle_v1`:

- final outcome: `ABORTED`;
- no final descent-to-`z=0.790 m` INSERT command was published;
- insertion depth: `0.0000 m`;
- contact-topic samples: `0`;
- reason: handoff feedback exceeded physical clearance before meaningful depth;
- decision: keep the handoff descent gate as a safety improvement, then make SEARCH convergence sustained rather than accepting a transient inside-clearance sample.

The latest SEARCH sustained-clearance validation is
`diagnostics/research_baseline_search_sustained_clearance_v1`:

- final outcome: `ABORTED`;
- reason: `SEARCH timeout (45s). XY error 0.0028m remains above tolerance.`;
- no INSERT phase was entered;
- insertion depth: `0.0000 m`;
- SEARCH samples inside physical clearance: `308/4500`;
- longest consecutive inside-clearance run: `2` observer samples;
- decision: keep the sustained SEARCH gate; improve no-contact centering stability before attempting INSERT.

The latest rejected SEARCH centered-hold diagnostic is
`diagnostics/research_baseline_search_centered_hold_v1`:

- final outcome: `ABORTED`;
- reason: `SEARCH timeout (45s). XY error 0.0048m remains above tolerance.`;
- no INSERT phase was entered;
- SEARCH samples inside physical clearance: `365/4500`;
- longest consecutive inside-clearance run: `4` observer samples;
- decision: source reverted because centered hold did not meet the sustained gate.

The latest XY stability diagnostic is
`research_baseline_xy_stability_analyzer_v1`:

- analyzer evidence:
  `diagnostics/research_baseline_search_sustained_clearance_v1/xy_stability_analysis.md`
  and
  `diagnostics/research_baseline_search_centered_hold_v1/xy_stability_analysis.md`;
- sustained-clearance SEARCH best estimated 1 mm window: `2` controller ticks;
- centered-hold SEARCH best estimated 1 mm window: `2` controller ticks;
- both runs reached `6` estimated SEARCH ticks inside the older `0.0020 m`
  band, but not inside the physical `0.0010 m` clearance;
- decision: keep the physical clearance gate and target feedback/control
  stability before attempting INSERT again.

The latest active SEARCH behavior is
`diagnostics/research_baseline_search_recenter_4mm_v1_repeat2`:

- final outcome: `ABORTED`;
- reason: `SEARCH timeout (45s). XY error 0.0017m remains above tolerance.`;
- no INSERT phase was entered;
- contact-topic samples: `0`;
- best estimated SEARCH `0.0010 m` window: `4` controller ticks;
- best estimated SEARCH `0.0020 m` window: `8` controller ticks;
- decision: keep bounded 4 mm recentering as an improvement, but continue
  treating sustained no-contact centering as unresolved.

The latest hold-window reference diagnostic is
`research_baseline_hold_window_reference_analyzer_v1`:

- analyzer evidence:
  `diagnostics/research_baseline_search_recenter_4mm_v1/hold_window_reference_analysis.md`
  and
  `diagnostics/research_baseline_search_recenter_4mm_v1_repeat2/hold_window_reference_analysis.md`;
- direct INSERT handoff hold best feedback `0.0010 m` window: `3` estimated
  task ticks;
- repeated SEARCH recenter holds best feedback `0.0010 m` window: `2`
  estimated task ticks;
- decision: hold references can be centered or near-centered, but feedback
  stability remains below the required `8` ticks. Continue with feedback
  stabilization or command sequencing, not gate loosening.

Repeated validation on 2026-06-01 produced 0/3 physical successes:

- one DEGRADED INSERT with only 0.0037 m depth and a 1237.45 N peak raw Fz spike;
- one ABORTED INSERT with a 3716.2 N peak raw Fz spike;
- one SEARCH timeout/no-outcome before final insertion evaluation.

This is not robust autonomous peg-in-hole success. The honest claim is now:

**controller-driven insertion-depth/contact event; no validated physical success under the latest clearance-aware criteria.**

## Known Open Risks

- Repeat validation is complete for the current controller revision and failed: 0/3 physical successes.
- MOVING_TO_START is non-deterministic, with roughly 1-in-3 failures reported.
- MOVING_TO_START and APPROACH can have large Cartesian tracking errors.
- Peak raw Fz spikes around 1237 N and 3716 N have now been observed in repeated validation.
- Multi-point INSERT trajectory behavior is broken; current INSERT uses a single-point trajectory.
- Contact/gravity estimation depends on median Fz baseline validity and needs more validation.
- Historical v4 INSERT contact is task F/T evidence; the passive Gazebo contact observer recorded contact-topic rows only during RETREAT in that run.
- Successful-insert RETREAT still produced contact-topic force up to `249.593329 N`, so withdrawal contact reduction is the next safety-critical blocker.
- A staged vertical-lift-then-home withdrawal diagnostic was rejected because it worsened RETREAT contact to `486.746287 N` over `307` rows.
- Withdrawal contact timing analysis showed the staged run's contact occurred during vertical extraction, not later home motion; the peak occurred at `0.019732 m` insertion depth with about `0.006 m` XY error.
- The latest clearance-aware validation downgraded depth/contact to `DEGRADED` because final insertion XY error `0.0030 m` exceeds the `0.0010 m` physical radial clearance.
- INSERT side-load abort now prevents continuing deeper after inserted-depth XY drift exceeds physical clearance, but this means the current baseline fails honestly before physical success.
- INSERT XY drift diagnostics motivated the no-contact INSERT gate against the `0.0010 m` physical radial clearance before deeper descent.
- INSERT pre-contact clearance gating now prevents descent when XY feedback leaves physical clearance before meaningful depth. The remaining blocker is reducing or constraining the one-point INSERT path drift after SEARCH centers the peg.
- A centered multi-waypoint Cartesian INSERT descent was tested and rejected; the next attempt should address immediate post-INSERT command handoff/hold dynamics, not just add more waypoints.
- INSERT handoff reference analysis shows that a centered multi-waypoint reference can remain within physical clearance while feedback leaves clearance within `0.005 s`, so the problem is now feedback/plant stabilization at handoff rather than a simple FK target error.
- INSERT handoff settle prevents publishing the final descent when feedback is already invalid, but validation still aborts. SEARCH convergence is now suspect because it accepted one transient inside-clearance sample and handed off with feedback already outside clearance.
- Sustained SEARCH clearance gating now prevents that transient handoff. The active blocker is stable no-contact centering near the hole surface, not success classification or INSERT descent timing.
- A centered SEARCH hold was tested and rejected; it was safe but insufficient. Continue treating sustained no-contact centering as the blocker.
- Per-state XY stability analysis confirms both recent SEARCH runs only held physical clearance for two estimated state-loop ticks. Do not treat sub-millimeter minima as readiness for INSERT.
- SEARCH recenter-on-coarse-band improved the best estimated physical-clearance window to four state-loop ticks without contact-topic samples, but still timed out before INSERT. Continue improving no-contact stability rather than loosening gates.
- Widening the recenter trigger to `0.0040 m` improved final SEARCH XY to `0.0017 m` in the SEARCH-exercising repeat, but still held physical clearance for only four estimated state-loop ticks. INSERT must remain blocked.
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
- A 2026-06-03 controller-state tracking validation added direct JTC `controller_state` recording and made the MOVING_TO_START analyzer prefer that source when available. The canonical run still aborted safely before descent after 120.0 s in `MOVING_TO_START` with final logged `xy_err=0.013 m`, zero insertion depth, zero contact-topic samples, and only `143 / 15930` controller-state samples inside the strict 2 mm XY band.
- A 2026-06-03 endpoint-hold dynamics analyzer isolated the post-command hold window from that same run. The hold window had X/Y/Z ranges `0.041273 / 0.035843 / 0.033742 m`, zero strict 10 Hz bins, and largest feedback range on `joint_1` at `0.057121 rad`. This points to hold dynamics/control authority rather than a simple target-frame offset.
- A 2026-06-03 `joint_damping_scale:=5.0` diagnostic improved the baseline enough to satisfy MOVING_TO_START and enter APPROACH without contact-topic samples or high raw-force regression. It still aborted before INSERT because the approach ended at `z=0.849622 m` against a `z=0.830000 m` target, above the preserved `0.8450 m` force-safe precondition.
- A 2026-06-03 approach Z-precondition gate correction made `APPROACH` completion require `peg_z <= 0.8450 m`, matching the preserved INSERT precondition. With `joint_damping_scale:=5.0`, validation reached `APPROACH complete` only after `peg_z=0.8417 m`, then ran `SEARCH` and `INSERT`. The insert trajectory still reported `physical_depth=0.0000 m`, contact-topic rows began only in `RETREAT`, and retreat contact reached `1970.434828 N`, so this is not insertion success and makes retreat collision/depth interpretation the next safety-critical blocker.
- A 2026-06-03 insert/retreat contact analyzer confirmed that `HOLE_TOP_Z=0.810 m` is consistent with the target plate top and that the latest failed INSERT missed the commanded final target: target peg-tip Z was `0.790008 m`, minimum feedback Z was `0.811899 m`, and max physical depth was `0.000000 m`. RETREAT contact was attributed to peg-target and right-finger-target collision pairs, so the next motion fix should add clearance-aware retreat behavior after failed insertion.
- A 2026-06-03 retreat clearance-lift fix added vertical peg-tip lift waypoints before moving to `SAFE_HOME`. Validation reached `DONE` with `DEGRADED` outcome because INSERT depth was only `0.0008 m`, but RETREAT contact improved from `1970.434828 N` over `7776` contact rows to `36.335073 N` over `4` rows. This resolves the immediate retreat-collision safety regression while leaving insertion-depth realization as the next blocker.
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
- A 2026-06-03 JTC state-topic instrumentation fix changed the trajectory observer from `/joint_trajectory_controller/state` to `/joint_trajectory_controller/controller_state`. A bounded validation run recorded `4186` JTC state samples, closing the previous zero-state-sample observability gap.

## Current Success Criteria

A state machine reaching DONE is insufficient. A physical success trial requires:

- final trial outcome `SUCCESS`;
- measured insertion depth at least 0.010 m;
- INSERT-phase contact evidence above the configured insertion/contact threshold;
- final inserted XY error within the physical peg/hole radial clearance (`0.0010 m` for the current 25 mm peg / 27 mm hole);
- no safety abort;
- no unresolved timeout that invalidates task execution;
- recorded peak raw Fz and Cartesian-error metrics.

Robust success requires repeated validation with a documented success rate and failure modes.

## Next Technical Milestone

`research_baseline_insert_centering_and_extraction_contact_reduction`

Reason: the current side-load abort evidence shows the controller now fails
closed when inserted-depth XY drift exceeds the `0.0010 m` physical clearance.
This prevents deeper invalid extraction contact, but it also means the baseline
still has no validated physical insertion success. The next implementation
should reduce XY drift during INSERT and then rerun the same analyzers before
attempting repeated validation.

Historical context: after the peg-tip frame correction, the canonical blocker
was stable above-hole holding. The corrected tool frame removed the reproduced
`link_5` target-plate collision, but the controller did not hold the peg inside
the strict 2 mm no-contact XY gate for the required consecutive state-machine
ticks. A post-correction slow settle crossed the gate only transiently and was
rejected.

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

The latest controller-state tracking run removes a remaining measurement
ambiguity: the JTC's own reference/feedback/error stream shows the same
transient strict-band crossing and failed hold. Treat this as evidence that the
current blocker is sustained above-hole endpoint stability, not a missing
command capture or analyzer reference reconstruction issue.

The endpoint hold window itself now has a dedicated diagnostic. Its latest
result shows multi-centimeter oscillation after the nominal 40 s trajectory is
complete, with no strict 10 Hz bin fully inside the 2 mm gate. The next control
change should therefore target post-command hold dynamics explicitly and should
be evaluated by this analyzer before any approach or insertion claim.

The 5x damping diagnostic moved the active blocker downstream: strict
MOVING_TO_START stability is achievable with stronger damping, but approach Z
realization still stops above the force-safe INSERT precondition. Future work
should preserve that precondition and investigate approach-depth tracking,
trajectory timing, or damping/gain balance rather than claiming insertion from
an abort-before-INSERT run.

The approach Z-precondition gate now prevents that specific false phase
completion. The remaining blocker is not gate definition: with the same 5x
damping diagnostic the task reached INSERT, but physical insertion depth stayed
at `0.0000 m` and high contact appeared during RETREAT. The next work should
audit insertion-depth geometry, peg/hole collision pairs during and after
INSERT, and retreat path clearance before any success or learning claim.

The insert/retreat contact analyzer resolved the first part of that audit: the
zero depth is consistent with measured peg-tip feedback staying above the plate
top, not a stale hole-top constant. The next implementation should therefore
target failed-insert recovery and retreat clearance, especially preventing the
peg and right finger from sweeping through the target plate during RETREAT.

The retreat clearance-lift implementation substantially reduced retreat
contact without claiming success. The remaining implementation problem is the
INSERT command itself: the peg reaches at most `0.000836 m` physical depth
while the final target is near `z=0.790 m`. Future changes should focus on
insert trajectory execution, timing, and monitored depth progress.

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

The current next step is INSERT path stabilization. The joint-state source
integrity milestone removed one measurement ambiguity; the latest pre-contact
gate now prevents descent when XY drift exceeds physical clearance.

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
