# SEARCH Timing Diagnostic: recenter 8 s, settle 9 s

Date: 2026-06-07

Purpose: test whether extending the SEARCH recenter command duration and
post-command settle window can move the current gain=3000, D=10,
damping-scale-10 configuration past SEARCH without changing the `0.0010 m`
physical clearance gate or the fixed 8-tick stability requirement.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  search_recenter_duration_s:=8.0 \
  search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 \
  tracking_log_dir:=diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1
```

Post-run analyzers:

```bash
python3 -m thesis_bringup.xy_stability_analyzer --state-loop-hz 25.0 diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1
python3 -m thesis_bringup.hold_window_reference_analyzer --state-loop-hz 25.0 diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1
python3 -m thesis_bringup.above_hole_hold_analyzer --state-loop-hz 25.0 diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1
python3 -m thesis_bringup.endpoint_hold_dynamics_analyzer --state-loop-hz 25.0 diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1
python3 -m thesis_bringup.search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_recenter8_settle9_gain3000_damping10_25hz_v1
```

## Outcome

- final outcome: `ABORTED`;
- reason: `INSERT handoff settle timeout: XY error 0.0011m did not remain within physical clearance 0.0010m for 8 ticks before descent.`;
- phase sequence: MOVING_TO_START OK, APPROACH OK, SEARCH OK, INSERT FAIL;
- insertion depth: `0.0000 m`;
- pre-insertion XY error: `0.0007 m`;
- final INSERT handoff XY error: `0.0011 m`;
- max raw `|Fz|`: `101.68 N`;
- max force norm: `169.48 N`;
- positive Gazebo contact-topic samples: `0`;
- final descent command was not published.

## Passive Analysis

- SEARCH converged under the runtime gate with the extended timing knobs.
- SEARCH best estimated `0.0010 m` stability window: `3` ticks in passive
  replay; runtime convergence was satisfied before INSERT handoff.
- INSERT best estimated `0.0010 m` stability window: `5` ticks, below the
  required `8`;
- INSERT best estimated `0.0020 m` stability window: `33` ticks;
- hold-like best feedback `0.0010 m` window: `4` ticks;
- max centered-hold p95 actual XY drift: `0.002232 m`;
- max centered-hold p95 JTC joint-position error: `0.007669 rad`;
- controller-state p95 max joint-position error over the run: `0.011490 rad`.

## Interpretation

Longer SEARCH recenter/settle timing can allow this non-default tuning to pass
SEARCH, but it does not produce physical insertion readiness. The remaining
failure is the same no-contact handoff stability problem: feedback enters
INSERT close to the hole center, then does not remain inside the `0.0010 m`
radial clearance for 8 consecutive 25 Hz ticks. This is not an insertion
success and should not be used as learning data for successful insertion.
