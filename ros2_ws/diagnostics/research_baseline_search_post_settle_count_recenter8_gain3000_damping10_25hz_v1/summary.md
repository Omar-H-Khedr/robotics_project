# SEARCH Post-Settle Count Diagnostic

Date: 2026-06-07

Milestone: `research_baseline_search_post_settle_count_recenter8_gain3000_damping10_25hz_v1`

## Command

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 210s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  search_recenter_duration_s:=8.0 \
  search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 \
  tracking_log_dir:=diagnostics/research_baseline_search_post_settle_count_recenter8_gain3000_damping10_25hz_v1
```

## Result

- launch exit code: `0`
- final outcome: `ABORTED`
- final status field: `DONE`
- reason: `INSERT handoff settle timeout: XY error 0.0023m did not remain within physical clearance 0.0010m for 8 ticks before descent.`
- phase results:
  - `MOVING_TO_START`: success, duration `40.16 s`, final Cartesian error `0.000946 m`, joint error `0.005148 rad`
  - `APPROACH`: success, duration `11.72 s`, final Cartesian error `0.014283 m`, joint error `0.025182 rad`
  - `INSERT`: failed, duration `12.08 s`, timed out during handoff settle
- insertion depth: `0.0000 m`
- pre-insertion XY error: `0.0009 m`
- final insertion XY error: `0.0023 m`
- fixed physical radial clearance: `0.0010 m`
- fixed sustained-clearance requirement: `8` ticks at `25.0 Hz`
- max raw `|Fz|`: `103.01 N`
- max force norm: `169.95 N`
- positive Gazebo contact-topic samples: `0`

## Passive Analyses

Files retained:

- `xy_stability_analysis.md`
- `xy_stability_analysis.json`
- `hold_window_reference_analysis.md`
- `hold_window_reference_analysis.json`
- `search_tracking_sensitivity_analysis.md`
- `search_tracking_sensitivity_analysis.json`
- `trajectory_tracking_summary.md`
- `wrench_state_summary.md`
- `contact_state_summary.md`

Key metrics:

- INSERT best estimated `0.0010 m` stability window: `9` task ticks in passive replay.
- INSERT final XY error in the observer stream: `0.000557 m`; the task outcome reports the handoff timeout at `0.002301 m`, so instantaneous samples still drift around the gate and must not be interpreted as successful descent readiness.
- Hold-like command count: `1`.
- Hold-like best feedback `0.0010 m` window: `5` ticks.
- Max centered-hold p95 actual XY drift: `0.002226 m`.
- Max centered-hold p95 JTC joint-position error: `0.007530 rad`.
- Trajectory observer max abs position error: `0.106609 rad`; controller-state max abs position error: `0.017601 rad`; controller-state p95 max abs position error: `0.011719 rad`.

## Interpretation

This run validates a real SEARCH sequencing improvement: after a SEARCH/recenter command has finished and the fixed settle interval has elapsed, the state machine now preserves and counts a valid inside-clearance sample instead of immediately publishing another command. With the same strict `0.0010 m` / `8`-tick gate, the run reached INSERT and therefore exercised the handoff path.

This is not insertion success. The final descent command was not accepted because handoff feedback did not remain inside physical clearance for the required sustained window. The current blocker remains no-contact feedback stability at SEARCH/INSERT handoff, not lack of timeouts or success classification.
