# research_baseline_velocity_state_500hz_gain3000_damping10_25hz_v1

Date: 2026-06-07

## Purpose

Test whether a 500 Hz velocity-state `joint_trajectory_controller`
configuration improves the strict physical-clearance insertion path without
relaxing the fixed `0.0010 m` radial clearance or `8`-tick stability gates.

This run followed a discarded stale-install attempt that launched an older
iisy3 path. The invalid diagnostics directory was removed. The retained run was
started only after deleting and rebuilding the selected package build/install
trees for `kuka_task_control`, `safety_layer`, and `thesis_bringup`.

## Commands

```bash
rm -rf build/kuka_task_control install/kuka_task_control \
  build/thesis_bringup install/thesis_bringup \
  build/safety_layer install/safety_layer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  kuka_task_control safety_layer thesis_bringup
```

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 480s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml \
  search_recenter_duration_s:=8.0 \
  search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 \
  tracking_log_dir:=diagnostics/research_baseline_velocity_state_500hz_gain3000_damping10_25hz_v1
```

## Runtime Result

- launch wrapper exit code: `0`
- final outcome: `SUCCESS`
- final reason: `Full cycle completed. Insertion depth 0.020m, contact 60.2N during INSERT.`
- failed phase: none reported
- timeout occurrence: no task timeout
- safety abort occurrence: no
- insertion depth: `0.0202 m`
- final INSERT XY error: `0.000848 m`
- APPROACH completion XY error: `0.000226 m`
- max INSERT contact reported by task node: `60.2 N`
- max task contact over the trial: `68.1 N`
- max raw `|Fz|`: `96.97 N`
- max force norm: `170.02 N`
- positive Gazebo contact-topic samples: `0`
- SEARCH gate trace rows: `0`

## Phase Evidence

The run reached `MOVING_TO_START`, `APPROACH`, `INSERT`, `RETREAT`, and `DONE`.
`SEARCH` was bypassed because `APPROACH` ended already inside the physical
clearance gate. This run therefore does not validate SEARCH stability.

Observed task-node phase details from the retained launch:

- `MOVING_TO_START`: OK, Cartesian error about `0.0003 m`;
- `APPROACH`: OK, Cartesian error about `0.0146 m`, final XY about `0.0002 m`;
- `INSERT`: OK, insertion depth `0.0202 m`, final INSERT XY `0.0008 m`;
- `RETREAT`: OK;
- final state: `DONE`.

## Passive Analyses

- `trajectory_tracking_summary.md`: `51532` JTC controller-state samples,
  controller-state max abs position error `0.008829 rad`, final error
  `0.003119 rad`.
- `wrench_state_summary.md`: `10485` wrench/state samples, max raw `|Fz|`
  `96.967518 N`, max force norm `170.020007 N`.
- `contact_state_summary.md`: `0` contact-topic samples and `0` positive
  contact samples from `/gazebo/contacts/*`.
- `xy_stability_analysis.md`: INSERT mean XY `0.000560 m`, final INSERT XY
  `0.000848 m`, and `2362/2532` INSERT samples inside `0.0010 m`.
- `hold_window_reference_analysis.md`: best hold-like feedback 1 mm stability
  was `97` estimated 25 Hz ticks; the 20 s INSERT hold/descent command target
  depth was `0.020000 m`.
- `search_tracking_sensitivity_analysis.md`: max centered-hold p95 actual XY
  drift was `0.000961 m`, max centered-hold p95 joint error was
  `0.003774 rad`.
- `search_gate_trace_analysis.md`: `0` rows because SEARCH was not entered.

## Decision

Keep the 500 Hz velocity-state controller configuration as a diagnostic
variant. This is the first retained strict-gate single-run event in the recent
control line that reached task outcome `SUCCESS` with measured insertion depth
and sub-millimetre final INSERT XY.

Do not claim final autonomous peg-in-hole success. This is one simulated
insertion-depth event with wrench-derived task contact evidence, no positive
Gazebo contact-topic rows, and no SEARCH traversal. Repeat validation with the
same controller config is required before treating it as a robust baseline.

Known limitations remain:

- SEARCH was bypassed, so the unresolved SEARCH sustained-centering issue is
  not proven fixed.
- Gazebo contact-topic observability reported zero positive samples.
- `ros_gz_bridge` and `gzserver` still exited with `-11` during teardown.
- This is a single run, not repeated validation.
