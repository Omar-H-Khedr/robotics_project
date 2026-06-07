# D20 SEARCH Stability Diagnostic

Date: 2026-06-07

## Purpose

Test whether increasing the trajectory controller derivative gain from `10.0`
to `20.0`, while preserving the stricter physical-clearance gates, improves
near-centered SEARCH stability enough to satisfy the online `0.0010 m` / `8`
tick gate.

This was a diagnostic launch-parameter test only. No source change is accepted
from this run.

## Commands

```bash
rm -rf build/kuka_task_control install/kuka_task_control build/thesis_bringup install/thesis_bringup build/safety_layer install/safety_layer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control safety_layer thesis_bringup
source install/setup.bash
rm -rf diagnostics/research_baseline_search_derivative_gain20_recenter8_damping10_25hz_v1
timeout 480s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=20.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  search_recenter_duration_s:=8.0 \
  search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 \
  tracking_log_dir:=diagnostics/research_baseline_search_derivative_gain20_recenter8_damping10_25hz_v1
```

Post-run analyzers:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m thesis_bringup.xy_stability_analyzer --state-loop-hz 25.0 diagnostics/research_baseline_search_derivative_gain20_recenter8_damping10_25hz_v1
python3 -m thesis_bringup.hold_window_reference_analyzer --state-loop-hz 25.0 diagnostics/research_baseline_search_derivative_gain20_recenter8_damping10_25hz_v1
python3 -m thesis_bringup.search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_derivative_gain20_recenter8_damping10_25hz_v1
```

## Result

- launch wrapper exit code: `0`
- final outcome: `ABORTED`
- failed phase: `SEARCH`
- final reason: `SEARCH timeout (45s). Instantaneous XY error 0.0006m is within physical clearance 0.0010m but was not sustained for 8 post-command ticks.`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `100.507143 N`
- max force norm: `170.494237 N`
- positive contact-topic samples: `0`
- data logger output: `/tmp/thesis_logs/insertion_log_20260607_192656.csv`
- Python observer/safety/data logger shutdown: clean
- remaining teardown limitation: `ros_gz_bridge` and `gzserver` exited with `-11`

## SEARCH Gate Trace

Source: `search_gate_trace.csv`

- rows: `1125`
- decisions:
  - `settling_count`: `119`
  - `settling_reset`: `999`
  - `post_settle_count`: `2`
  - `recenter_command`: `4`
  - `timeout_abort`: `1`
- best all-trace `0.0010 m` streak: `5` ticks
- best online counted post-command `0.0010 m` streak: `1` tick
- required online gate: `8` ticks

The online state machine therefore remained correct to reject INSERT, even
though some passive samples and final console metrics were inside the physical
clearance.

## Passive Analysis

`xy_stability_analysis.md`:

- SEARCH samples: `4500`
- SEARCH min XY: `0.000008 m`
- SEARCH mean XY: `0.001249 m`
- SEARCH p95 XY: `0.002397 m`
- SEARCH final XY from observer stream: `0.001034 m`
- SEARCH best estimated `0.0010 m` window: `6` task ticks
- SEARCH best estimated `0.0020 m` window: `33` task ticks

`hold_window_reference_analysis.md`:

- hold-like command count: `4`
- hold-like best feedback `0.0010 m` window: `9` task ticks
- centered reference windows were longer than feedback windows

`search_tracking_sensitivity_analysis.md`:

- centered hold-like command count: `4`
- max centered-hold p95 actual XY drift: `0.002337 m`
- max centered-hold p95 joint error: `0.007646 rad`
- dominant p95 contribution remained `joint_1`
- linearized XY drift closely matched actual drift, so controller tracking
  error remains sufficient to explain the millimeter-scale peg-tip offset.

## Decision

Reject D=20 as a baseline change. It improved some passive indicators relative
to the previous D=10 clean-shutdown run, but the authoritative online SEARCH
gate still observed only `1` counted post-command tick inside the `0.0010 m`
physical clearance, far below the required `8`.

No insertion success is claimed. The next control milestone should address
near-centered feedback stabilization or command sequencing while keeping the
fixed physical clearance gate intact.
