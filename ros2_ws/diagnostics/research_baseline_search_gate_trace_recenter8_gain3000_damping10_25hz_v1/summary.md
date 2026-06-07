# SEARCH Gate Trace Hook Diagnostic

Date: 2026-06-07

Milestone: `research_baseline_search_gate_trace_recenter8_gain3000_damping10_25hz_v1`

## Code Change

`admittance_insertion_node` now accepts `tracking_log_dir` and writes a
task-side `search_gate_trace.csv` when SEARCH is entered. The trace records the
online gate inputs and decisions that passive replay cannot see directly:

- task timestamp and SEARCH tick;
- state-entry ticks and SEARCH elapsed time;
- post-command settle elapsed time;
- `ready_to_count`, `settling_window_active`, and `command_still_running`;
- current search step and recenter count;
- convergence counter before and after the branch;
- measured XY error and peg Z;
- branch decision such as `settling_count`, `post_settle_count`,
  `recenter_command`, `spiral_command`, or `timeout_abort`.

The launch file passes the existing `tracking_log_dir` launch argument into the
task node. This is diagnostic-only instrumentation: it does not change the
`0.0010 m` clearance, `8`-tick gates, SEARCH transitions, or INSERT behavior.

## Command

```bash
rm -rf build/kuka_task_control install/kuka_task_control build/thesis_bringup install/thesis_bringup
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup

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
  tracking_log_dir:=diagnostics/research_baseline_search_gate_trace_recenter8_gain3000_damping10_25hz_v1
```

## Result

- build result: `kuka_task_control` and `thesis_bringup` completed successfully
- launch exit code: `0`
- final status field: `DONE`
- final outcome: `ABORTED`
- reason: `INSERT handoff settle timeout: XY error 0.0016m did not remain within physical clearance 0.0010m for 8 ticks before descent.`
- phase sequence: `MOVING_TO_START -> APPROACH -> INSERT -> ABORT -> DONE`
- SEARCH was not entered because APPROACH reached pre-insertion XY `0.0009 m`
- `search_gate_trace.csv` was created with only its header row; no SEARCH gate rows were expected for this run
- insertion depth: `0.0000 m`
- final insertion XY error: `0.0016 m`
- max raw `|Fz|`: `101.55 N`
- max force norm: `170.63 N`
- positive Gazebo contact-topic samples: `0`
- shutdown limitation: child observers and bridge nodes still report shutdown-time signal/context errors after the task-node-driven launch shutdown, even though the task node writes DONE and the launch exits with code `0`

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

- INSERT best estimated `0.0010 m` stability window: `4` task ticks.
- INSERT best estimated `0.0020 m` stability window: `30` task ticks.
- INSERT final XY error in the observer stream: `0.001588 m`.
- Hold-like command count: `1`.
- Hold-like best feedback `0.0010 m` window: `5` ticks.
- Max centered-hold p95 actual XY drift: `0.002246 m`.
- Max centered-hold p95 JTC joint-position error: `0.007544 rad`.
- Trajectory observer max abs position error: `0.100291 rad`; controller-state max abs position error: `0.017568 rad`; controller-state p95 max abs position error: `0.011995 rad`.

## Interpretation

The task-side SEARCH trace hook is implemented, built, and non-interfering in a
full headless launch. This run does not resolve the SEARCH passive/online
counter discrepancy because SEARCH was bypassed. It does add future evidence
plumbing and reconfirms the current physical blocker: pre-insert/INSERT
handoff feedback does not remain inside the strict `0.0010 m` clearance for
`8` ticks, so descent is correctly withheld and no insertion success is
claimed.
