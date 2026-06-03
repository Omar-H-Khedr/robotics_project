# Search Post-Command Stability Gate

Milestone: `research_baseline_search_post_command_stability_gate_v1`

Status: `validated_failed_closed`

## Purpose

Prevent SEARCH from counting transient inside-clearance samples while a
trajectory command is still executing. The task now records a
`_search_stability_ready_s` timestamp whenever SEARCH publishes a recenter or
spiral step command. Sustained physical-clearance ticks are counted only after
that command duration has elapsed.

This preserves the `0.0010 m` physical radial clearance gate and does not
loosen SEARCH or INSERT preconditions.

## Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_search_post_command_stability_gate_v1_repeat2
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_post_command_stability_gate_v1_repeat2
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_post_command_stability_gate_v1_repeat2
```

## Result

- Syntax check passed.
- Targeted `colcon build --packages-select kuka_task_control thesis_bringup`
  passed.
- Headless Gazebo launched, spawned `lbr_iisy6_r1300`, activated
  `joint_state_broadcaster` and `joint_trajectory_controller`, and completed a
  controller-driven trial.
- The wrapper exited `124` after the task outcome had been printed and observer
  summaries were flushed.

## Runtime Outcome

- outcome: `ABORTED`
- reason: `SEARCH timeout (45s). XY error 0.0028m remains above physical clearance 0.0010m.`
- insertion depth: `0.0000 m`
- final outcome XY field: `0.0019 m`
- max raw `|Fz|`: `129.8 N`
- max raw force norm: `209.0 N`
- contact-topic samples: `0`
- observed trajectory commands: `10`
- controller-state samples: `54510`
- controller-state p95 max absolute joint-position error: `0.011381 rad`

## Analyzer Evidence

`xy_stability_analysis.md`:

- SEARCH samples: `4500`
- SEARCH min XY: `0.000025 m`
- SEARCH mean XY: `0.002371 m`
- SEARCH final XY: `0.002798 m`
- SEARCH best estimated `0.0010 m` window: `4` task ticks
- SEARCH best estimated `0.0020 m` window: `13` task ticks

`hold_window_reference_analysis.md`:

- hold-like command count: `7`
- best feedback `0.0010 m` hold window: `2` task ticks
- centered/near-centered references still produced feedback mean XY around
  `0.0022-0.0025 m` for the recenter holds.

## Interpretation

The change makes SEARCH fail closed more honestly: transient inside-clearance
samples during command execution are no longer counted toward the sustained
gate. The system still cannot hold feedback inside the `0.0010 m` radial
clearance for the required `8` post-command task ticks, so INSERT remains
correctly blocked. No insertion success is claimed.
