# Search Streak Preservation

Milestone: `research_baseline_search_streak_preservation_v1`

Status: `validated_failed_closed`

## Purpose

Avoid interrupting an active post-command physical-clearance streak at the end
of the fixed SEARCH settling window. SEARCH now keeps waiting if
`_search_convergence_ticks > 0`, allowing a valid streak to either reach the
required `8` ticks or reset naturally before another SEARCH command is sent.

This does not loosen the `0.0010 m` physical radial clearance gate and does not
allow INSERT unless the sustained gate is satisfied.

## Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_search_streak_preservation_v1
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_streak_preservation_v1
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_streak_preservation_v1
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
- reason: `SEARCH timeout (45s). Instantaneous XY error 0.0010m is within physical clearance 0.0010m but was not sustained for 8 post-command ticks.`
- insertion depth: `0.0000 m`
- final outcome XY field: `0.0026 m`
- max raw `|Fz|`: `132.1 N`
- max raw force norm: `208.3 N`
- contact-topic samples: `0`
- observed trajectory commands: `10`
- controller-state p95 max absolute joint-position error: `0.011391 rad`

## Analyzer Evidence

`xy_stability_analysis.md`:

- SEARCH samples: `4500`
- SEARCH min XY: `0.000059 m`
- SEARCH mean XY: `0.002276 m`
- SEARCH final XY: `0.002779 m`
- SEARCH best estimated `0.0010 m` window: `3` task ticks
- SEARCH best estimated `0.0020 m` window: `8` task ticks

`hold_window_reference_analysis.md`:

- hold-like command count: `7`
- best feedback `0.0010 m` hold window: `3` task ticks
- centered/near-centered references still produced feedback mean XY around
  `0.0022-0.0026 m` for the recenter holds.

## Interpretation

The change is safe and makes SEARCH less likely to interrupt a valid stability
streak, but it did not solve sustained physical centering. INSERT remained
correctly blocked, no contact-topic samples were observed, and no insertion
success is claimed.
