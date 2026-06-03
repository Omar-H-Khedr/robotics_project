# Search Feedback-Compensated Recenter

Milestone: `research_baseline_search_feedback_compensated_recenter_v1`

Status: `rejected_source_reverted`

## Purpose

Test a bounded feedback-compensated SEARCH recenter target. When feedback was
inside the existing `0.0040 m` recenter band but not sustained inside the
`0.0010 m` physical radial clearance, the experiment targeted the opposite
feedback offset, capped to `0.0020 m`.

The strict `0.0010 m` measured-feedback gate was not relaxed, and INSERT was
still blocked unless that gate was sustained for `8` post-command ticks.

## Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_search_feedback_compensated_recenter_v1
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_feedback_compensated_recenter_v1
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_feedback_compensated_recenter_v1
```

## Result

- Syntax check passed.
- Targeted `colcon build --packages-select kuka_task_control thesis_bringup`
  passed.
- Headless Gazebo launched, spawned `lbr_iisy6_r1300`, activated the trajectory
  controller, and completed a controller-driven trial.
- Startup warning: the `joint_state_broadcaster` spawner reported activation
  failure because the controller was already active. The trial still published
  joint states, accepted trajectory commands, and completed.
- The launch wrapper was terminated after the task outcome and observer
  summaries were flushed.

## Runtime Outcome

- outcome: `ABORTED`
- reason: `SEARCH timeout (45s). XY error 0.0033m remains above physical clearance 0.0010m.`
- insertion depth: `0.0000 m`
- final outcome XY field: `0.0017 m`
- max raw `|Fz|`: `128.2 N`
- max raw force norm: `207.5 N`
- contact-topic samples: `0`
- observed trajectory commands: `10`
- controller-state p95 max absolute joint-position error: `0.011345 rad`

## Analyzer Evidence

`xy_stability_analysis.md`:

- SEARCH samples: `4500`
- SEARCH min XY: `0.000040 m`
- SEARCH mean XY: `0.002472 m`
- SEARCH final XY: `0.003319 m`
- SEARCH best estimated `0.0010 m` window: `4` task ticks
- SEARCH best estimated `0.0020 m` window: `11` task ticks

`hold_window_reference_analysis.md`:

- hold-like command count: `7`
- best feedback `0.0010 m` hold window: `4` task ticks
- compensated hold targets intentionally moved reference up to about `0.0020 m`
  away from the hole center.

## Decision

Reject and revert the source change. Although the best hold-window feedback
streak reached `4` ticks, the trial still failed before INSERT, SEARCH mean XY
and final XY were worse than the latest active centered-recenter run, and the
compensated reference intentionally moved away from the hole center. Keeping it
would add behavior complexity without credible downstream progress.
