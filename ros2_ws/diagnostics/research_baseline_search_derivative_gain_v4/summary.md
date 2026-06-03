# Search Derivative Gain 10.0 with Gain 2000

Milestone: `research_baseline_search_derivative_gain_v4`

Status: `validated_failed_closed`

## Purpose

Isolate the D-term effect at the canonical `position_gain=2000.0` by setting
`position_derivative_gain=10.0`. The previous diagnostic
(`research_baseline_search_derivative_gain_v3`) combined `position_gain=3000.0`
with `position_derivative_gain=10.0` and reached the best `0.0020 m` SEARCH
window so far. This diagnostic uses the same D-term at the canonical gain to
attribute the v3 improvement to the D-term, the gain, or both.

## Commands

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_search_derivative_gain_v4
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  position_gain:=2000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=5.0 \
  tracking_log_dir:=diagnostics/research_baseline_search_derivative_gain_v4
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_derivative_gain_v4
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_derivative_gain_v4
ros2 run thesis_bringup search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_derivative_gain_v4
```

## Result

- Syntax check passed.
- Targeted `colcon build --symlink-install --packages-select kuka_task_control
  thesis_bringup` passed.
- Headless Gazebo launched, spawned `lbr_iisy6_r1300`, activated
  `joint_state_broadcaster` and `joint_trajectory_controller`, and completed a
  controller-driven trial.

## Runtime Outcome

- outcome: `ABORTED`
- reason: `SEARCH timeout (45s). XY error 0.0024m remains above physical clearance 0.0010m.`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `129.2 N`
- max raw force norm: `210.3 N`
- contact-topic samples: `0`
- observed trajectory commands: `10`
- controller-state p95 max absolute joint-position error: `0.011697 rad`
- `SEARCH` recenter attempts: `7`
- final SEARCH XY: `0.0024 m` (reporter) / `0.000342 m` (xy_stability mean over window)

## Analyzer Evidence

`xy_stability_analysis.md`:

- SEARCH samples: `4500`
- SEARCH min XY: `0.000028 m`
- SEARCH mean XY: `0.002510 m`
- SEARCH final XY: `0.000342 m` (best SEARCH final XY in this line of work)
- SEARCH best estimated `0.0010 m` window: `3` task ticks
- SEARCH best estimated `0.0020 m` window: `7` task ticks

`hold_window_reference_analysis.md`:

- hold-like command count: `7`
- best feedback `0.0010 m` hold window: `3` task ticks
- recenter hold feedback mean XY around `0.0022-0.0027 m`
- p95 JTC joint error on recenter holds: about `0.0083-0.0085 rad`

`search_tracking_sensitivity_analysis.md`:

- max centered-hold p95 actual XY drift: `0.004123 m`
- max centered-hold p95 joint error: `0.008472 rad`
- dominant p95 XY contributor: `joint_1`
- linearization residual p95: about `0.000020 m`

## Comparison

| Run | Gain | D-term | Outcome | Best 1 mm window | Best 2 mm window | SEARCH final XY m | Centered-hold p95 actual XY drift m | Centered-hold p95 JTC joint error rad |
| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| streak-preservation | 2000 | 0 | ABORTED SEARCH timeout | 3 | n/a | 0.002779 | 0.003981 | 0.008404 |
| derivative_gain_v1 | 2000 | 0.5 | ABORTED SEARCH timeout | 3 | n/a | 0.003823 | 0.003919 | 0.008617 |
| derivative_gain_v2 | 2000 | 5.0 | ABORTED SEARCH timeout | 3 | 7 | 0.002667 | 0.004018 | 0.008434 |
| derivative_gain_v3 | 3000 | 10.0 | ABORTED SEARCH timeout | 3 | 10 | 0.002974 | 0.004029 | 0.008481 |
| derivative_gain_v4 | 2000 | 10.0 | ABORTED SEARCH timeout | 3 | 7 | 0.000342 | 0.004123 | 0.008472 |

`position_derivative_gain=10.0` at the canonical `position_gain=2000.0`
produced the best SEARCH final XY (`0.000342 m`, below the physical
`0.0010 m` clearance) seen in this line of work, but the best 1 mm SEARCH
window stayed at `3` task ticks and SEARCH still failed closed before
INSERT.

## Interpretation

The D-term at the canonical gain reaches a tighter final SEARCH XY than
either the canonical-D baseline or the higher-gain variant, but the
1 mm sustained window is the binding constraint and the D-term alone does
not move it. The v3 (combined higher-gain / higher-D-term) result of
`10` task ticks in the 2 mm window remains the best SEARCH 2 mm window
seen so far, and the v4 (D-term at canonical gain) result of
`0.000342 m` SEARCH final XY is the best absolute end-of-window
centering. Both diagnostics are safe, fail-closed, and do not loosen
the safety gates. The next iteration should look at the binding
constraint (1 mm sustained window) rather than at gain/D-term scaling.
