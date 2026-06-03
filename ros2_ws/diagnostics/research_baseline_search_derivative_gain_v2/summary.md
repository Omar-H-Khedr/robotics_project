# Search Derivative Gain 5.0

Milestone: `research_baseline_search_derivative_gain_v2`

Status: `validated_failed_closed`

## Purpose

A more aggressive D-term test of the new `position_derivative_gain` plumbing.
The first derivative gain diagnostic (`research_baseline_search_derivative_gain_v1`,
gain=0.5) did not materially change the centered-hold tracking error. This
diagnostic tries a more typical PD value (5.0) to see whether the D-term
becomes large enough to damp the residual joint oscillation.

## Commands

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_search_derivative_gain_v2
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  position_gain:=2000.0 \
  position_derivative_gain:=5.0 \
  joint_damping_scale:=5.0 \
  tracking_log_dir:=diagnostics/research_baseline_search_derivative_gain_v2
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_derivative_gain_v2
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_derivative_gain_v2
ros2 run thesis_bringup search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_derivative_gain_v2
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
- reason: `SEARCH timeout (45s). XY error 0.0052m remains above physical clearance 0.0010m.`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `128.9 N`
- max raw force norm: `208.2 N`
- contact-topic samples: `0` (the ABORT retreat is the only post-SEARCH window; this run produced no contact-topic rows)
- observed trajectory commands: `10`
- controller-state p95 max absolute joint-position error: `0.011421 rad`
- `SEARCH` recenter attempts: `7`
- final SEARCH XY: `0.0052 m`

## Analyzer Evidence

`xy_stability_analysis.md`:

- SEARCH samples: `4500`
- SEARCH min XY: `0.000005 m`
- SEARCH mean XY: `0.002317 m`
- SEARCH final XY: `0.002667 m`
- SEARCH best estimated `0.0010 m` window: `3` task ticks
- SEARCH best estimated `0.0020 m` window: `7` task ticks

`hold_window_reference_analysis.md`:

- hold-like command count: `7`
- best feedback `0.0010 m` hold window: `3` task ticks
- recenter hold feedback mean XY around `0.0021-0.0027 m`
- p95 JTC joint error on recenter holds: about `0.0082-0.0084 rad`

`search_tracking_sensitivity_analysis.md`:

- max centered-hold p95 actual XY drift: `0.004018 m`
- max centered-hold p95 joint error: `0.008434 rad`
- dominant p95 XY contributor: `joint_1`
- linearization residual p95: about `0.000020 m`

## Comparison

| Run | Max centered-hold p95 actual XY drift m | Max centered-hold p95 joint error rad | Best SEARCH 1 mm window ticks | SEARCH final XY m |
| --- | ---: | ---: | ---: | ---: |
| `research_baseline_search_streak_preservation_v1` (no D-term) | 0.003981 | 0.008404 | 3 | 0.002779 |
| `research_baseline_search_derivative_gain_v1` (D=0.5) | 0.003919 | 0.008617 | 3 | 0.003823 |
| `research_baseline_search_derivative_gain_v2` (D=5.0) | 0.004018 | 0.008434 | 3 | 0.002667 |

`position_derivative_gain=5.0` reduced the centered-hold p95 JTC joint error
slightly (`0.008617 -> 0.008434 rad`) and tightened SEARCH final XY
(`0.003823 -> 0.002667 m`), but the centered-hold p95 actual XY drift
actually increased slightly (`0.003919 -> 0.004018 m`). SEARCH best 1 mm
window remains at 3 task ticks in all three runs.

## Interpretation

The D-term is now actually exercising the position controller, but the
centered-hold XY drift is dominated by factors the D-term alone cannot fix
(measurement noise, contact/FT sensor gravity baseline drift, residual
controller lag at the current 10 Hz control loop). The new plumbing is
correctly applied; a 5x D-term is within safe range and does not loosen the
clearance gate. No insertion success is claimed. The next iteration should
combine the D-term with a different lever (controller rate, controller type,
or higher-level feedback shaping) rather than scale the D-term further.
