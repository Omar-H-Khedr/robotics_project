# Search Derivative Gain

Milestone: `research_baseline_search_derivative_gain_v1`

Status: `validated_failed_closed`

## Purpose

Test whether a small `position_derivative_gain` at `gz_ros2_control` reduces the
controller-state tracking error during centered no-contact SEARCH holds. The
search tracking sensitivity analyzer identified `joint_1` as the dominant p95
contributor with about `0.0084 rad` p95 joint error and about `0.004 m` p95
peg-tip XY drift. A bounded D-term in the position controller should damp
centered-hold oscillation without lowering the `0.0010 m` physical clearance
gate or relaxing the hard-force abort.

This also exercises the new `position_derivative_gain` plumbing in
`spawn_robot_sdf.py` and `research_baseline.launch.py`. The launch argument
defaults to `0.0` so canonical behavior is unchanged; only explicit overrides
add the `<position_derivative_gain>` SDF element.

## Commands

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_search_derivative_gain_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  position_gain:=2000.0 \
  position_derivative_gain:=0.5 \
  joint_damping_scale:=5.0 \
  tracking_log_dir:=diagnostics/research_baseline_search_derivative_gain_v1
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_derivative_gain_v1
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_derivative_gain_v1
ros2 run thesis_bringup search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_derivative_gain_v1
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
- reason: `SEARCH timeout (45s). XY error 0.0039m remains above physical clearance 0.0010m.`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `128.3 N`
- max raw force norm: `206.5 N`
- contact-topic samples: `26` (only in `ABORT` retreat, peg vs target plate right collision)
- observed trajectory commands: `10`
- controller-state p95 max absolute joint-position error: `0.011385 rad`
- `SEARCH` recenter attempts: `7`
- final SEARCH XY: `0.0039 m`

## Analyzer Evidence

`xy_stability_analysis.md`:

- SEARCH samples: `4500`
- SEARCH min XY: `0.000008 m`
- SEARCH mean XY: `0.002375 m`
- SEARCH final XY: `0.003823 m`
- SEARCH best estimated `0.0010 m` window: `3` task ticks
- SEARCH best estimated `0.0020 m` window: `7` task ticks

`hold_window_reference_analysis.md`:

- hold-like command count: `7`
- best feedback `0.0010 m` hold window: `3` task ticks
- recenter hold feedback mean XY around `0.0022-0.0027 m`
- p95 JTC joint error on recenter holds: about `0.0082-0.0086 rad`

`search_tracking_sensitivity_analysis.md`:

- max centered-hold p95 actual XY drift: `0.003919 m`
- max centered-hold p95 joint error: `0.008617 rad`
- dominant p95 XY contributor: `joint_1`
- linearization residual p95: about `0.000020 m`

## Comparison

| Run | Max centered-hold p95 actual XY drift m | Max centered-hold p95 joint error rad | Best SEARCH 1 mm window ticks | SEARCH final XY m |
| --- | ---: | ---: | ---: | ---: |
| `research_baseline_search_streak_preservation_v1` | 0.003981 | 0.008404 | 3 | 0.002779 |
| `research_baseline_search_derivative_gain_v1` | 0.003919 | 0.008617 | 3 | 0.003823 |

`position_derivative_gain=0.5` did not improve centered-hold tracking error at
the current `position_gain=2000.0` and `joint_damping_scale=5.0`. The
linearization residual is unchanged at about `0.000020 m`, so the controller
state log is still consistent with the Jacobian estimate and `joint_1` is still
the dominant p95 contributor. SEARCH still failed closed before INSERT.

## Interpretation

The change is safe and a fail-closed SEARCH outcome was preserved. The D-term
of `0.5` is too small to materially damp the residual joint oscillation in
this run. A larger D-term, or a tuned D combined with the recently added
`position_derivative_gain` plumbing, is a candidate for the next iteration.
The physical `0.0010 m` clearance gate and the hard-force abort are
unchanged; no insertion success is claimed.
