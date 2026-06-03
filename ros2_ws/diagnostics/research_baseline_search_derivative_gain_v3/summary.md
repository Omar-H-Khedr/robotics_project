# Search Derivative Gain 10.0 with Gain 3000

Milestone: `research_baseline_search_derivative_gain_v3`

Status: `validated_failed_closed`

## Purpose

A combined higher-gain / higher-D-term test of the new `position_derivative_gain`
plumbing. The two prior diagnostics only adjusted the D-term at canonical
`position_gain=2000.0`. This diagnostic uses a much more aggressive
`position_gain=3000.0` together with `position_derivative_gain=10.0` to see
whether a stiffer, damped controller can hold the centered no-contact pose
inside the physical `0.0010 m` clearance window for more task ticks.

## Commands

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_search_derivative_gain_v3
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=5.0 \
  tracking_log_dir:=diagnostics/research_baseline_search_derivative_gain_v3
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_derivative_gain_v3
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_derivative_gain_v3
ros2 run thesis_bringup search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_derivative_gain_v3
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
- reason: `SEARCH timeout (45s). XY error 0.0030m remains above physical clearance 0.0010m.`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `128.2 N`
- max raw force norm: `208.4 N`
- contact-topic samples: `0`
- observed trajectory commands: `10`
- controller-state p95 max absolute joint-position error: `0.011757 rad`
- `SEARCH` recenter attempts: `7`
- final SEARCH XY: `0.0030 m`

## Analyzer Evidence

`xy_stability_analysis.md`:

- SEARCH samples: `4500`
- SEARCH min XY: `0.000043 m`
- SEARCH mean XY: `0.002393 m`
- SEARCH final XY: `0.002974 m`
- SEARCH best estimated `0.0010 m` window: `3` task ticks
- SEARCH best estimated `0.0020 m` window: `10` task ticks

`hold_window_reference_analysis.md`:

- hold-like command count: `7`
- best feedback `0.0010 m` hold window: `3` task ticks
- recenter hold feedback mean XY around `0.0022-0.0027 m`
- p95 JTC joint error on recenter holds: about `0.0082-0.0085 rad`

`search_tracking_sensitivity_analysis.md`:

- max centered-hold p95 actual XY drift: `0.004029 m`
- max centered-hold p95 joint error: `0.008481 rad`
- dominant p95 XY contributor: `joint_1`
- linearization residual p95: about `0.000020 m`

## Comparison

| Run | Gain | D-term | Outcome | Best 1 mm window | Best 2 mm window | SEARCH final XY m | Centered-hold p95 actual XY drift m | Centered-hold p95 JTC joint error rad |
| --- | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: |
| streak-preservation | 2000 | 0 | ABORTED SEARCH timeout | 3 | n/a | 0.002779 | 0.003981 | 0.008404 |
| derivative_gain_v1 | 2000 | 0.5 | ABORTED SEARCH timeout | 3 | n/a | 0.003823 | 0.003919 | 0.008617 |
| derivative_gain_v2 | 2000 | 5.0 | ABORTED SEARCH timeout | 3 | 7 | 0.002667 | 0.004018 | 0.008434 |
| derivative_gain_v3 | 3000 | 10.0 | ABORTED SEARCH timeout | 3 | 10 | 0.002974 | 0.004029 | 0.008481 |

The combined `position_gain=3000.0` and `position_derivative_gain=10.0`
doubled the best `0.0020 m` SEARCH window to `10` task ticks (from `7` in
v2 and `6-7` in earlier diagnostics). The best `0.0010 m` window is still
`3` task ticks, far below the required `8`, so SEARCH still fails closed
before INSERT. The D-term plumbing is correctly applied at this higher
gain, and the safety gates are unchanged.

## Interpretation

The combined higher-gain and higher-D-term run is the first SEARCH variant
to reach `10` estimated `0.0020 m` clearance ticks in the state-loop window.
This indicates that stronger PD control tightens the near-centered XY band
without loosening the `0.0010 m` physical clearance gate. The best
`0.0010 m` window is still well below the required `8` task ticks, so
SEARCH still fails closed before INSERT. The next iteration should look
beyond the position controller gains and D-term: the centered-hold
reference is correct, the controller tracks it within the predicted
Jacobian, and the residual drift is dominated by factors that gain
increases alone cannot remove (sensor noise, contact/FT gravity baseline
drift, residual controller lag at the 10 Hz control loop).
