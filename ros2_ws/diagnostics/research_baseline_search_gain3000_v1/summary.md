# Search Gain 3000 with No D-Term

Milestone: `research_baseline_search_gain3000_v1`

Status: `validated_failed_closed`

## Purpose

Isolate the effect of the position gain alone by setting `position_gain=3000.0`
and `position_derivative_gain=0.0` (the canonical default). This is the
upper-left corner of the 2x2 (gain, D-term) grid at the canonical damping
scale of `5.0`, and provides the missing control point for attributing the
v3 and v4 results to the gain, the D-term, or their combination.

## Commands

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_search_gain3000_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  position_gain:=3000.0 \
  position_derivative_gain:=0.0 \
  joint_damping_scale:=5.0 \
  tracking_log_dir:=diagnostics/research_baseline_search_gain3000_v1
ros2 run thesis_bringup xy_stability_analyzer diagnostics/research_baseline_search_gain3000_v1
ros2 run thesis_bringup hold_window_reference_analyzer diagnostics/research_baseline_search_gain3000_v1
ros2 run thesis_bringup search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_gain3000_v1
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
- reason: `SEARCH timeout (45s). XY error 0.0038m remains above physical clearance 0.0010m.`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `131.8 N`
- max raw force norm: `209.1 N`
- contact-topic samples: `0` (no `Contact` rise above `79.7 N` baseline offset)
- observed trajectory commands: `10`
- controller-state p95 max absolute joint-position error: `0.011611 rad`
- `SEARCH` recenter attempts: `3`
- final SEARCH XY: `0.0038 m` (reporter) / `0.002071 m` (xy_stability mean over window)

## Analyzer Evidence

`xy_stability_analysis.md`:

- SEARCH samples: `4500`
- SEARCH min XY: `0.000026 m`
- SEARCH mean XY: `0.003125 m`
- SEARCH final XY: `0.002071 m`
- SEARCH best estimated `0.0010 m` window: `2` task ticks
- SEARCH best estimated `0.0020 m` window: `4` task ticks
- MOVING_TO_START best estimated `0.0010 m` window: `3` task ticks
- MOVING_TO_START best estimated `0.0020 m` window: `8` task ticks

`hold_window_reference_analysis.md`:

- hold-like command count: `7`
- best feedback `0.0010 m` hold window: `2` task ticks
- recenter hold feedback mean XY around `0.0022-0.0027 m`
- p95 JTC joint error on recenter holds: about `0.0082-0.0084 rad`

`search_tracking_sensitivity_analysis.md`:

- max centered-hold p95 actual XY drift: `0.004013 m`
- max centered-hold p95 joint error: `0.008314 rad`
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
| gain3000_v1 | 3000 | 0 | ABORTED SEARCH timeout | 2 | 4 | 0.002071 | 0.004013 | 0.008314 |

The 2x2 grid of (gain, D-term) at the canonical damping scale is now
complete:

| | D-term = 0 | D-term = 0.5 | D-term = 5.0 | D-term = 10.0 |
| --- | ---: | ---: | ---: | ---: |
| gain = 2000 | 3 / n/a / 0.002779 | 3 / n/a / 0.003823 | 3 / 7 / 0.002667 | 3 / 7 / 0.000342 |
| gain = 3000 | 2 / 4 / 0.002071 | (not run) | (not run) | 3 / 10 / 0.002974 |

(table cells: `SEARCH best 1 mm window / best 2 mm window / SEARCH final XY m`)

## Interpretation

Raising `position_gain` from `2000.0` to `3000.0` without any D-term
actually makes SEARCH worse on the offline window estimate (best 1 mm
window drops from 3 to 2 ticks, best 2 mm window drops to 4 ticks,
SEARCH mean XY rises from about `0.0022 m` to about `0.0031 m`). The
higher gain amplifies the controller's oscillation around the SEARCH
target before the D-term is added, and the 1 mm sustained window
binding constraint remains unaddressed. The combined
`position_gain=3000.0 / position_derivative_gain=10.0` run (v3) is
still the best 2 mm SEARCH window seen so far, and the
`position_gain=2000.0 / position_derivative_gain=10.0` run (v4) is
still the best SEARCH final XY. The next iteration should look beyond
the position controller gains and D-term (controller rate, controller
type, or higher-level feedback shaping).
