# research_baseline_joint_damping_scale_2p0_v1

Date: 2026-06-03

## Purpose

Test whether increasing converted SDF joint damping improves the corrected
tool-tip `MOVING_TO_START` endpoint hold without relaxing the strict 2 mm
no-contact XY gate, hard-force abort, or descent preconditions.

This was a diagnostic-only launch override. Canonical damping remains
unchanged unless a repeated validation later proves a better setting.

## Validation Command

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 200s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  joint_damping_scale:=2.0 \
  tracking_log_dir:=diagnostics/research_baseline_joint_damping_scale_2p0_v1
```

Runtime log confirmed the override:

- `joint_1`: damping `30 -> 60`
- `joint_2`: damping `30 -> 60`
- `joint_3`: damping `20 -> 40`
- `joint_4`: damping `10 -> 20`
- `joint_5`: damping `10 -> 20`
- `joint_6`: damping `5 -> 10`
- `ft_sensor_joint`: damping `1 -> 2`
- `position_proportional_gain=1000`

## Result

Rejected as a canonical change, but useful diagnostic evidence.

The run aborted honestly in `MOVING_TO_START` and did not descend:

- outcome observed in launch log: `ABORTED`;
- final reason: `MOVING_TO_START timeout/failure (120.0s)`;
- final timeout: `cart_err=0.013 m`, `xy_err=0.007 m`, `joint_err=0.013 rad`,
  `stable=0/5`;
- insertion depth: `0.0000 m`;
- max raw `|Fz|`: `165.4 N`;
- max raw force norm: `255.5 N`;
- contact-topic samples: `0`.

## Analyzer Evidence

- `above_hole_hold_analysis.md`: strict gate still failed, but improved to
  best estimated state-loop hold `2/5` ticks, minimum XY `0.000153 m`, final
  `MOVING_TO_START` XY `0.006362 m`.
- `moving_to_start_tracking_analysis.md`: p95 max joint error `0.015812 rad`,
  final Cartesian error `0.005915 m`, final XY error `0.005788 m`, worst p95
  joint `joint_6` at `0.014434 rad`.
- `trajectory_tracking_summary.md`: p95 max joint error `0.015841 rad`, mean
  RMS error `0.005625 rad`.
- `wrench_state_summary.md`: `MOVING_TO_START` max raw `|Fz|=165.446 N`, max
  force norm `255.544 N`, minimum XY `0.000153 m`.
- `contact_state_summary.md`: zero bridged contact samples.

## Conclusion

Increasing damping reduces trajectory tracking error and improves the estimated
hold window from one tick in most post-tool runs to two ticks here, but it does
not satisfy the required five 10 Hz stable ticks. The canonical damping remains
unchanged. The next diagnostic should test whether a moderate damping increase
combined with a less oscillatory controller gain or higher update-rate evidence
can produce a sustained hold without force/contact regression.
