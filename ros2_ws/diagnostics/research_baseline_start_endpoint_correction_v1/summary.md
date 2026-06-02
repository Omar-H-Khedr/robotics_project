# research_baseline_start_endpoint_correction_v1

Date: 2026-06-02

## Purpose

Test a bounded `MOVING_TO_START` endpoint correction after the tool-tip frame
fix and hold analyzers showed repeated transient crossings of the strict 2 mm
above-hole gate. The experiment preserved the existing descent gate and only
allowed small no-contact endpoint corrections above the workpiece.

## Validation Command

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 180s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  tracking_log_dir:=diagnostics/research_baseline_start_endpoint_correction_v1
```

## Result

Rejected and source change removed.

The run stayed bounded and did not descend, but it still failed the preserved
strict above-hole stability gate:

- outcome observed in launch log: `ABORTED`;
- final reason: `MOVING_TO_START timeout/failure (120.0s)`;
- final timeout: `cart_err=0.010 m`, `xy_err=0.004 m`, `joint_err=0.018 rad`,
  `stable=0/5`;
- insertion depth: `0.0000 m`;
- max raw `|Fz|`: `174.1 N`;
- max raw force norm: `270.2 N`;
- contact-topic samples: `0`.

The endpoint correction rejected one large correction (`q_step=0.1300 rad`) and
accepted three small corrections (`q_step=0.0130`, `0.0164`, and `0.0079 rad`).
This improved the final timeout XY error relative to several earlier rejected
post-tool runs, but it did not produce a controlled hold.

## Analyzer Evidence

- `above_hole_hold_analysis.md`: estimated state-loop gate still failed, with
  best stable ticks `1/5`, minimum XY `0.000173 m`, and final
  `MOVING_TO_START` XY `0.003925 m`.
- `moving_to_start_tracking_analysis.md`: the last selected axis-align command
  had p95 max joint error `0.023946 rad`, final Cartesian error `0.010554 m`,
  and final XY error `0.007752 m`.
- `wrench_state_summary.md`: `MOVING_TO_START` max raw `|Fz|=174.122 N`, max
  force norm `270.241 N`, minimum XY `0.000173 m`.
- `contact_state_summary.md`: zero bridged contact samples.

## Conclusion

Small endpoint corrections are safe in this run but are not a credible
canonical fix. The behavior change was removed. The next implementation should
focus on sustained hold control or controller/physics damping at the endpoint,
with the 2 mm no-contact gate and hard-force abort unchanged.
