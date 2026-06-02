# Start Gain 2000 After Tool-Tip Fix

Milestone: `research_baseline_start_gain_2000_after_tool_fix_v1`

Status: rejected diagnostic. The canonical `position_gain` remains unchanged.

## Command

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  position_gain:=2000 \
  tracking_log_dir:=diagnostics/research_baseline_start_gain_2000_after_tool_fix_v1
```

## Result

The trial aborted safely in `MOVING_TO_START`; it did not descend, enter
contact search, or claim insertion.

- final outcome: `ABORTED`
- final reason: `MOVING_TO_START timeout/failure (120.0s). cart_err=0.006m, xy_err=0.002m, joint_err=0.009rad, stable=0/5, tolerance=0.050m`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `172.34 N`
- max raw force norm: `272.66 N`
- contact-topic samples: `0`
- planned `link_5` target-plate intersections: `0/201`
- runtime-feedback `link_5` target-plate intersections: `0/2883`
- closest runtime `link_5` target-plate AABB clearance: `0.110278 m`

Corrected peg-tip replay from the trajectory tracking CSV showed:

- minimum replayed XY error: `0.000455 m` at `stamp_s=46.541`
- strict XY samples: `147/28821`
- best consecutive strict observer samples: `3`

## Conclusion

Gain 2000 improved the final timeout error relative to the default-gain
tool-tip correction run, but it still did not satisfy the state-machine
requirement of five consecutive strict 2 mm no-contact samples. It is useful
evidence that endpoint authority matters, but it is not a sufficient canonical
fix.
