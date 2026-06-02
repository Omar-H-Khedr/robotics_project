# Start Gain 3000 After Tool-Tip Fix

Milestone: `research_baseline_start_gain_3000_after_tool_fix_v1`

Status: rejected diagnostic. The canonical `position_gain` remains unchanged.

## Command

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_start_gain_3000_after_tool_fix_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  position_gain:=3000 \
  tracking_log_dir:=diagnostics/research_baseline_start_gain_3000_after_tool_fix_v1
```

## Result

The trial aborted safely in `MOVING_TO_START`; it did not descend, enter
contact search, or claim insertion.

- final outcome: `ABORTED`
- final reason: `MOVING_TO_START timeout/failure (120.0s). cart_err=0.008m, xy_err=0.007m, joint_err=0.015rad, stable=0/5, tolerance=0.050m`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `174.24 N`
- max raw force norm: `273.60 N`
- contact-topic samples: `0`
- planned `link_5` target-plate intersections: `0/201`
- runtime-feedback `link_5` target-plate intersections: `0/1334`
- closest runtime `link_5` target-plate AABB clearance: `0.115674 m`

Corrected peg-tip replay from the trajectory tracking CSV showed:

- minimum replayed XY error: `0.000372 m` at `stamp_s=64.964`
- strict XY samples: `10/13332`
- best consecutive strict observer samples: `2`

## Conclusion

Gain 3000 did not improve the above-hole hold relative to gain 2000 and ended
with a larger timeout XY error. It remains rejected. The next implementation
work should focus on explicit endpoint hold behavior, trajectory timing, or
controller/physics damping from evidence rather than further global gain
increase.
