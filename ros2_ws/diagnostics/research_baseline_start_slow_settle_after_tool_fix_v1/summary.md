# Slow Start Settle After Tool-Tip Fix

Milestone: `research_baseline_start_slow_settle_after_tool_fix_v1`

Status: rejected diagnostic. The source change was removed and is not part of
the canonical controller.

## Command

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  tracking_log_dir:=diagnostics/research_baseline_start_slow_settle_after_tool_fix_v1
```

The tested controller experiment published one additional 20 s
`MOVING_TO_START` settle trajectory from current joint feedback to the same
axis-aligned start target after the initial command had nearly converged.

## Result

The trial aborted safely in `MOVING_TO_START`; it did not descend, did not enter
contact search, and did not claim insertion.

- final outcome: `ABORTED`
- final reason: `MOVING_TO_START timeout/failure (120.0s). cart_err=0.012m, xy_err=0.011m, joint_err=0.025rad, stable=0/5, tolerance=0.050m`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `171.25 N`
- max raw force norm: `271.34 N`
- contact-topic samples: `0`
- planned `link_5` target-plate intersections: `0/201`
- runtime-feedback `link_5` target-plate intersections: `0/1884`
- closest runtime `link_5` target-plate AABB clearance: `0.111635 m`

Offline tracking replay with the corrected peg-tip kinematics showed the
strict 2 mm XY gate was crossed only transiently:

- minimum replayed XY error: `0.000072 m` at `stamp_s=57.664`
- strict XY samples: `37/18836`
- best consecutive strict observer samples: `2`
- final replay sample was during abort/retreat, not a valid hold sample.

## Conclusion

The slow-settle command was rejected because it did not create a stable
above-hole hold at the state-machine cadence and it slightly worsened the final
timeout error compared with the prior tool-tip correction run. The retained
evidence still matters: after the tool-tip correction there were no contact
rows and no `link_5` clearance intersections, so the next blocker is
controller/physics stabilization of the above-hole pose rather than collision
geometry or safety-gate relaxation.
