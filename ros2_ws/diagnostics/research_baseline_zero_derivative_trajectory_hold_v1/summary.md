# Zero-Derivative Trajectory Hold Diagnostic

Milestone: `research_baseline_zero_derivative_trajectory_hold_v1`

Status: rejected diagnostic. The source change was removed and is not part of
the canonical controller.

## Command

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_zero_derivative_trajectory_hold_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  tracking_log_dir:=diagnostics/research_baseline_zero_derivative_trajectory_hold_v1
```

The tested source change populated every published `JointTrajectoryPoint` with
zero joint velocities and accelerations, attempting to make the spline
controller stop cleanly at the above-hole target.

## Result

The trial aborted safely in `MOVING_TO_START`; it did not descend, enter
contact search, or claim insertion.

- final outcome: `ABORTED`
- final reason: `MOVING_TO_START timeout/failure (120.0s). cart_err=0.014m, xy_err=0.014m, joint_err=0.020rad, stable=0/5, tolerance=0.050m`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `176.57 N`
- max raw force norm: `272.23 N`
- contact-topic samples: `0`
- planned `link_5` target-plate intersections: `0/201`
- runtime-feedback `link_5` target-plate intersections: `0/2647`
- closest runtime `link_5` target-plate AABB clearance: `0.110797 m`

Corrected peg-tip replay from the trajectory tracking CSV showed:

- minimum replayed XY error: `0.000014 m` at `stamp_s=42.300`
- strict XY samples: `154/26463`
- best consecutive strict observer samples: `4`

## Conclusion

The zero-derivative command reached the strict XY gate transiently but still did
not satisfy the five-tick state-machine stability requirement. It also
regressed the final timeout error to about 14 mm XY, similar to the earlier
default post-tool run. The change was removed. The retained evidence suggests
that the remaining issue is not a missing zero-velocity field alone; the next
work should instrument or control the endpoint hold window more directly.
