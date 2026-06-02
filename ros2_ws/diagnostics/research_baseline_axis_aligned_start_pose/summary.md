# Research Baseline Axis-Aligned Start Pose Validation

Date: 2026-06-02

## Purpose

Address the contact evidence from
`research_baseline_contact_bridge_full_paths`, where the peg contacted the
target plate during `MOVING_TO_START` before any descent was allowed.

Offline FK showed the previous position-only IK solved the above-hole peg-tip
position while leaving the peg body highly tilted:

- target peg-tip pose: `[0.520, -0.200, 0.885]`;
- peg local +Z axis: `[-0.3549, 0.8264, -0.4371]`;
- angle from world +Z: about `115.9 deg`;
- estimated peg top: `[0.4810, -0.1091, 0.8369]`.

This made the no-contact start pose physically unsafe even when the peg-tip
position appeared above the hole.

## Changes

- Added `RobotKinematics.inverse_position_axis(...)`, a dependency-free
  finite-difference damped least-squares IK method that constrains peg-tip
  position and the peg local +Z axis.
- Added iisy6 joint-limit clipping to that constrained IK.
- Updated `AdmittanceInsertionNode._solve_ik(...)` to request peg local +Z
  aligned with world +Z for Cartesian task targets.

Offline checks confirmed in-limit vertical solutions for:

- above-hole target `[0.520, -0.200, 0.885]`;
- touch target `[0.520, -0.200, 0.830]`;
- final insertion target `[0.520, -0.200, 0.790]`.

## Validation

Syntax:

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/robot_kinematics.py src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
```

Targeted build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control
```

Runtime:

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_axis_aligned_start_pose
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_axis_aligned_start_pose
```

## Runtime Result

The task no longer hard-aborted on raw force during `MOVING_TO_START`. It timed
out at the strict no-contact stability gate:

- outcome: `ABORTED`;
- reason: `MOVING_TO_START timeout/failure (90.0s)`;
- final phase Cartesian error: `0.011498 m`;
- final phase XY error from log: `0.006 m`;
- final joint error: `0.026354 rad`;
- insertion depth: `0.0000 m`;
- max `|Fz|`: `554.24 N`;
- max force norm: `628.61 N`.

The vertical target required a much larger joint-space move than the previous
position-only target:

- trajectory duration: `40.0 s`;
- waypoints: `20`;
- max joint-space distance: `2.4145 rad`.

Contact observer notes:

- no peg-source contact rows were recorded;
- no hole-source contact rows were recorded;
- target-source rows were continuously positive in `MOVING_TO_START`, but this
  target sensor also sees target-plate support/fixture contacts and is not
  by itself evidence of peg contact;
- previous peg-target contact evidence came from both `peg` and `target`
  sources, whereas this run had target-only contact rows.

Tracking evidence:

- observed trajectory commands: `2`;
- max absolute joint-position error: `0.074927 rad`;
- p95 max absolute joint-position error: `0.062845 rad`;
- final max absolute joint-position error: `0.012819 rad`.

## Conclusion

Axis-aligned IK is a retained safety improvement: it removes the position-only
tilted-peg start pose and prevented the previous raw hard-force abort in this
validation. This is still not insertion success. The controller now fails
honestly because the larger vertical-pose move does not satisfy the strict
2 mm no-contact XY stability gate before the 90 s `MOVING_TO_START` timeout.

The next blocker is trajectory timing/settling for the axis-aligned start pose,
not contact-gate loosening. The next milestone should either make the
axis-aligned move converge within the existing timeout or split the move into a
clear staging posture followed by a shorter vertical above-hole alignment move.
