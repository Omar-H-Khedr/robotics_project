# Research Baseline Move-To-Start Hold Correction

Date: 2026-06-02

Status: rejected experiment; code not retained.

## Purpose

Test whether a small number of bounded final hold commands to the already-computed
`MOVING_TO_START` joint target can convert near-target oscillation into a stable
above-hole pose without weakening the strict no-contact descent gate.

## Command

```bash
cd /home/omar/code/robotics_project/ros2_ws
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_move_to_start_hold_correction
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_move_to_start_hold_correction
```

## Result

The trial remained a bounded safety failure:

- `Outcome: ABORTED`
- `Reason: MOVING_TO_START timeout/failure (90.0s)`
- final logged `cart_err=0.016 m`
- final logged `xy_err=0.011 m`
- final logged `joint_err=0.014 rad`
- `stable=0/5`
- `Depth: 0.0000 m`
- `Max Fz: 170.8 N`

The correction did briefly reduce XY error:

- first hold command: `xy_err=0.0247 m`
- second hold command: `xy_err=0.0028 m`
- third hold command: `xy_err=0.0008 m`

However, the pose did not remain stable for the required consecutive samples.
Later `MOVING_TO_START` logs drifted back to `xy_err=0.008-0.014 m`, and the
phase timed out without satisfying the strict 2 mm no-contact gate.

## Tracking Evidence

`trajectory_tracking_summary.md` reported:

- observed trajectory commands: `5`
- direct JTC state samples: `0`
- command-vs-joint-state samples: `18495`
- max absolute position error: `0.054733 rad`
- mean max absolute position error: `0.016210 rad`
- p95 max absolute position error: `0.026865 rad`
- final max absolute position error: `0.030123 rad`

## Decision

The experiment is rejected and the code was removed. Re-publishing the same
final joint target can create transient good XY samples, but it did not produce
a stable above-hole pose and should not be used to justify descent. The strict
above-hole stability gate remains unchanged.

The next credible blocker remains runtime tracking/physics/force behavior near
the above-hole target, not a relaxation of safety criteria.
