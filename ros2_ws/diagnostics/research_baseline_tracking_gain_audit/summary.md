# Research Baseline Tracking Gain Audit

Date: 2026-06-02

## Objective

Investigate why `MOVING_TO_START` does not reliably reach the above-hole no-contact gate after `/joint_states` source integrity was fixed.

## Offline IK Evidence

Command:

```bash
source install/setup.bash
python3 - <<'PY'
import numpy as np
from kuka_task_control.robot_kinematics import RobotKinematics
kin = RobotKinematics()
safe = np.array([0.0, -0.8, 1.2, 0.0, 0.8, 0.0])
target = np.array([0.520, -0.200, 0.885])
q, converged, residual = kin.inverse_position(target, safe, max_iter=100)
print(converged, residual, q, kin.pose(q)[0])
PY
```

Result:

- IK converged for `AXIS_ALIGN_POSE`.
- FK residual was approximately `1.26e-09 m`.
- Target joint vector was approximately `[-0.446294, -0.884288, 1.851243, -0.102913, 1.055585, 0.014104]`.
- Maximum joint displacement from `SAFE_HOME` was approximately `0.651 rad`.

The target is reachable in the local kinematic model; failure is runtime tracking/physics/controller behavior, not an unreachable Cartesian target.

## Runtime Evidence

Baseline with canonical gain 1000 and no refinement, after joint-state source fix:

- reached best observed XY error around `0.011 m` at `MOVING_TO_START t=60 s`;
- later drifted/oscillated back to about `0.030-0.049 m`;
- never met the 0.002 m no-contact descent gate.

Gain override test with `position_gain:=250.0`:

- confirmed `gz_ros2_control` accepted `position_proportional_gain=250`;
- remained outside the gate: XY about `0.023 m` at `t=60 s` and `0.072 m` by `t=65 s`;
- was rejected because it did not improve convergence.

Bounded refinement test:

- added short repeated joint-space refinement steps after the initial trajectory;
- was rejected and removed because it drove joint error up to about `1.15 rad` and left XY error around `0.14-0.23 m`;
- did not descend or relax any gate.

## Retained Changes

- `spawn_robot_sdf.py` now accepts `--position-gain` and injects the requested `gz_ros2_control` `position_proportional_gain`.
- `research_baseline.launch.py` exposes `position_gain` with canonical default `1000.0`.
- `MOVING_TO_START` logging now includes `joint_err` so future diagnostics distinguish joint tracking from Cartesian model error.

## Status

No physical insertion success is claimed. Above-hole tracking remains unresolved. The next credible milestone is controller/physics stabilization with measured joint and Cartesian tracking, not safety-gate relaxation.
