# SEARCH Tracking Sensitivity Diagnostic

Date: 2026-06-03

Milestone: `research_baseline_search_tracking_sensitivity_v1`

Purpose: determine whether the remaining no-contact SEARCH XY instability is
consistent with measured joint tracking error, or whether it points to a frame,
target, or kinematics mismatch.

## Implementation

Added passive analyzer:

- `thesis_bringup.search_tracking_sensitivity_analyzer`
- console script: `search_tracking_sensitivity_analyzer`

The analyzer reads:

- `trajectory_commands.csv`
- `trajectory_controller_state_samples.csv`

For each command window it computes peg-tip Cartesian reference and feedback
from `RobotKinematics`, evaluates the finite-difference peg-tip Jacobian at
the controller reference, and compares:

- actual XY drift: `feedback_xy - reference_xy`;
- linearized XY drift: `J_xy * (feedback - reference)`;
- linearization residual;
- per-joint XY contribution.

It does not publish ROS commands or alter safety gates.

## Validation Commands

Syntax:

```bash
python3 -m py_compile ros2_ws/src/thesis_bringup/thesis_bringup/search_tracking_sensitivity_analyzer.py
python3 -m py_compile ros2_ws/src/thesis_bringup/setup.py
```

Build:

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select thesis_bringup kuka_task_control
```

Offline evidence generation:

```bash
source install/setup.bash
ros2 run thesis_bringup search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_streak_preservation_v1
ros2 run thesis_bringup search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_post_command_stability_gate_v1_repeat2
ros2 run thesis_bringup search_tracking_sensitivity_analyzer diagnostics/research_baseline_search_feedback_compensated_recenter_v1
```

## Evidence

Generated files:

- `diagnostics/research_baseline_search_streak_preservation_v1/search_tracking_sensitivity_analysis.md`
- `diagnostics/research_baseline_search_streak_preservation_v1/search_tracking_sensitivity_analysis.json`
- `diagnostics/research_baseline_search_post_command_stability_gate_v1_repeat2/search_tracking_sensitivity_analysis.md`
- `diagnostics/research_baseline_search_post_command_stability_gate_v1_repeat2/search_tracking_sensitivity_analysis.json`
- `diagnostics/research_baseline_search_feedback_compensated_recenter_v1/search_tracking_sensitivity_analysis.md`
- `diagnostics/research_baseline_search_feedback_compensated_recenter_v1/search_tracking_sensitivity_analysis.json`

Key comparable results:

| Run | Centered hold commands | Max centered-hold p95 actual XY drift m | Max centered-hold p95 joint error rad | Linearization residual p95 m | Dominant joint |
| --- | ---: | ---: | ---: | ---: | --- |
| `research_baseline_search_streak_preservation_v1` | 7 | 0.003981 | 0.008404 | about 0.000020 | `joint_1` |
| `research_baseline_search_post_command_stability_gate_v1_repeat2` | 6 | 0.003876 | 0.008488 | about 0.000019 | `joint_1` |
| `research_baseline_search_feedback_compensated_recenter_v1` | 1 | 0.003483 | 0.008287 | about 0.000016 | `joint_1` |

The accepted streak-preservation run showed centered hold feedback mean XY
around `0.0022-0.0026 m`, while centered hold targets were effectively at the
hole center. The actual reference-feedback XY drift and the linearized
`J_xy * dq` estimate matched within roughly `0.00002 m`, so the controller-state
tracking error is sufficient to explain the remaining millimeter-scale peg-tip
XY error in these logs.

## Conclusion

This diagnostic does not show a target-frame mismatch in the controller-state
log path. The remaining blocker is controller/physics tracking accuracy around
the no-contact centered SEARCH hold: approximately `0.0084 rad` p95 joint error
maps through the current posture sensitivity to approximately `0.0035-0.0040 m`
p95 peg-tip XY drift. `joint_1` is the largest single p95 XY contributor.

The physical `0.0010 m` clearance gate remains correct and should not be
loosened. The next implementation milestone should reduce or compensate the
actual joint tracking error/dynamics during centered no-contact hold, with
runtime validation still required before INSERT can be re-enabled.
