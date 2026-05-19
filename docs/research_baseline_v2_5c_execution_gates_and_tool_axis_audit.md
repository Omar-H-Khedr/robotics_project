# Research Baseline v2.5c Execution Gates and Tool-Axis Audit

Research Baseline v2.5c keeps coordinate-based peg-hole insertion in a
diagnostic-only state. It does not move the robot, launch
`task_trajectory_executor`, send `FollowJointTrajectory` goals, or add random
joint targets.

## v2.5b Baseline

v2.5b made the Cartesian insertion geometry valid. The hole center is
`[0.520, -0.200, 0.845]`, the staging pose is intentionally offset at
`[0.640, -0.120, 0.920]`, and the insertion-aligned target stack shares the hole
x/y coordinates:

- `axis_align_pose`: `[0.520, -0.200, 0.920]`
- `insertion_touch_pose`: `[0.520, -0.200, 0.865]`
- `insertion_hold_pose`: `[0.520, -0.200, 0.845]`
- `final_insertion_pose`: `[0.520, -0.200, 0.825]`
- `retreat_pose`: `[0.520, -0.200, 0.920]`

## Unified Execution Gates

v2.5c adds `execution_gate_monitor`, which publishes JSON on
`/execution_gate_status`. The monitor consumes `/cartesian_insertion_diagnostics`,
`/ik_feasibility_diagnostics`, `/safety_status`, and optionally
`/insertion_metrics`.

The Cartesian diagnostics remain the source of truth for
`geometry_valid` through `cartesian_geometry_valid`. IK diagnostics also
recomputes the same geometry rule so its own `execution_gates.geometry_valid`
does not drift from the Cartesian layer.

Controller execution is allowed only when all gates are true:

- Cartesian geometry is valid.
- An IK solver is available.
- Real IK solutions exist for all targets.
- The tool insertion axis has been manually validated.
- The safety guard is active from an observed OK `/safety_status`.
- The force/contact guard is active.

Because real IK solutions, manual tool-axis validation, and force/contact guard
activation are not available in v2.5c, `controller_execution_allowed` remains
`false`.

## Tool-Axis Audit

v2.5c adds `tool_axis_audit`, which publishes JSON on `/tool_axis_audit`. It
reads TF for `world -> tool0`, `world -> base_link`, `world -> hole_center`, and
`world -> axis_align_pose`, then compares the six local tool axes against the
world insertion axis `[0.0, 0.0, -1.0]`.

The audit reports:

- World directions for tool0 `+X`, `-X`, `+Y`, `-Y`, `+Z`, and `-Z`.
- Dot product and angular error for each candidate.
- The best candidate axis.
- `orientation_validated=false`.

The audit never auto-approves orientation. Even if an axis is closely aligned,
manual validation is required before motion can be considered.

## Execution Block

The v2.5c diagnostic launch is:

```bash
ros2 launch thesis_bringup run_full_cartesian_insertion_diagnostics.launch.py
```

This launch starts the object-frame publisher, Cartesian diagnostics, IK
diagnostics, safety monitor, contact metrics, tool-axis audit, and execution gate
monitor. It intentionally does not start any controller execution node.
