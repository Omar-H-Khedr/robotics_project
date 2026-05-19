# Research Baseline v2.5d: Orientation Target Calculation

## Purpose

Research Baseline v2.5c confirmed that the Cartesian peg/hole target geometry is
valid and that the execution gates keep controller execution blocked. The
tool-axis audit identified `tool0_+Z` as the closest candidate insertion axis,
with an alignment angle of roughly 21.25 degrees against the configured world
insertion axis `[0.0, 0.0, -1.0]`.

That is not accurate enough for peg-in-hole insertion. v2.5d therefore adds a
diagnostic-only orientation target calculation step.

## Added Diagnostic

The new `cartesian_orientation_target_calculator` node publishes JSON on
`/cartesian_orientation_targets`. It reads the current `world -> tool0` TF
orientation, the configured orientation-planning block in
`kuka_task_control/config/peg_hole_cartesian_targets.yaml`, and the Cartesian
target frames:

- `axis_align_pose`
- `insertion_touch_pose`
- `insertion_hold_pose`
- `final_insertion_pose`
- `retreat_pose`

For each target, it computes a desired world-frame orientation quaternion that
aligns the selected `tool0_+Z` axis with the world insertion direction
`[0.0, 0.0, -1.0]`.

## Yaw Reference

Aligning one tool axis to the insertion direction leaves yaw about the insertion
axis underdetermined. v2.5d uses the current tool yaw as the reference when the
current tool X or Y axis can be projected onto the plane perpendicular to the
insertion axis. If that projection cannot be resolved, the diagnostic reports
`yaw_reference_unresolved=true`.

## Safety and Validation Status

This is diagnostic only:

- No robot motion is commanded.
- `task_trajectory_executor` is not launched.
- No `FollowJointTrajectory` goals are sent.
- No random joint values are introduced.
- MoveIt is not required.
- `orientation_validated` remains `false`.
- `motion_execution_allowed` remains `false`.

The computed quaternions are orientation targets, not validated motion plans.
Orientation remains unvalidated until an IK solver and a dry-run joint plan
confirm that the robot can reach each pose safely without executing controller
motion.
