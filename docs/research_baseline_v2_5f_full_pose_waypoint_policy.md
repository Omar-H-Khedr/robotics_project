# Research Baseline v2.5f: Full-Pose Waypoint Policy

## Purpose

Research Baseline v2.5f makes the coordinate-based peg-hole insertion waypoint
policy explicit for every planned Cartesian waypoint. The `staging_pose` is now
a full-pose waypoint, not a position-only diagnostic target.

The planned full-pose waypoint set is:

- `staging_pose`
- `axis_align_pose`
- `insertion_touch_pose`
- `insertion_hold_pose`
- `final_insertion_pose`
- `retreat_pose`

## Orientation Policy

All planned waypoints use `align_tool_axis_to_insertion_axis`. The selected tool
axis candidate remains `tool0_+Z`, and the configured world insertion axis
remains `[0.0, 0.0, -1.0]`.

`staging_pose` is oriented before lateral alignment so the tool does not need to
rotate close to the hole. The yaw reference policy remains
`keep_current_tool_yaw_if_possible`.

## Diagnostic Contract

`cartesian_orientation_target_calculator` publishes orientation targets for all
planned waypoints on `/cartesian_orientation_targets`. When the orientation
calculation succeeds, each waypoint reports:

- `orientation_target_available=true`
- `orientation_source="cartesian_orientation_targets"`
- `expected_alignment_after_orientation.dot=1.0`
- `expected_alignment_after_orientation.angle_deg=0.0`

This does not validate the orientation. `orientation_validated` remains `false`.

## IK and Execution Gates

`ik_feasibility_diagnostics` treats `staging_pose` as a full-pose target when
its orientation target is available. Without a real IK solver, full-pose
waypoints are reported as `full_pose_ready_but_no_ik_solver`.

This baseline remains diagnostic-only:

- No robot motion is commanded.
- `task_trajectory_executor` is not launched.
- No `FollowJointTrajectory` goals are sent.
- No random joint targets are introduced.
- No fake IK solutions are reported.
- `motion_execution_allowed=false`.
- `controller_execution_allowed=false`.

Controller motion remains blocked until real IK solutions exist for all planned
full-pose waypoints, orientation is explicitly validated, and the safety and
force/contact gates are active.
