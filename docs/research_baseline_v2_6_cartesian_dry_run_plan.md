# Research Baseline v2.6: Cartesian Dry-Run Plan

## Purpose

Research Baseline v2.6 assembles the complete Cartesian peg-hole insertion
waypoint plan as diagnostics only. It combines the validated Cartesian geometry
from v2.5f with computed orientation targets and IK feasibility diagnostics, but
does not command the robot.

The dry-run waypoint order is:

- `current_tool_pose`
- `staging_pose`
- `axis_align_pose`
- `insertion_touch_pose`
- `insertion_hold_pose`
- `final_insertion_pose`
- `retreat_pose`

## Published Plan

`cartesian_insertion_dry_run_planner` publishes JSON on
`/cartesian_insertion_dry_run_plan` with:

- `status="cartesian_dry_run_no_motion"`
- `motion_execution_enabled=false`
- `trajectory_execution_requested=false`
- `controller_execution_allowed=false`
- full per-waypoint pose, source, distance, workspace, orientation-target, IK,
  and executability diagnostics
- global full-pose, geometry, IK-solution, and executability flags
- explicit `block_reasons` and `primary_block_reason`

The node subscribes to `/cartesian_insertion_diagnostics`,
`/cartesian_orientation_targets`, `/ik_feasibility_diagnostics`,
`/execution_gate_status`, and `/joint_states` when available.

## Execution Contract

v2.6 is diagnostic-only. It does not launch `task_trajectory_executor`, does not
send `FollowJointTrajectory` goals, does not add random joint targets, and does
not fake IK solutions.

The plan remains non-executable until real IK solutions exist for every planned
Cartesian waypoint and all execution gates are available. In the current
validated configuration the expected state is:

- `all_waypoints_have_full_pose=true`
- `all_waypoints_geometrically_feasible=true`
- `all_waypoints_have_ik_solution=false`
- `plan_executable=false`
- `primary_block_reason="IK solver not available"`

No controller command is sent in v2.6.
