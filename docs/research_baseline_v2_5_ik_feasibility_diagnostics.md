# Research Baseline v2.5: IK Feasibility Diagnostics

Research Baseline v2.5 adds a diagnostic-only IK feasibility layer before any
robot movement. It evaluates the coordinate-based peg/hole insertion targets
published in v2.4 and reports whether they are inside a conservative geometric
workspace envelope.

The new `ik_feasibility_diagnostics` node reads TF for:

- `world -> base_link`
- `world -> tool0`
- `world -> hole_center`
- `world -> pre_insertion_pose`
- `world -> insertion_touch_pose`
- `world -> insertion_hold_pose`
- `world -> final_insertion_pose`

It also subscribes to `/joint_states`, reports current joint names and
positions, and loads available joint metadata from the KUKA joint-limit config
when that package is installed. If position limits are unavailable, it reports
conservative fallback limits instead of inventing target joint values.

## Diagnostic Scope

The node publishes JSON on `/ik_feasibility_diagnostics` with:

- current tool pose in `world` and `base_link`
- each target pose in `world` and `base_link`
- tool-to-target translational distance
- z offset from `hole_center`
- approximate radial workspace feasibility from `base_link`
- MoveIt or IK service detection through service introspection
- `ik_solution_available=null` unless a real IK solver is explicitly called
- `motion_execution_enabled=false`

This is not a motion node. It does not launch `task_trajectory_executor`, does
not call a trajectory action server, and does not send `FollowJointTrajectory`
goals.

## Feasibility Semantics

The v2.5 diagnostics intentionally distinguish geometric reachability from true
IK feasibility:

- `geometric_feasible_no_ik_solver`: target is inside the conservative radial
  workspace envelope, but no IK solution has been computed.
- `geometric_infeasible`: target is outside the configured workspace envelope or
  the target frame is unavailable.
- `ik_solver_not_available`: reserved for a future configuration that requires a
  solver endpoint and cannot find one.
- `ik_solution_available` and `ik_solution_unavailable`: reserved for a future
  version that actually calls an IK solver.

No target is reported as having a true IK solution in v2.5.

## Launch

The full Cartesian diagnostics launch now starts:

- `peg_hole_frame_publisher`
- `cartesian_insertion_diagnostics`
- `ik_feasibility_diagnostics`
- `safety_monitor`
- `contact_metrics_node`

It still does not execute a robot trajectory.

An optional package-local launch is also available:

```bash
ros2 launch kuka_task_control run_ik_feasibility_diagnostics.launch.py
```

## Next Step

Research Baseline v2.6 should integrate a real IK solver or MoveIt service call
when available. Only then should `ik_solution_available` become true or false
based on an actual solver result.
