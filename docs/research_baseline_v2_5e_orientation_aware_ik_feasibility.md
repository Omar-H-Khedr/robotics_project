# Research Baseline v2.5e: Orientation-Aware IK Feasibility

## Purpose

Research Baseline v2.5d computed desired Cartesian target orientations. Those
targets align the selected `tool0_+Z` axis with the configured world insertion
axis `[0.0, 0.0, -1.0]`, while keeping `orientation_validated=false`.

v2.5e extends the IK feasibility diagnostics to evaluate the complete target
pose shape used by a future IK check:

- Cartesian position from TF or the configured Cartesian target YAML.
- Desired world-frame orientation from `/cartesian_orientation_targets`.
- Per-target tool-axis alignment dot product and angle.

The checked full-pose targets are:

- `axis_align_pose`
- `insertion_touch_pose`
- `insertion_hold_pose`
- `final_insertion_pose`
- `retreat_pose`

## Diagnostic Behavior

`ik_feasibility_diagnostics` subscribes to `/cartesian_orientation_targets` and
publishes per-target fields for the world position, desired world orientation,
orientation source, alignment score, orientation target availability, and
full-pose feasibility status.

When no IK solver is available, each complete target pose is reported as
`full_pose_ready_but_no_ik_solver`. In that case `ik_solver_available=false`,
`ik_solution_available=null`, and no IK result is claimed.

## Execution Gate

`execution_gate_monitor` preserves `geometry_valid` from
`/cartesian_insertion_diagnostics` and `orientation_targets_available` from
`/cartesian_orientation_targets`. It also reports whether orientation-aware IK
diagnostics were checked and whether full pose targets are available.

Controller execution remains blocked. v2.5e is diagnostic only:

- No robot motion is commanded.
- `task_trajectory_executor` is not launched.
- No `FollowJointTrajectory` goals are sent.
- No random joint targets are introduced.
- No fake IK solutions are published.
- `orientation_validated` remains `false` unless explicitly validated by a
  later real IK and safety workflow.

An executable plan must not be reported unless a real IK solver exists, every
target has a real IK solution, orientation is explicitly validated, and the
safety and force gates are active.
