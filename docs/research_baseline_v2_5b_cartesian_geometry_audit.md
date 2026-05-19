# Research Baseline v2.5b Cartesian Geometry Audit

Research Baseline v2.5b corrects the peg-hole Cartesian target structure before
any controller execution. It is diagnostic-only and does not move the robot,
launch `task_trajectory_executor`, send `FollowJointTrajectory` goals, or add
joint-space insertion guesses.

## Geometry Correction

The old `pre_insertion_pose` at `[0.640, -0.120, 0.920]` was laterally offset
from the measured hole center at `[0.520, -0.200, 0.845]`. It was therefore not
an insertion-aligned pose. v2.5b keeps that Cartesian point only as
`staging_pose`, a safe waypoint used before moving onto the insertion axis.

The corrected insertion-aligned target stack is centered on the hole x/y
coordinates:

- `axis_align_pose`: `[0.520, -0.200, 0.920]`
- `insertion_touch_pose`: `[0.520, -0.200, 0.865]`
- `insertion_hold_pose`: `[0.520, -0.200, 0.845]`
- `final_insertion_pose`: `[0.520, -0.200, 0.825]`
- `retreat_pose`: `[0.520, -0.200, 0.920]`

The insertion axis is world negative Z: `[0.0, 0.0, -1.0]`.

## Orientation Gate

The target config now declares:

- `orientation_mode: align_tool_axis_to_insertion_axis`
- `insertion_axis_world: [0.0, 0.0, -1.0]`
- `tool_insertion_axis: unknown`

Because the actual tool insertion axis has not been validated,
`motion_execution_allowed` is forced to `false`. Any identity orientation in TF
diagnostics is only a frame-visualization placeholder, not an executable
end-effector orientation.

## Diagnostic Outputs

`peg_hole_frame_publisher` publishes static TF frames for:

- `hole_center`
- `staging_pose`
- `axis_align_pose`
- `insertion_touch_pose`
- `insertion_hold_pose`
- `final_insertion_pose`
- `retreat_pose`
- `insertion_axis_marker`

`cartesian_insertion_diagnostics` reports the lateral offsets, vertical
clearance, and path segments:

- `current_tool_to_staging`
- `staging_to_axis_align`
- `axis_align_to_touch`
- `touch_to_hold`
- `hold_to_final`
- `final_to_retreat`

`cartesian_geometry_valid` can become true only when the insertion-aligned
targets match the hole center x/y within 2 mm, the z-order is strictly
`axis_align > touch > hold > final`, and the lateral staging point is explicitly
labeled as staging.

## Execution Block

Control remains blocked until all gates pass:

- Cartesian geometry is valid.
- A real IK path is available.
- The tool insertion axis orientation is validated.
- The safety guard is active.

`ik_feasibility_diagnostics` is still diagnostic-only. It can report approximate
workspace feasibility and visible IK services, but it does not call an IK solver,
does not claim a real executable plan, and does not command robot motion.
