# Research Baseline v2.12: Tool-Link Validation

v2.12 validates `tool0` as the diagnostic tool/planning link candidate for
future MoveIt IK diagnostics. This is a readiness check only. It does not
launch `move_group`, call `/compute_ik`, send trajectory goals, fake IK
solutions, start `task_trajectory_executor`, or approve controller motion.

The new `tool_link_validator` node publishes JSON on `/tool_link_validation`.
It checks the candidate link against:

- TF availability for `world -> tool0`, `base_link -> tool0`, and
  `world -> base_link`.
- `robot_description` URDF link names, requiring `tool0` to exist before the
  candidate can be considered valid for diagnostics.
- the project-local `lbr_iisy6_r1300.srdf` semantic candidate, requiring the
  `arm` group and `joint_1` through `joint_6`.
- optional `/tool_axis_audit` and `/cartesian_orientation_targets` observations
  for the configured `selected_tool_axis_candidate="tool0_+Z"` and expected
  insertion axis `[0, 0, -1]`.

The validator reports
`tool_link_validation_status="tool_link_candidate_valid_but_not_motion_approved"`
only when the URDF, TF, and SRDF checks are sufficient for diagnostic launch
preparation. Even then, it keeps:

- `approved_for_motion=false`
- `controller_motion_allowed=false`
- `trajectory_execution_allowed=false`

`semantic_model_validator` now records whether `/tool_link_validation` has
validated the candidate for diagnostics through
`tool_link_candidate_validated_for_diagnostics`. This does not change motion
approval.

v2.12b propagates the latest valid `/tool_link_validation` result into
`moveit_launch_readiness_audit`. The readiness audit now reports the candidate
link, URDF and TF availability, selected tool-axis observation, orientation
target observation, validation status, and
`tool_link_approved_for_motion=false`.

When the semantic candidate is structurally valid and the tool link candidate is
valid for diagnostics, the recommended next step becomes
`prepare_move_group_diagnostic_launch_inputs`. Motion remains blocked: the audit
still reports `moveit_launch_ready=false`,
`compute_ik_expected_after_launch=false`, `controller_motion_allowed=false`, and
`trajectory_execution_allowed=false`.
