# Research Baseline v2.13: MoveIt Diagnostic Inputs

v2.13 prepares a complete diagnostic-only MoveIt input bundle for a future
`move_group` `/compute_ik` test. It does not launch `move_group`, call
`/compute_ik`, send `FollowJointTrajectory` goals, start
`task_trajectory_executor`, enable controller motion, or approve the robot for
motion.

The new `moveit_diagnostic_input_builder` node publishes JSON on
`/moveit_diagnostic_inputs`. It reports:

- `robot_description` availability, source node, length, required joint names,
  and whether `tool0` exists in the URDF.
- `robot_description_semantic` readiness from the project-local
  `lbr_iisy6_r1300.srdf`, including parse status, semantic text length, and the
  `arm` group joints `joint_1` through `joint_6`.
- MoveIt config inputs: `kinematics.yaml`, `ompl_planning.yaml`, and
  `joint_limits.yaml` discovery. If `joint_limits.yaml` is missing, the report
  names the required fallback source instead of treating the config as motion
  ready.
- planning frames and links: `planning_frame="base_link"`, `tool_link="tool0"`,
  `selected_tool_axis_candidate="tool0_+Z"`, and the latest
  `/tool_link_validation` status.
- safety gates that remain false:
  `trajectory_execution_allowed`, `controller_motion_allowed`,
  `move_group_launch_allowed`, `compute_ik_test_allowed`, and
  `approved_for_motion`.

`moveit_diagnostic_inputs_ready=true` only when `robot_description` is
available, the SRDF exists and parses, the `arm` group is valid, the kinematics
file contains an `arm` config, and the tool-link candidate is valid for
diagnostics. This readiness flag means only that the future no-motion
`move_group` diagnostic launch inputs are prepared.

`moveit_launch_readiness_audit` now subscribes to `/moveit_diagnostic_inputs`
and exposes `moveit_diagnostic_inputs_available`,
`moveit_diagnostic_inputs_ready`, `move_group_launch_inputs_ready`, and
`move_group_launch_allowed=false`. When the inputs are ready, the recommended
next step is:

```text
create_move_group_diagnostic_launch_with_trajectory_execution_disabled
```

The next baseline step is to create that diagnostic-only `move_group` launch
with trajectory execution disabled. Only after that launch exists should a
separate no-motion `/compute_ik` service test be considered.
