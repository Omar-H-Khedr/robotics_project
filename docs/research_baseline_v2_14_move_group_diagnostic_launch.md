# Research Baseline v2.14: Move Group Diagnostic Launch

v2.14 prepares a diagnostic-only MoveIt `move_group` launch path for observing
whether `/compute_ik` can become available. It is not a motion milestone and it
does not approve execution.

The default launch:

```bash
ros2 launch thesis_bringup run_move_group_ik_diagnostic.launch.py
```

keeps `launch_move_group=false`, so it starts diagnostics only and does not
start `move_group`.

The optional diagnostic launch path is:

```bash
ros2 launch thesis_bringup run_move_group_ik_diagnostic.launch.py launch_move_group:=true
```

That path must keep `allow_trajectory_execution=false`,
`trajectory_execution_allowed=false`, and `controller_motion_allowed=false`.
It must not launch `task_trajectory_executor`, must not send
`FollowJointTrajectory` goals, must not execute a MoveIt plan, and must not call
`/compute_ik` by itself.

`move_group_diagnostic_config_builder` publishes
`/move_group_diagnostic_config` with the required diagnostic inputs:
`robot_description`, `robot_description_semantic`, the project-local
`kinematics.yaml`, the project-local `ompl_planning.yaml`,
`planning_group="arm"`, `planning_frame="base_link"`, and `tool_link="tool0"`.
It reports `move_group_launch_allowed=false`, `approved_for_motion=false`, and
`diagnostic_only=true`.

`move_group_runtime_audit` publishes `/move_group_runtime_audit`. It only
observes whether a `move_group` node and `/compute_ik` service are visible. It
does not call `/compute_ik`, does not fake IK solutions, and does not approve
motion.

The only intended v2.14 outcome is service availability observation:
`/compute_ik` visible or missing. Any later IK request must be a separate
no-motion diagnostic step.

## v2.14b robot_description availability

v2.14b fixes `robot_description` availability in the diagnostic launch path.
`run_move_group_ik_diagnostic.launch.py` now starts `robot_state_publisher`
with the same project robot description xacro used by the full Cartesian
diagnostics path, so diagnostic nodes can retrieve `robot_description` without
launching `move_group`.

`move_group` remains disabled by default with `launch_move_group=false`.
Trajectory execution remains disabled:
`allow_trajectory_execution=false`, `trajectory_execution_allowed=false`, and
`controller_motion_allowed=false`.
