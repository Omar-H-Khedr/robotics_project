# Research Baseline v2.10: LBR iisy 6 R1300 Semantic Candidate

v2.10 adds a project-local MoveIt semantic configuration candidate for the
KUKA LBR iisy 6 R1300 at:

`ros2_ws/src/kuka_task_control/config/moveit_lbr_iisy6_r1300/`

The overlay contains:

- `lbr_iisy6_r1300.srdf`
- `kinematics.yaml`
- `ompl_planning.yaml`
- `moveit_config_metadata.yaml`

The SRDF is derived from the same-family
`lbr_iisy11_r1300_arm.srdf.xacro` template because that model shares the R1300
reach family. It is intentionally marked as
`candidate_requires_validation`. The planning group is `arm` and references
only `joint_1` through `joint_6`. End-effector/tool semantics are not defined
because the exact LBR iisy 6 R1300 tool link has not been confirmed from an
exact URDF in this project.

The copied adjacent-link collision disables come from the same-family iisy11
R1300 template and are marked as
`collision_matrix_source=same_family_template_requires_validation`.

This is diagnostic preparation only. It is not approved for motion execution,
does not launch `move_group`, does not call IK, does not fake IK solutions,
does not send `FollowJointTrajectory` goals, and does not unblock controller
motion. It is only a prerequisite for future no-motion `/compute_ik`
diagnostics after a verified diagnostic MoveIt launch path exists.

v2.10b makes semantic validation machine-readable. The
`semantic_model_validator` node publishes JSON on `/semantic_model_validation`
with explicit `true` or `false` values for SRDF file existence, XML parse
success, `arm` group discovery, required joint coverage, `/joint_states`
availability, and `/joint_states` to SRDF joint matching. It also publishes the
selected SRDF path, the extracted `arm_group_joints`, the fixed
`required_joints` list, and any `missing_required_joints`.

Even if the SRDF candidate is structurally valid and matches observed joint
states, it remains not approved for motion. v2.10b continues to publish
`approved_for_motion=false`, `controller_motion_allowed=false`, and
`trajectory_execution_allowed=false`. A complete candidate can only advance the
diagnostic readiness guidance toward creating a no-motion `move_group`
diagnostic launch; it does not launch `move_group`, call IK, fake IK solutions,
send `FollowJointTrajectory` goals, or unblock controller motion.
