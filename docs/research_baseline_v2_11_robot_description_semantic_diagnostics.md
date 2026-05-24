# Research Baseline v2.11: robot_description_semantic Diagnostics

v2.11 prepares diagnostics for a future MoveIt `robot_description_semantic`
parameter and `/compute_ik` workflow without launching motion infrastructure.

The new `robot_description_semantic_diagnostics` node reads the project-local
SRDF candidate at:

`ros2_ws/src/kuka_task_control/config/moveit_lbr_iisy6_r1300/lbr_iisy6_r1300.srdf`

When the package is installed, the same relative file is resolved from the
installed `kuka_task_control` share directory. The node publishes JSON on
`/robot_description_semantic_diagnostics` with the SRDF path, file existence,
XML parse status, semantic text availability and length, `arm` group detection,
`arm_group_joints`, required joint coverage, and the conservative semantic
validation status.

The SRDF candidate is structurally valid for diagnostics when it parses, defines
the `arm` group, and references `joint_1` through `joint_6`. This does not make
the model approved for motion. The tool link and end-effector assumptions still
require validation against the exact robot description, so v2.11 keeps:

- `approved_for_motion=false`
- `controller_motion_allowed=false`
- `trajectory_execution_allowed=false`

`moveit_launch_readiness_audit` now reports whether the semantic candidate is
available, where it came from, whether `/robot_description_semantic_diagnostics`
has been observed, and the latest semantic diagnostics status. If the SRDF
candidate is structurally valid while tool-link validation is still required,
the audit keeps `moveit_launch_ready=false` and recommends
`validate_tool_link_and_prepare_move_group_diagnostic_launch`.

v2.11 does not launch `move_group`, does not call `/compute_ik`, does not fake
IK solutions, does not send `FollowJointTrajectory` goals, does not start
`task_trajectory_executor`, and does not unblock controller execution.
