# Research Baseline v2.9/v2.9b: MoveIt IK Diagnostic Launch Preparation

## Purpose

Research Baseline v2.9 prepares a diagnostic-only MoveIt/move_group readiness
path so `/compute_ik` can be enabled later without moving the robot.

The new `moveit_launch_readiness_audit` node publishes JSON on
`/moveit_launch_readiness_audit`. It checks whether the available KUKA LBR iisy
MoveIt resources are safe to use for a future diagnostic `move_group` launch.
It does not launch `move_group`, does not call IK, does not create joint
solutions, and does not command any controller.

v2.9b adds a package-consistency gate. The readiness audit evaluates each
MoveIt config package independently and will not combine an SRDF from one
package with `kinematics.yaml`, `ompl_planning.yaml`, joint-limits, or launch
files from another package.

## Published Report

The readiness audit reports:

- `moveit_launch_ready`
- `compute_ik_expected_after_launch`
- `exact_robot_semantic_match`
- `same_family_srdf_available`
- `selected_moveit_config_package`
- `selected_srdf`
- `available_srdf_variants`
- `kinematics_yaml_found` and `kinematics_yaml_file`
- `ompl_planning_yaml_found` and `ompl_planning_yaml_file`
- `joint_limits_yaml_found` and `joint_limits_yaml_file`
- `robot_description_available`
- `robot_description_source_node`
- `robot_description_check_reason`
- `robot_joint_names_from_joint_states`
- `robot_joint_names_from_urdf`
- `robot_description_semantic_available`
- `move_group_launch_found` and `move_group_launch_files`
- `config_package_candidates`
- `controller_motion_allowed=false`
- `trajectory_execution_allowed=false`
- `recommended_next_step`

## Decision Policy

The diagnostic target is the exact `lbr_iisy6_r1300` semantic model. If no exact
LBR iisy 6 R1300 SRDF exists, the audit reports:

- `exact_robot_semantic_match=false`
- `selected_srdf=null`
- `moveit_launch_ready=false`
- `recommended_next_step="create_or_select_matching_srdf_for_lbr_iisy6_r1300"`

If same-family iisy SRDFs are available but the exact iisy6 R1300 SRDF is not,
the audit reports:

- `same_family_srdf_available=true`
- `exact_robot_semantic_match=false`
- `selected_srdf=null`
- `moveit_launch_ready=false`
- `recommended_next_step="create_lbr_iisy6_r1300_srdf_from_same_family_template"`

The current system has same-family SRDFs in
`kuka_lbr_iisy_moveit_config` for iisy3, iisy11, and iisy15, but no exact
`lbr_iisy6_r1300` SRDF. The audit therefore leaves `move_group` blocked even
though same-package iisy `kinematics.yaml` and `ompl_planning.yaml` files exist.

If an exact semantic model exists but no launch file that starts `move_group`
exists in the same selected package, the next step is
`create_move_group_diagnostic_launch`.

If a safe launch path exists but `/compute_ik` is not running, the next step is
`launch_move_group_diagnostic_only`.

If `/compute_ik` is visible, the next step is
`test_compute_ik_service_no_motion`.

## Launch Contract

`thesis_bringup/launch/run_move_group_ik_diagnostic.launch.py` currently starts
only:

- `moveit_launch_readiness_audit`
- `moveit_config_audit`
- `ik_backend_audit`

`move_group` remains blocked unless semantic model compatibility is confirmed
for the exact robot model and the launch inputs are safe. Controller execution
also remains blocked. No `task_trajectory_executor`, `FollowJointTrajectory`
goal, Gazebo process, or motion command is started by v2.9.
