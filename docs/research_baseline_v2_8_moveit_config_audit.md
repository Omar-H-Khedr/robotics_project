# Research Baseline v2.8 / v2.8b: MoveIt Config Audit

## Purpose

Research Baseline v2.8 audits whether the KUKA LBR iisy model has enough
MoveIt configuration available to prepare a safe path toward `/compute_ik`.

The current validated status is that MoveIt-related packages are visible, but
`/compute_ik` is not running. A MoveIt config package with SRDF and
`kinematics.yaml` resources must be confirmed before IK service calls can be
trusted. Even then, the first use remains diagnostic-only and must not execute
trajectories.

v2.8b fixes inconsistent MoveIt config audit fields. If
`moveit_config_package_found=true`, the audit must also publish the selected
package name, package path, and explicit file paths. If those paths cannot be
identified, the audit must report either `moveit_config_package_found=false` or
`partial_moveit_config_found=true`.

## Published Report

`moveit_config_audit` publishes JSON on `/moveit_config_audit` with:

- `status="moveit_config_audit_diagnostic_only_no_motion"`
- `controller_motion_allowed=false`
- `trajectory_execution_allowed=false`
- `motion_execution_enabled=false`
- `trajectory_execution_requested=false`
- availability of `moveit_ros_move_group`, `moveit_msgs`, and
  `moveit_kinematics`
- likely installed and source MoveIt config packages whose names contain
  `moveit_config`, `lbr_iisy`, `kuka`, or `iisy`
- `moveit_config_package_found` and `partial_moveit_config_found`
- selected `moveit_config_package_name` and `moveit_config_package_path`
- explicit discovered SRDF, `kinematics.yaml`, joint-limits, OMPL, and launch
  resource file lists
- `robot_description` visibility from `robot_state_publisher` parameter
  services when available
- observed `/joint_states` joint names
- `/compute_ik` service availability
- `moveit_ready_for_compute_ik`, which remains false unless the required config
  resources are present, move-group launch readiness is confirmed, and
  `/compute_ik` is visible

## Decision Policy

The audit reports `recommended_next_step="create_moveit_config_package"` when
no SRDF and no `kinematics.yaml` are found.

It reports `recommended_next_step="complete_moveit_config_package"` when some
MoveIt config files exist, but no complete config package or constructible
diagnostic launch can be identified.

It reports `recommended_next_step="prepare_move_group_diagnostic_launch"` when a
complete config package exists but `/compute_ik` is not running. MoveIt launch
remains blocked until the config package name and file paths are explicit.

It reports `recommended_next_step="test_compute_ik_service_no_motion"` only
when `/compute_ik` is visible. Testing that service still means diagnostic IK
requests only: no random targets, no fabricated joint solutions, no trajectory
goals, and no controller execution.

## Launch Preparation

`thesis_bringup/launch/run_moveit_ik_diagnostic.launch.py` starts only
`moveit_config_audit` and `ik_backend_audit`. It intentionally does not launch
`move_group`, `task_trajectory_executor`, Gazebo, or any controller client.

`move_group` should only be added after the config audit confirms the correct
KUKA LBR iisy MoveIt config resources, publishes explicit file paths, and the
launch is configured with trajectory execution blocked.

## Current Contract

Controller execution remains blocked. v2.8 prepares the diagnostic path toward
IK service availability, but it does not solve IK, generate joint targets, send
`FollowJointTrajectory` goals, or move the robot.
