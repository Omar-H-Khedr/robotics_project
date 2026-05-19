# Research Baseline v2.7: IK Backend Audit

## Purpose

Research Baseline v2.7 does not solve IK yet. It audits the available IK
infrastructure and publishes a diagnostic decision report that identifies
whether the next implementation step should be MoveIt configuration or a
project-owned custom IK service.

The audit is a no-motion diagnostic layer. It does not launch
`task_trajectory_executor`, does not send `FollowJointTrajectory` goals, does
not add random joint targets, and does not fabricate IK solutions.

## Published Report

v2.7b makes the audit report explicit and machine-readable: diagnostic booleans
and lists are always present instead of relying on consumers to infer missing
fields from nested reports.

`ik_backend_audit` publishes JSON on `/ik_backend_audit` with:

- `status="ik_backend_audit_diagnostic_only_no_motion"`
- `motion_execution_enabled=false`
- `trajectory_execution_requested=false`
- `controller_motion_allowed=false`
- explicit top-level IK service diagnostics: `compute_ik_service_available`,
  `compute_ik_services`, `ik_services`, and `moveit_services`
- explicit package diagnostics: `moveit_packages_available`,
  `available_packages`, and `missing_packages`
- explicit robot-state diagnostics: `robot_description_available` with a
  reason, `joint_states_available`, `joint_names_observed`,
  `joint_positions_observed`, `joint_limits_file_available`, and
  `joint_limits_file_path`
- explicit project-readiness diagnostics: `full_pose_dry_run_available`,
  `orientation_targets_available`, and `execution_gate_status_observed`
- visible `/compute_ik`, `compute_ik`-like, and MoveIt planning services
- availability of relevant ROS packages through `ament_index_python`
- robot model resource availability, including `robot_description` visibility,
  `/joint_states` joint names, joint-limits file readability, and KUKA LBR iisy
  URDF/xacro discovery from package share folders
- observed project readiness from `/cartesian_insertion_dry_run_plan`,
  `/cartesian_orientation_targets`, and `/execution_gate_status`
- `ik_backend_available`, `recommended_backend`, `recommended_next_step`, and a
  short decision reason

## Decision Policy

The audit reports `recommended_backend="moveit_compute_ik"` only when a
callable `/compute_ik` path is visible and the required MoveIt IK message
dependencies are importable.

It reports `recommended_backend="configure_moveit"` when MoveIt-related
packages are present but no callable IK service is running.

It reports `recommended_backend="add_moveit_or_custom_ik_service"` when no IK
backend is available and the project must either add MoveIt integration or
provide a dedicated custom IK service.

## Current Contract

Controller execution remains blocked. v2.7 only decides what infrastructure is
missing before the project can request real IK solutions for the full-pose
Cartesian waypoints validated in v2.6.

v2.7b does not enable motion. If no real callable IK service is present,
`ik_backend_available` remains false and the absence of IK keeps controller
execution blocked.
