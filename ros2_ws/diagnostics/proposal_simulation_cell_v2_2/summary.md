# proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation

Status: `moveit_ik_diagnostic_validated`

This diagnostic validates MoveIt IK availability without executing plans, trajectories, controllers, or real robot endpoints.

- move_group_started: true
- compute_ik_service_available: true
- compute_ik_called: true
- ik_solution_found: true
- ik_error_code: 1
- trajectory_execution_allowed: false
- controller_execution_allowed: false
- follow_joint_trajectory_execution_allowed: false
- planning_execution_allowed: false
- real_robot_used: false
- motion_executed: false
- trajectory_sent: false
