# ros2_ws

This folder is reserved for the future ROS 2 workspace.

When ROS 2 is installed later, this workspace can contain build, install, log, and source folders used by ROS 2 development tools.

## Milestones

| Milestone | Status |
| --- | --- |
| proposal_simulation_cell_v1_2_rgbd_image_bridge_fix | Completed |
| proposal_simulation_cell_v1_3_contact_physics_validation | Completed |
| proposal_simulation_cell_v1_5_safety_virtual_force_interface | Completed |
| proposal_simulation_cell_v1_6_safety_gate_readiness | Completed |
| proposal_simulation_cell_v1_7_pre_control_contract | Completed |
| proposal_simulation_cell_v1_8_control_development_scaffold | Completed |
| proposal_simulation_cell_v1_9_no_motion_control_law_dry_run | Completed |
| proposal_simulation_cell_v1_10_experiment_configuration_matrix | Completed |
| proposal_simulation_cell_v1_11_single_scenario_loader_validation | Completed |
| proposal_simulation_cell_v1_12_scenario_batch_selector | Completed |

## proposal_simulation_cell_v1_12_scenario_batch_selector

Status: `scenario_batch_selector_validated`

The v1.12 proposal simulation sprint adds a scenario batch selector. It loads a representative selected batch from the v1.10 matrix and validates the selected scenarios.

The batch is configuration-only: no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_12/`.

## proposal_simulation_cell_v1_11_single_scenario_loader_validation

Status: `single_scenario_loader_validated`

The v1.11 proposal simulation sprint adds a single-scenario loader. It loads the selected scenario from the v1.10 matrix and validates the selected scenario configuration.

The loader is configuration-only: no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_11/`.

## proposal_simulation_cell_v1_10_experiment_configuration_matrix

Status: `experiment_configuration_matrix_validated`

The v1.10 proposal simulation sprint adds an experiment configuration matrix for future peg-in-hole validation scenarios. It defines scenario variants for clearance, x/y offset, angular misalignment, insertion depth, and contact thresholds.

The matrix is configuration-only: no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_10/`.

## proposal_simulation_cell_v1_9_no_motion_control_law_dry_run

Status: `no_motion_control_law_dry_run_validated`

The v1.9 proposal simulation sprint adds a no-motion control-law dry run. It reads validated simulated inputs and generates diagnostic control-law output, a blocked control command, and safety clipping/reporting evidence without connecting any output to execution.

The dry-run command remains blocked. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_9/`.

## proposal_simulation_cell_v1_8_control_development_scaffold

Status: `control_development_scaffold_validated`

The v1.8 proposal simulation sprint adds a control-development scaffold for future controller work without executing robot motion. It includes the control input monitor, diagnostic command proposal, command blocker, safety gate checker, control boundary checker, and control readiness report.

The command proposal is diagnostic only and blocked. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_8/`.

## proposal_simulation_cell_v1_7_pre_control_contract

Status: `pre_control_contract_validated`

The v1.7 proposal simulation sprint adds a pre-control simulation contract. It defines the required input signal contract, allowed diagnostic output suggestions, forbidden execution interfaces, readiness dependency contract, and future controller boundary before any controller work is introduced.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_7/`.

Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

## proposal_simulation_cell_v1_6_safety_gate_readiness

Status: `safety_gate_readiness_validated`

The v1.6 proposal simulation sprint adds readiness gates for the next control-development stage. It evaluates the sensor gate, contact gate, safety gate, virtual-force gate, admittance gate, execution-disabled gate, and proposal readiness gate from validated simulation diagnostic signals only.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_6/`.

Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, no real robot execution, no `FollowJointTrajectory`, and no command output.

## proposal_simulation_cell_v1_5_safety_virtual_force_interface

Status: `safety_virtual_force_interface_validated`

The v1.5 proposal simulation sprint adds a simulation-only runtime safety and virtual-force interface foundation. It adds the safety status interface on `/proposal_simulation_cell/safety_status`, contact-state classification on `/proposal_simulation_cell/contact_state`, virtual-force diagnostic command suggestions on `/proposal_simulation_cell/virtual_force_command`, and admittance diagnostic command suggestions on `/proposal_simulation_cell/admittance_command_suggestion`.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_5/`. The validated run used Gazebo fallback because Isaac Sim was unavailable. The contact wrench topic and sample were available, the maximum observed force was `0.0981000000182301 N`, and the final contact state was `contact_below_threshold` against the configured `0.1 N` detection threshold.

Safety constraints are enforced in config and diagnostics: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, no real robot execution, no `FollowJointTrajectory`, and no command execution.
