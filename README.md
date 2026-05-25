# Visuomotor Context-Based Meta-RL with Virtual-Force Safety for Peg-in-Hole Assembly

This repository documents the technical development of my doctoral research project on safe and adaptable robotic peg-in-hole assembly for smart manufacturing.

## Project Status

The current implementation focuses on a ROS 2 Jazzy and Gazebo-based research framework for:

- KUKA LBR iisy simulation workcell
- Peg-in-hole task environment
- Joint-space task execution baseline
- Safety monitoring layer
- Experiment logging and trial summaries
- Gazebo contact sensing and contact-force extraction
- Robot-generated contact validation with force guard logic

## Proposal Simulation Cell Progress

### proposal_simulation_cell_v1_3_contact_physics_validation

Validated the proposal-aligned simulation foundation using the Gazebo fallback. RGB-D sampling remained valid, peg/hole/table collision bodies were configured, and Gazebo contact evidence was captured between the peg and hole collision objects. A nonzero contact wrench was observed with max force approximately 0.0981 N, with `safety_violation_count=0`. MoveIt, `/compute_ik`, controllers, and real robot execution were not used.

### proposal_simulation_cell_v1_5_safety_virtual_force_interface

Added the simulation-only safety status interface, contact-state classification, virtual-force diagnostic command suggestions, and admittance diagnostic command suggestions. The interface reads simulated contact wrench, joint state, TF, TF static, and task phase signals, then publishes diagnostic outputs only on `/proposal_simulation_cell/safety_status`, `/proposal_simulation_cell/contact_state`, `/proposal_simulation_cell/virtual_force_command`, and `/proposal_simulation_cell/admittance_command_suggestion`.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_5/`. Safety constraints remain enforced: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

### proposal_simulation_cell_v1_9_no_motion_control_law_dry_run

Added the simulation-only no-motion control-law dry run. It reads validated simulated inputs, generates diagnostic control-law output, generates a blocked control command, and adds safety clipping/reporting without connecting any output to controllers or execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_9/`. Safety constraints remain enforced: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

### proposal_simulation_cell_v1_10_experiment_configuration_matrix

Added the simulation-only experiment configuration matrix for future peg-in-hole validation scenarios. It defines scenario variants for clearance, x/y offset, angular misalignment, insertion depth, and contact thresholds.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_10/`. The matrix is configuration-only: no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain enforced: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

### proposal_simulation_cell_v1_11_single_scenario_loader_validation

Added the simulation-only single-scenario loader. It loads the selected scenario from the v1.10 matrix and completes selected scenario validation.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_11/`. The loader is configuration-only: no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain enforced: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

### proposal_simulation_cell_v1_12_scenario_batch_selector

Added the simulation-only scenario batch selector. It loads a representative selected batch from the v1.10 matrix and validates the selected scenarios.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_12/`. The batch is configuration-only: no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain enforced: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

### proposal_simulation_cell_v1_13_batch_execution_plan_validator

Added the simulation-only batch execution plan validator. It converts the selected v1.12 batch into a configuration-only execution plan, lists required gates for every scenario, and defines planned diagnostic outputs.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_13/`. The plan is configuration-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain enforced: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

### proposal_simulation_cell_v1_14_batch_dry_run_orchestrator

Added the simulation-only batch dry-run orchestrator. It converts the v1.13 batch execution plan into blocked dry-run orchestration records, defines the per-scenario gate-check order, and adds the blocked batch execution report.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_14/`. The orchestration is configuration-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain enforced: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

### proposal_simulation_cell_v1_15_evidence_package_generator

Added the simulation-only evidence package generator. It collects evidence from v1.0, v1.1, v1.2, v1.3, and v1.5 through v1.14, marks v1.4 as absent/not implemented and not invented, generates the proposal simulation evidence package, and creates a validated evidence summary.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_15/`. The package is evidence-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain enforced: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

## Current Stable Milestones

| Version | Description | Status |
|---|---|---|
| v0.1 | Stable Gazebo KUKA workcell baseline | Completed |
| v0.2 | Full task sequence with logging and safety monitor | Completed |
| v0.3 | Contact metrics infrastructure and diagnostics | Completed |
| v0.4 | Minimal Gazebo contact validation world | Completed |
| v0.5 | Contact force extraction from Gazebo Contacts messages | Completed |
| v0.6 | Robot-generated contact validation | Completed |
| v0.7 | Force-threshold diagnostics | Completed |
| v0.8/v0.9 | Force-guarded and early-contact guard experiments | In progress |
| v1.5 | Proposal simulation safety and virtual-force diagnostic interface | Completed |
| v1.9 | No-motion control-law dry run | Completed |
| v1.10 | Experiment configuration matrix | Completed |
| v1.11 | Single-scenario loader validation | Completed |
| v1.12 | Scenario batch selector | Completed |
| v1.13 | Batch execution plan validator | Completed |
| v1.14 | Batch dry-run orchestrator | Completed |
| v1.15 | Evidence package generator | Completed |
| v1.8 | Low-force segmented robot contact validation | Completed |
| v2.0 | Peg/hole insertion validation instrumentation | In progress |
| v2.3 | Coordinate-based insertion diagnostics | In progress |
| v2.4 | Object-frame publisher for insertion targets | In progress |
| v2.5 | IK feasibility diagnostics before motion | In progress |
| v2.5c | Unified execution gates and tool-axis audit | In progress |
| v2.5d | Diagnostic Cartesian orientation target calculation | In progress |
| v2.5e/v2.5f | Orientation-aware IK diagnostics and full-pose waypoint policy | In progress |
| v2.6 | Diagnostic-only Cartesian dry-run insertion plan | In progress |
| v2.7 | Diagnostic-only IK backend audit and decision report | In progress |
| v2.8 | MoveIt configuration audit and non-motion IK launch preparation | In progress |
| v2.9 | MoveIt IK diagnostic launch readiness audit | In progress |
| v2.10 | LBR iisy 6 R1300 semantic candidate for MoveIt IK diagnostics | In progress |
| v2.11 | robot_description_semantic diagnostics and MoveIt readiness gating | In progress |
| v2.12 | Diagnostic tool-link validation for MoveIt IK readiness | In progress |
| v2.13 | MoveIt diagnostic input bundle preparation | In progress |
| v2.14 | Diagnostic-only move_group launch path with execution disabled | In progress |

## Recommended Launch Commands

### Full research baseline trial

```bash
cd ~/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup run_full_research_trial.launch.py
```

### Peg/hole insertion validation trial

```bash
cd ~/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup run_full_peg_hole_insertion_validation_trial.launch.py
```

### Coordinate-based insertion diagnostics

```bash
cd ~/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup run_full_cartesian_insertion_diagnostics.launch.py
```

This diagnostic launch publishes named peg/hole target frames, reports
Cartesian distances, audits tool-axis alignment, computes diagnostic target
orientations for all planned waypoints, including `staging_pose`, on
`/cartesian_orientation_targets`, assembles the full no-motion Cartesian dry-run
plan on `/cartesian_insertion_dry_run_plan`, combines the execution gates on
`/execution_gate_status`, and publishes the IK backend decision report on
`/ik_backend_audit` plus the MoveIt configuration and launch readiness audits
on `/moveit_config_audit` and `/moveit_launch_readiness_audit`, the MoveIt
diagnostic input bundle on `/moveit_diagnostic_inputs`, plus the semantic
candidate validation reports on `/semantic_model_validation` and
`/robot_description_semantic_diagnostics`. It does not
start `task_trajectory_executor`, does not send trajectory goals, and does not
command robot motion. Controller execution remains blocked until geometry, IK,
real IK solutions for every waypoint, exact semantic model compatibility,
MoveIt configuration readiness, tool-axis validation, diagnostic tool-link
validation, safety, and force/contact gates all pass.

### MoveIt IK diagnostic preparation

```bash
cd ~/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup run_moveit_ik_diagnostic.launch.py
```

For the v2.14 move-group diagnostic path:

```bash
cd ~/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch thesis_bringup run_move_group_ik_diagnostic.launch.py
```

By default this launch starts only diagnostics:
`move_group_diagnostic_config_builder`, `moveit_diagnostic_input_builder`,
`robot_description_semantic_diagnostics`, `semantic_model_validator`,
`tool_link_validator`, `moveit_launch_readiness_audit`, `moveit_config_audit`,
`ik_backend_audit`, and `move_group_runtime_audit`. It does not launch
`move_group`, `task_trajectory_executor`, Gazebo, or any trajectory client.
The optional `launch_move_group:=true` path is diagnostic-only and sets
`allow_trajectory_execution=false`; it must not execute plans or send
controller goals.

v2.10 adds a project-local semantic candidate for `lbr_iisy6_r1300` under
`ros2_ws/src/kuka_task_control/config/moveit_lbr_iisy6_r1300/`. It is derived
from the same-family iisy11 R1300 template, marked
`candidate_requires_validation`, and is not approved for robot motion.

v2.11 adds `robot_description_semantic_diagnostics` on
`/robot_description_semantic_diagnostics` to report the SRDF candidate as a
future `robot_description_semantic` source. The SRDF can be structurally valid
while still not approved for motion; `/compute_ik` is not called and controller
execution remains blocked.

v2.12 adds `tool_link_validator` on `/tool_link_validation` to validate `tool0`
as a diagnostic tool/planning link candidate using TF, `robot_description` URDF
links, the project-local SRDF candidate, and optional tool-axis/orientation
diagnostics. A valid result prepares move-group diagnostic launch inputs only;
motion approval remains false.

v2.13 adds `moveit_diagnostic_input_builder` on `/moveit_diagnostic_inputs` to
assemble the future diagnostic `move_group` input bundle without launching
`move_group` or calling `/compute_ik`. It reports `robot_description`, SRDF,
MoveIt YAML, planning-frame, tool-link, and safety readiness while keeping
`approved_for_motion=false`, `move_group_launch_allowed=false`,
`controller_motion_allowed=false`, and `trajectory_execution_allowed=false`.

v2.14 adds `move_group_diagnostic_config_builder` on
`/move_group_diagnostic_config` and `move_group_runtime_audit` on
`/move_group_runtime_audit`. The default move-group diagnostic launch remains
blocked. If explicitly enabled, `move_group` is launched only to expose service
availability, with trajectory execution disabled and motion still disallowed.

# Robotics Project

This repository is being developed as the main PhD robotics project, focused on
KUKA robot simulation for contact-rich manipulation and adaptive control.

The early mobile robot ROS 2 packages remain in the repository as infrastructure
validation work. They were used to verify the ROS 2 workspace, package creation,
basic publisher/subscriber workflows, and simulation organization before moving
to the main KUKA manipulation research direction. They should not be treated as
the primary research target.

## PhD Research Direction

The current project direction is practical, simulation-first research around:

- ROS 2-based control and experiment orchestration.
- Gazebo simulation for repeatable manipulation scenarios.
- KUKA robot descriptions, launch files, and baseline controllers.
- Contact-rich tasks such as surface interaction, constrained motion, insertion,
  pushing, and force-sensitive manipulation.
- Data logging pipelines for robot state, command signals, contact events,
  task outcomes, and environment metadata.
- Future AI-based adaptation for improving controller behavior under contact,
  uncertainty, and changing task conditions.

## Repository Layout

- `ros2_ws/`: ROS 2 workspace containing existing validation packages and future
  KUKA simulation packages.
- `docs/phd_plan/`: Research planning, roadmap, and design notes.
- `experiments/kuka_baseline/`: Baseline KUKA simulation and control experiments.
- `experiments/contact_tasks/`: Contact-rich manipulation task experiments.
- `experiments/data_logging/`: Data collection and logging experiments.
- `scripts/kuka_tools/`: Helper scripts for KUKA simulation, experiment setup,
  and analysis support.
