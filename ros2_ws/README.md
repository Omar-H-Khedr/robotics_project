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
| proposal_simulation_cell_v1_13_batch_execution_plan_validator | Completed |
| proposal_simulation_cell_v1_14_batch_dry_run_orchestrator | Completed |
| proposal_simulation_cell_v1_15_evidence_package_generator | Completed |
| proposal_simulation_cell_v1_16_reproducibility_checklist | Completed |
| proposal_simulation_cell_v1_17_release_documentation_index | Completed |
| proposal_simulation_cell_v2_0_first_gazebo_motion_smoke_test | Completed |
| proposal_simulation_cell_v2_1_gazebo_motion_validation_suite | Completed |
| proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation | Completed |
| proposal_simulation_cell_v2_3_moveit_model_alignment_and_plan_only_validation | Completed |
| proposal_simulation_cell_v2_4_moveit_gazebo_execution_validation | Completed |
| proposal_simulation_cell_v2_5_guarded_pre_contact_task_sequence | Completed |
| proposal_simulation_cell_v2_6_contact_gated_guarded_approach_validation | Completed |
| proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_calibration | Completed |
| proposal_simulation_cell_v2_8_contact_reachability_and_trigger_validation | Completed |
| proposal_simulation_cell_v2_9_non_overlapping_approach_to_contact_validation | Completed |
| proposal_simulation_cell_v2_10_misalignment_contact_gate_batch_validation | Completed |
| proposal_simulation_cell_v2_11_multimodal_contact_observation_logging | Completed |
| proposal_simulation_cell_v2_12_context_vector_extraction | Completed |
| proposal_simulation_cell_v2_13_context_encoder_prototype | Completed |
| proposal_simulation_cell_v2_14_context_conditioned_guarded_action_validation | Completed |
| proposal_simulation_cell_v2_15_context_action_ablation_validation | Completed |

## proposal_simulation_cell_v2_15_context_action_ablation_validation

Status: `context_action_ablation_validated`

The v2.15 proposal simulation sprint adds a paired diagnostic ablation comparing fixed-baseline guarded action parameters with deterministic context-conditioned guarded action parameters. The five validated scenarios are tested under two action modes, and paired comparison reports are generated for trigger step, max force, final return error, safety violations, and action parameter differences.

This sprint is diagnostic ablation only. It does not run RL training, train a policy, create fake learning results, use a real robot, use a physical endpoint, execute peg insertion, or perform forceful contact. Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_15/`.

## proposal_simulation_cell_v2_14_context_conditioned_guarded_action_validation

Status: `context_conditioned_guarded_action_validated`

The v2.14 proposal simulation sprint uses the real v2.13 deterministic context embeddings to generate guarded action suggestions for the five validated contact-gate scenarios. It validates the suggested action parameters against safety bounds, then performs Gazebo-only contact-gated execution with initial no-contact checks, stop-on-contact, retreat, post-retreat no-contact checks, and return-to-ready checks.

This sprint does not train a policy, run RL training, create fake learning results, use a real robot, use a physical endpoint, execute peg insertion, or perform forceful contact. Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_14/`.

## proposal_simulation_cell_v2_13_context_encoder_prototype

Status: `context_encoder_prototype_validated`

The v2.13 proposal simulation sprint adds a deterministic context encoder prototype using the real v2.12 simulation context vectors. It defines a stable context feature schema, validates required normalized features, generates deterministic 8-D context embeddings for each scenario, and writes similarity plus nearest-context reports.

The prototype does not train a policy, run RL training, create fake learning results, use a real robot, or execute peg insertion. Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_13/`.

## proposal_simulation_cell_v2_12_context_vector_extraction

Status: `context_vector_extraction_validated`

The v2.12 proposal simulation sprint extracts compact context vectors from the real v2.11 Gazebo simulation observation logs. It reads the v2.11 multimodal observation log, contact-transition log, scenario summary, RGB-D frame-count report, channel completeness report, and safety report.

Scenario-level context vectors, contact-transition feature vectors, episode summaries, observation-channel summaries, safety-gated context summaries, and a metadata manifest are generated. This sprint performs feature extraction only: no fake dataset, fake result, learning, policy training, real robot execution, peg insertion, or forceful contact is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_12/`.

## proposal_simulation_cell_v2_11_multimodal_contact_observation_logging

Status: `multimodal_contact_observation_logging_validated`

The v2.11 proposal simulation sprint adds synchronized multimodal contact observation logging. It replays the five validated v2.10 misalignment contact-gate scenarios and records observation rows for RGB-D availability, joint state, TF/tool pose, contact wrench, task phase, scenario metadata, and contact transition labels.

RGB-D topics were available, frame counts were recorded, and lightweight metadata was saved without writing full image datasets. The outputs are real Gazebo simulation observation logs only. No fake dataset, experimental performance claim, real robot execution, physical endpoint, peg insertion, forceful contact, or learning is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_11/`.

## proposal_simulation_cell_v2_10_misalignment_contact_gate_batch_validation

Status: `misalignment_contact_gate_batch_validated`

The v2.10 proposal simulation sprint adds misalignment contact-gate batch validation using MoveIt planning and Gazebo-only execution. It runs five actual Gazebo scenarios: nominal centered, positive x offset, negative x offset, positive y offset, and negative y offset. Each scenario computes the calibration pad pose from robot/tool/table geometry, applies the lateral offset, verifies initial no-contact, triggers contact after guarded approach motion, stops on contact, retreats, verifies post-retreat no-contact, and returns to ready.

Scenario definitions, pad poses, IK reachability, initial no-contact checks, contact transitions, post-retreat checks, safety evidence, and endpoint checks are recorded. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or fake scenario evidence is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_10/`.

## proposal_simulation_cell_v2_9_non_overlapping_approach_to_contact_validation

Status: `non_overlapping_approach_to_contact_validated`

The v2.9 proposal simulation sprint adds non-overlapping approach-to-contact validation using MoveIt planning and Gazebo-only execution. It computes the robot/tool/table/pad geometry, places the simulation-only calibration pad on the computed tool path with positive clearance, verifies the initial no-contact standby condition, and executes bounded approach motion until contact triggers after motion rather than at step 0.

Stop-on-contact, retreat, post-retreat no-contact, return-to-ready, raw contact evidence, and derived compliant force evidence are recorded. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_9/`.

## proposal_simulation_cell_v2_8_contact_reachability_and_trigger_validation

Status: `contact_reachability_and_trigger_validated`

The v2.8 proposal simulation sprint adds contact reachability and trigger validation using MoveIt planning and Gazebo-only execution. It computes a simulation-only calibration pad pose relative to the tool/distal-link path, checks raw contact topic wiring, records raw contact plus derived wrench evidence, and runs bounded contact-trigger steps through the verified Gazebo simulation endpoint.

The contact gate triggered with nonzero raw contact and derived wrench evidence. Stop-on-contact, retreat, return-to-ready, and final state validation are recorded. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_8/`.

## proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_calibration

Status: `contact_triggered_guarded_touch_not_reached`

The v2.7 proposal simulation sprint adds contact-triggered guarded touch calibration using MoveIt planning and Gazebo-only execution. It uses a simulation-only contact calibration target, records guarded touch steps, records contact wrench reports, and validates stop-on-contact plus retreat behavior if the contact gate triggers.

The contact gate was not reached within the bounded guarded touch steps, and the diagnostic output records that result without fake contact evidence. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_7/`.

## proposal_simulation_cell_v2_6_contact_gated_guarded_approach_validation

Status: `contact_gated_guarded_approach_validated_no_contact_detected`

The v2.6 proposal simulation sprint adds a contact-gated guarded approach sequence using MoveIt planning and Gazebo-only execution. It validates ready, pre-approach, pre-contact standby, guarded approach steps, stop-on-contact or stand-off gating, retreat, return-to-ready, and final state validation through the verified Gazebo simulation endpoint.

Guarded approach step reports, contact gate reports, joint-state evidence, endpoint checks, and contact wrench monitoring are recorded. No real robot execution, physical endpoint, peg insertion, forceful contact, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_6/`.

## proposal_simulation_cell_v2_5_guarded_pre_contact_task_sequence

Status: `guarded_pre_contact_task_sequence_validated`

The v2.5 proposal simulation sprint adds a guarded pre-contact task sequence using MoveIt planning and Gazebo-only execution. It validates ready, pre-approach, pre-insertion standby, hold, and return phases while verifying the Gazebo simulation endpoint before executed phases.

Phase reports, joint-state evidence, endpoint checks, and contact wrench monitoring are recorded. No real robot execution, physical endpoint, peg insertion, contact-seeking motion, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_5/`.

## proposal_simulation_cell_v2_4_moveit_gazebo_execution_validation

Status: `moveit_gazebo_execution_validated`

The v2.4 proposal simulation sprint adds the first MoveIt-generated Gazebo-only trajectory execution. It verifies the Gazebo simulation controller endpoint, generates and executes one small MoveIt plan only in Gazebo, records joint-state before/after evidence, returns to the initial posture, and monitors the contact wrench.

No real robot execution, physical endpoint, peg insertion, contact-seeking motion, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_4/`.

## proposal_simulation_cell_v2_3_moveit_model_alignment_and_plan_only_validation

Status: `moveit_model_alignment_and_plan_only_validated`

The v2.3 proposal simulation sprint adds a MoveIt/Gazebo model alignment audit, five nearby diagnostic IK checks for repeatability, and MoveIt plan-only validation.

No trajectory execution, controller execution, real robot execution, `FollowJointTrajectory` execution, peg insertion, contact-seeking motion, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_3/`.

## proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation

Status: `moveit_ik_diagnostic_validated`

The v2.2 proposal simulation sprint adds diagnostic-only MoveIt IK validation. It loads the diagnostic MoveIt model, starts `move_group` with trajectory execution disabled, verifies `/compute_ik`, and records the IK request and response.

No real robot execution, controller execution, trajectory execution, `FollowJointTrajectory` execution, peg insertion, contact-seeking motion, learning, or scenario batch execution is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_2/`.

## proposal_simulation_cell_v2_1_gazebo_motion_validation_suite

Status: `gazebo_motion_validation_suite_validated`

The v2.1 proposal simulation sprint adds a combined Gazebo-only motion validation suite. It tests single forward and return motion, three repeatability cycles, and a small two-joint motion using the Gazebo simulation controller.

The suite records joint-state evidence, repeatability and return errors, contact wrench monitoring, and a safety report. No real robot execution, MoveIt, `/compute_ik`, learning, scenario batch execution, peg insertion, or contact-seeking motion is used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_1/`.

## proposal_simulation_cell_v2_0_first_gazebo_motion_smoke_test

Status: `first_gazebo_motion_smoke_test_validated`

The v2.0 proposal simulation sprint adds the first intentional Gazebo-only motion smoke test. It sends one small joint-space movement for the selected sixth-axis joint, records joint-state before/after evidence, monitors the contact wrench topic, and writes a safety report.

The smoke test is Gazebo-only: no real robot execution, no MoveIt, no `/compute_ik`, no learning, and no scenario batch execution are used.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v2_0/`.

## proposal_simulation_cell_v1_17_release_documentation_index

Status: `release_documentation_index_validated`

The v1.17 proposal simulation sprint adds a release documentation index, reviewer quickstart, sprint traceability, and no-false-claims statement. The documents link the v1.15 evidence package and v1.16 reproducibility checklist, summarize completed sprints v1.0, v1.1, v1.2, v1.3, and v1.5 through v1.16, and confirm that v1.4 remains absent/not implemented.

The release index is documentation-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, no `FollowJointTrajectory`, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_17/`.

## proposal_simulation_cell_v1_16_reproducibility_checklist

Status: `reproducibility_checklist_validated`

The v1.16 proposal simulation sprint adds a reproducibility checklist and reviewer-facing implementation summary. It verifies that the v1.15 evidence package and evidence registry are available, checks implemented diagnostics folders, and confirms that v1.4 remains absent/not implemented.

The checklist is diagnostic-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_16/`.

## proposal_simulation_cell_v1_15_evidence_package_generator

Status: `evidence_package_validated`

The v1.15 proposal simulation sprint adds an evidence package generator. It collects evidence from v1.0, v1.1, v1.2, v1.3, and v1.5 through v1.14, marks v1.4 as absent/not implemented and not invented, generates the proposal simulation evidence package, and creates a validated evidence summary.

The package is evidence-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_15/`.

## proposal_simulation_cell_v1_14_batch_dry_run_orchestrator

Status: `batch_dry_run_orchestrator_validated`

The v1.14 proposal simulation sprint adds a batch dry-run orchestrator. It converts the v1.13 batch execution plan into blocked dry-run orchestration records, defines the per-scenario gate-check order, and adds the blocked batch execution report.

The orchestration is configuration-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_14/`.

## proposal_simulation_cell_v1_13_batch_execution_plan_validator

Status: `batch_execution_plan_validated`

The v1.13 proposal simulation sprint adds a batch execution plan validator. It converts the selected v1.12 batch into a configuration-only execution plan, lists required gates for every scenario, and defines planned diagnostic outputs.

The plan is configuration-only: no scenario execution, no fake datasets, no fake plots, and no experimental results are generated. Safety constraints remain explicit: `command_output_enabled=false`, `motion_execution_enabled=false`, no MoveIt, no `/compute_ik`, no controllers, and no real robot execution.

Evidence is stored in `ros2_ws/diagnostics/proposal_simulation_cell_v1_13/`.

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
