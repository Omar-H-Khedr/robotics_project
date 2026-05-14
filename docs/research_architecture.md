# Research Architecture

This repository is organized as a research-grade ROS 2 Jazzy and Gazebo framework for safe adaptive peg-in-hole assembly using a KUKA robot. The immediate goal is a publishable simulation platform that supports controlled experiments, clear module boundaries, reproducible logs, and later integration of perception and learning methods.

## Overall System Architecture

The framework is built around a modular ROS 2 graph:

1. `thesis_bringup` launches complete experiment configurations.
2. `peg_in_hole_description` provides the Gazebo task scene, object descriptions, world files, and task geometry.
3. `kuka_task_control` produces task-level robot commands for the KUKA arm and targets `joint_trajectory_controller`.
4. `safety_layer` filters proposed commands and reports safety violations.
5. `experiment_manager` coordinates trials, metadata, resets, logging, and result labeling.
6. `perception_pipeline` will provide simulated RGB-D sensing and task-state estimation.
7. `learning_interface` will expose observations, actions, rewards, and reset interfaces for adaptive or reinforcement-learning experiments.

Existing demo packages are retained as legacy/demo references. They should not be treated as the primary research architecture, but they can remain useful for regression checks and historical context.

The current canonical simulation entry point is `thesis_bringup`'s `research_baseline.launch.py`. It launches the configured peg-in-hole Gazebo world from `peg_in_hole_description`, exposes that package's `models` directory through `GZ_SIM_RESOURCE_PATH`, builds the robot description from the project-owned `peg_in_hole_description/urdf/lbr_iisy3_r760_research_gripper.urdf.xacro`, and starts the Gazebo entity spawn, bridge, `joint_state_broadcaster`, and `joint_trajectory_controller`.

Research Baseline v0.1 adds the reproducible trial entry point `thesis_bringup/launch/run_research_trial.launch.py`. It composes the Gazebo baseline, `safety_layer/safety_monitor`, and `experiment_manager/baseline_trial_manager`. Robot motion remains a deliberate second command through `kuka_task_control/launch/run_task_sequence.launch.py`, which sends `FollowJointTrajectory` action goals and publishes `/task_phase`.

The project-owned robot wrapper still reuses the upstream KUKA iisy meshes, kinematic macro, and ROS 2 control macro. It adds only the Phase 2 passive research gripper at the upstream `flange` attachment link, with a fixed `gripper_tcp` frame for peg-in-hole task programming. The upstream KUKA description remains unmodified.

## Why Gazebo Is the Main Simulator

Gazebo is the main simulator because the project requires ROS-native integration, robot controller compatibility, contact-rich task scenes, reproducible world configuration, and future sensor simulation. It provides a practical bridge between KUKA trajectory control, ROS 2 launch workflows, simulated RGB-D perception, and contact-event instrumentation.

For this thesis framework, Gazebo is not only a visualization tool. It is the controlled experimental environment where task geometry, robot motion, collision/contact behavior, sensing assumptions, and logging can be versioned and reproduced across experiment phases.

## Package Responsibilities

### thesis_bringup

Owns top-level launch files and high-level configuration for complete research runs. This package should define canonical experiment entry points such as baseline Gazebo control, safety-filtered control, perception-enabled control, and learning-enabled evaluation.

For the Phase 2B baseline, `thesis_bringup/config/research_baseline.yaml` records the simulation world package/file, task identity and frames, insertion axis, robot name, six KUKA joints, and home pose. The launch logs the resolved world, robot model, task frames, and expected controller stack at startup so experiment logs are self-describing.

### peg_in_hole_description

Owns the peg-in-hole task scene. This includes peg and hole geometry, fixtures, frames, Gazebo worlds, contact/material parameters, and task variants such as clearance, offset, and insertion depth.

For Phase 2, this package also owns the simplified passive gripper/end-effector model used by the research baseline. The gripper is deliberately primitive and fixed: palm block, left/right finger links, box visual/collision geometry, and `gripper_tcp`. Actuation and grasp-state modeling are deferred until the task controller and contact experiments need them.

### kuka_task_control

Owns task-level command generation for the KUKA arm. It should use the six KUKA joints:

- `joint_1`
- `joint_2`
- `joint_3`
- `joint_4`
- `joint_5`
- `joint_6`

The package should send controller-compatible `FollowJointTrajectory` action goals while keeping a clean boundary for safety filtering and logging. Direct publication to the trajectory command topic is not the baseline control path because the confirmed reliable control path is `/joint_trajectory_controller/follow_joint_trajectory`.

### safety_layer

Owns safety filters, safety monitors, and violation reporting. In v0.1 it is monitor-only: it subscribes to `/joint_states` and `/task_phase`, checks soft joint limits, NaN/Inf values, missing joint-state timeout, and phase-duration timeout placeholders, then publishes `/safety_status`.

### experiment_manager

Owns reproducible trial execution. In v0.1 it records one trial folder under `results/baseline_trials/` with metadata, joint-state CSV rows, task events, safety events, and a summary JSON with explicit placeholders for contact and task-success metrics.

### perception_pipeline

Owns future Gazebo RGB-D input handling and task-state estimation. It should publish perception-derived state in a way that can be compared against Gazebo ground truth and later transferred to real sensor pipelines.

### learning_interface

Owns the future interface between ROS 2 experiments and learning systems. It should define observations, actions, rewards, resets, and policy-evaluation boundaries without coupling the full framework to one specific learning library.

## Data Flow

The intended command and data flow is:

1. `experiment_manager` selects a trial configuration and starts the experiment through `thesis_bringup`.
2. `thesis_bringup` starts `peg_in_hole_description/worlds/peg_in_hole_world.sdf` and spawns the KUKA robot with the passive research gripper and controller stack into that same Gazebo world.
3. `kuka_task_control` sends the scripted task sequence through the `FollowJointTrajectory` action interface and publishes `/task_phase`.
4. `safety_layer` monitors `/joint_states` and `/task_phase` and publishes `/safety_status`.
5. `joint_trajectory_controller` executes the accepted action goals in the KUKA Gazebo simulation.
6. Gazebo and ROS 2 topics provide robot state, task state, contacts, controller feedback, and later RGB-D sensor data.
7. `experiment_manager` records logs and computes trial outcomes.
8. `perception_pipeline` and `learning_interface` can be added without changing the baseline controller or safety boundary.

## Future Publication-Oriented Milestones

- Establish a validated KUKA Gazebo baseline with repeatable joint trajectory execution.
- Build a parameterized peg-in-hole task scene with controlled clearance and misalignment.
- Introduce safety filters and compare filtered versus unfiltered command execution.
- Add reproducible trial management with structured logs and metric extraction.
- Add simulated RGB-D perception and compare perception-estimated task state against simulator ground truth.
- Add adaptive or reinforcement-learning policy interfaces behind the same safety layer.
- Evaluate success rate, insertion robustness, collision/contact behavior, trajectory time, safety violations, and repeatability over controlled trial sets.
