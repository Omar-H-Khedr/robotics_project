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

## Why Gazebo Is the Main Simulator

Gazebo is the main simulator because the project requires ROS-native integration, robot controller compatibility, contact-rich task scenes, reproducible world configuration, and future sensor simulation. It provides a practical bridge between KUKA trajectory control, ROS 2 launch workflows, simulated RGB-D perception, and contact-event instrumentation.

For this thesis framework, Gazebo is not only a visualization tool. It is the controlled experimental environment where task geometry, robot motion, collision/contact behavior, sensing assumptions, and logging can be versioned and reproduced across experiment phases.

## Package Responsibilities

### thesis_bringup

Owns top-level launch files and high-level configuration for complete research runs. This package should define canonical experiment entry points such as baseline Gazebo control, safety-filtered control, perception-enabled control, and learning-enabled evaluation.

### peg_in_hole_description

Owns the peg-in-hole task scene. This includes peg and hole geometry, fixtures, frames, Gazebo worlds, contact/material parameters, and task variants such as clearance, offset, and insertion depth.

### kuka_task_control

Owns task-level command generation for the KUKA arm. It should use the six KUKA joints:

- `joint_1`
- `joint_2`
- `joint_3`
- `joint_4`
- `joint_5`
- `joint_6`

The package should publish controller-compatible commands while keeping a clean boundary for safety filtering.

### safety_layer

Owns safety filters, safety monitors, and violation reporting. It should evaluate proposed robot commands against configured constraints and publish either safe commands or structured violation information.

### experiment_manager

Owns reproducible trial execution. It should manage trial manifests, parameter sweeps, reset logic, metadata, rosbag recording, metrics extraction hooks, and result summaries.

### perception_pipeline

Owns future Gazebo RGB-D input handling and task-state estimation. It should publish perception-derived state in a way that can be compared against Gazebo ground truth and later transferred to real sensor pipelines.

### learning_interface

Owns the future interface between ROS 2 experiments and learning systems. It should define observations, actions, rewards, resets, and policy-evaluation boundaries without coupling the full framework to one specific learning library.

## Data Flow

The intended command and data flow is:

1. `experiment_manager` selects a trial configuration and starts the experiment through `thesis_bringup`.
2. `peg_in_hole_description` provides the Gazebo scene and task frames.
3. `kuka_task_control` generates a proposed joint trajectory or task command.
4. `safety_layer` evaluates the proposed command and publishes a filtered command to the controller.
5. `joint_trajectory_controller` executes the command in the KUKA Gazebo simulation.
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
