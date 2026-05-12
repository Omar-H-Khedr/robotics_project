# System Architecture

## Overview

The real research project is organized as a ROS 2 simulation stack for one KUKA LBR iisy robot. The architecture separates robot modeling, simulation launch, context adaptation, safety checks, and reinforcement learning interfaces so each research component can be developed and evaluated independently.

The existing sandbox ROS 2/Gazebo packages remain in the repository but are not part of this architecture.

## Package Roles

### `ros2_ws/src/kuka_description`

Owns the robot model.

Planned contents:

- URDF/Xacro model for the selected KUKA LBR iisy variant.
- Mesh references and visual/collision geometry.
- Joint limits, inertial parameters, transmissions, and ROS 2 control tags.
- Optional payload attachment descriptions.

### `ros2_ws/src/kuka_simulation`

Owns simulator integration.

Planned contents:

- Gazebo launch files for the baseline simulation.
- World files for free-space, payload, and contact experiments.
- Controller configuration for simulation.
- Later compatibility hooks for Isaac Sim if higher-fidelity contact or rendering is needed.

### `ros2_ws/src/kuka_context_adapter`

Owns context estimation and adaptation logic.

Planned contents:

- Context state definitions for payload and contact conditions.
- Estimators that infer context from joint states, effort signals, contact force, task error, or simulator ground truth during early experiments.
- Adapter nodes that modify controller parameters, impedance settings, references, or policy inputs.

### `ros2_ws/src/kuka_safety_layer`

Owns command filtering and constraint enforcement.

Planned contents:

- Joint limit checks.
- Velocity, acceleration, and torque bounds.
- Workspace and self-collision guard hooks.
- Contact force thresholds.
- Emergency stop and safe fallback command behavior for simulation.

The safety layer should sit between high-level command sources and the simulated robot controller.

### `ros2_ws/src/kuka_rl_env`

Owns reinforcement learning integration.

Planned contents:

- Environment wrappers around ROS 2 simulation tasks.
- Observation, action, reward, reset, and termination definitions.
- Interfaces for later meta-RL experiments.
- Logging utilities for comparing learned policies against fixed and context-adapted baselines.

This package should remain lightweight until Milestone 01 and Milestone 02 are stable.

## Role of ROS 2

ROS 2 provides the middleware and integration structure for the research stack:

- Nodes separate simulation, estimation, adaptation, safety, and experiment orchestration.
- Topics and services expose robot state, commands, context, and experiment control.
- Launch files make experiments reproducible.
- Bags and logs support quantitative analysis.
- Package boundaries keep the project understandable for proposal review and later implementation.

## Role of Gazebo and Isaac Sim

Gazebo is the first simulator because it is practical for ROS 2 integration, robot description validation, controller setup, and repeatable baseline experiments.

Isaac Sim is a later option, not an initial dependency. It may be useful if the project needs higher-fidelity contact behavior, richer assets, or GPU-accelerated simulation. The architecture should keep simulator-specific code inside `kuka_simulation` so research logic can remain mostly simulator-independent.

## Data Flow

Initial data flow:

1. `kuka_simulation` publishes simulated robot state and task signals.
2. `kuka_context_adapter` estimates payload/contact context from state, effort, and task feedback.
3. A baseline controller or later RL policy proposes commands.
4. `kuka_safety_layer` filters commands and enforces limits.
5. Safe commands are sent to the simulated robot controller.
6. Experiment logs capture state, context, command, safety intervention, and task outcome.

## Design Principle

Learning should not bypass physics or safety. Reinforcement learning is introduced as one command-generation method inside a ROS 2 architecture that already has simulation, context signals, and safety constraints.

