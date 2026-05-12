# Project Roadmap

## Objective

Build a practical research demonstration for contact and payload adaptation on a single KUKA LBR iisy robot in ROS 2 simulation. The roadmap is staged so each milestone produces a testable result suitable for PhD proposal discussion.

## Milestone 01: Simulation Baseline

Goal: create a reproducible single-arm simulation baseline.

Expected outcomes:

- KUKA LBR iisy robot description package placeholder is replaced with URDF/Xacro, meshes, joint limits, and inertial parameters.
- Simulation package launches the robot in Gazebo first.
- Basic ROS 2 control interfaces are defined for joint state feedback and command input.
- A simple scripted motion task verifies that the simulated robot can move safely in free space.
- Baseline logs capture joint position, velocity, effort, command, and task status.

This is the first practical milestone because every later research claim depends on a stable, measurable simulation pipeline.

## Milestone 02: Contact and Payload Adaptation

Goal: reproduce controller degradation under changed physical context, then introduce context-based adaptation.

Expected outcomes:

- Add payload variants with different mass and center of mass assumptions.
- Add contact scenarios such as surface approach, light pressing, sliding, or constrained motion.
- Define measurable failure modes: tracking error, excessive contact force, oscillation, instability, or task failure.
- Implement a context adapter that selects or adjusts controller parameters based on estimated payload/contact context.
- Compare fixed-controller behavior against context-adapted behavior.

## Milestone 03: Meta-RL Prototype

Goal: introduce learning after the deterministic simulation and safety infrastructure are reliable.

Expected outcomes:

- Wrap selected simulation tasks as an RL environment.
- Define observations that include robot state, task state, and context features.
- Define actions that are compatible with the safety layer, such as target offsets, impedance parameters, or bounded velocity commands.
- Train or evaluate a small meta-RL prototype across payload/contact variations.
- Compare adaptation speed and robustness against non-learning baselines.

## Later Work

Later work can expand the simulator and research scope:

- Improve contact realism and domain randomization.
- Evaluate Isaac Sim as a higher-fidelity alternative or complement to Gazebo.
- Add real-robot deployment only after simulation behavior, safety checks, and experiment definitions are mature.
- Extend from single-task adaptation to multi-task manipulation.

## Practical Rule

The project should always preserve a runnable baseline. New research components should be added behind clear ROS 2 interfaces so the baseline can be tested independently from learning code.

