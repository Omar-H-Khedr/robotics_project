# Problem Statement

## Project Title

Physics-guided Context-Based Meta-RL for Contact/Payload Adaptation on a Single KUKA LBR iisy Robot using ROS 2 Simulation.

## Research Problem

Industrial and collaborative robot arms are usually controlled with fixed models, fixed gains, and task-specific assumptions about the tool, payload, contact surface, and environment. These assumptions break down when the robot handles different payloads, touches surfaces with uncertain stiffness or friction, or transitions between free-space motion and contact-rich manipulation.

This project studies how a single KUKA LBR iisy robot can adapt its behavior in simulation when payload and contact conditions change. The central research question is:

How can a ROS 2-based robot control stack combine physics-guided modeling, online context estimation, safety constraints, and later meta-reinforcement learning to adapt to changing payload and contact conditions without requiring a new hand-tuned controller for every case?

## Why Fixed Controllers Fail

Fixed robot controllers can perform well when the robot, object, and environment match the design assumptions. They become fragile when the assumptions change:

- Payload variation changes the effective inertia, gravity compensation requirement, and joint torque demand.
- Contact variation changes force response, stability margins, and the relationship between commanded motion and actual motion.
- Friction and compliance uncertainty can cause tracking error, oscillation, excessive contact force, or conservative motion.
- A controller tuned for free-space motion may be too aggressive in contact, while a controller tuned for contact may be unnecessarily slow in free space.
- Manual retuning does not scale when the robot must handle many tools, objects, or surfaces.

For a PhD proposal demonstration, the goal is not to solve all real-world contact manipulation immediately. The goal is to build a credible simulation pipeline where these failure modes can be reproduced, measured, and addressed in staged milestones.

## Proposed Direction

The proposed direction is a physics-guided context-based adaptation framework:

- Physics-guided modeling provides structured assumptions about mass, inertia, contact stiffness, damping, friction, and force limits.
- Context estimation summarizes the current interaction condition, such as payload class, contact state, or estimated environment compliance.
- A context adapter modifies controller parameters, references, or policy inputs based on the estimated context.
- A safety layer enforces hard limits on joint position, velocity, torque, workspace, and contact force before commands reach the simulated robot.
- Reinforcement learning is introduced later, after the baseline simulation and safety interfaces are stable.

## Scope of the Initial Project

The first version targets simulation only. It should establish a clean ROS 2 architecture around one KUKA LBR iisy robot, with placeholder packages for description, simulation, context adaptation, safety, and reinforcement learning environment integration.

The existing sandbox differential-drive demo remains in the repository for reference and should not be extended as part of this research track.

