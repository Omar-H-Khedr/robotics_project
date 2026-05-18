# Visuomotor Context-Based Meta-RL with Virtual-Force Safety for Peg-in-Hole Assembly

This folder contains ROS 2 packages for the KUKA peg-in-hole thesis framework.

## Research Packages

- `thesis_bringup`: top-level launch and experiment bringup.
- `peg_in_hole_description`: task scene, geometry, frames, worlds, and assets.
- `kuka_task_control`: task-level KUKA command interfaces targeting `joint_trajectory_controller`.
- `safety_layer`: command filtering, safety monitoring, and violation reporting.
- `experiment_manager`: reproducible trial orchestration, metadata, logging, and result labeling.
- `perception_pipeline`: future Gazebo RGB-D and task-state estimation pipeline.
- `learning_interface`: future adaptive control and reinforcement-learning interface.

## External Packages

- `external/kuka_robot_descriptions`: upstream KUKA robot description and Gazebo resources.

## Legacy/Demo Packages

The existing demo-oriented packages are kept for reference and should not be deleted:

- `first_robot_demo`
- `kuka_description`
- `robot_description`
- `robot_simulation`

New thesis development should use the research packages above as the primary architecture.
