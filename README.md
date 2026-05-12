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
