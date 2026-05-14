# kuka_task_control

`kuka_task_control` owns task-level control logic for the KUKA robot during peg-in-hole assembly.

The package sits above `joint_trajectory_controller` and below the experiment manager or learning policy. It should translate task plans into controller-compatible commands while preserving a clean interface for safety filtering.

## Research Responsibilities

- Maintain the canonical KUKA joint list: `joint_1` through `joint_6`.
- Provide trajectory generation and command publishing interfaces for baseline insertion experiments.
- Keep controller assumptions explicit, including timing, interpolation, tolerances, and command topics.
- Expose a command interface that can be filtered by `safety_layer` before reaching the robot controller.

## Boundary

This package should not own experiment sweeps, safety certificates, perception inference, or reinforcement learning policy code. It should provide deterministic control building blocks used by those packages.
