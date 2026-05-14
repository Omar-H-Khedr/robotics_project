# perception_pipeline

`perception_pipeline` is reserved for Gazebo RGB-D sensing, task-object state estimation, and perception outputs used by adaptive control or learning methods.

## Research Responsibilities

- Define simulated camera topics, frames, calibration assumptions, and synchronized RGB-D inputs.
- Estimate peg, hole, fixture, and contact-relevant task state from Gazebo sensor data.
- Publish perception outputs with uncertainty where possible.
- Support later comparison between ground-truth state, simulated perception, and real sensor estimates.

## Boundary

This package should not directly command the robot or schedule experiments. It publishes perception-derived state consumed by control, safety, and learning packages.
