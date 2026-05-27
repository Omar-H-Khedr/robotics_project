# learning_interface

`learning_interface` provides the future boundary between ROS 2 simulation experiments and adaptive or reinforcement-learning methods.

The package should expose task observations, actions, rewards, resets, and safety feedback in a form that can support reproducible learning experiments without coupling the rest of the framework to a specific RL library.

## Research Responsibilities

- Define observation and action interfaces for peg-in-hole learning experiments.
- Convert task state, robot state, safety status, and trial metadata into learning-ready observations.
- Publish proposed actions through the same filtered command interface used by non-learning controllers.
- Support offline dataset generation and future policy evaluation.

## Boundary

This package should not bypass `safety_layer` or own Gazebo scene definitions. Learning policies propose actions; the research framework decides whether and how those actions are executed safely.
