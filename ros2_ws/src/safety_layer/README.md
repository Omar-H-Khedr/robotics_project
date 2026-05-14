# safety_layer

`safety_layer` owns safety filtering, constraint monitoring, and violation reporting for adaptive peg-in-hole control.

The package should sit between proposed commands and the final controller command topic. It provides a stable research interface for comparing unfiltered, filtered, and learning-generated behavior under identical task conditions.

## Research Responsibilities

- Filter or reject commands that violate configured joint, velocity, workspace, contact, or task constraints.
- Publish safety status and violation events for logging and later analysis.
- Keep safety assumptions explicit and parameterized.
- Support ablation studies by enabling, disabling, or varying individual safety constraints.

## Boundary

This package should not generate task trajectories or own experiment scheduling. It evaluates proposed actions and publishes safe commands or safety events.
