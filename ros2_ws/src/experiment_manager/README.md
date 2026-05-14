# experiment_manager

`experiment_manager` owns reproducible trial orchestration, metadata, and logging for the research framework.

This package should make it possible to repeat the same peg-in-hole experiment with controlled seeds, parameters, initial conditions, controller variants, safety settings, and logging outputs.

## Research Responsibilities

- Define trial manifests and parameter sweeps.
- Record experiment metadata, software configuration, task configuration, and random seeds.
- Coordinate trial start, stop, reset, timeout, and result labeling.
- Manage logs needed for metrics, plots, and publication artifacts.

## Boundary

This package should not implement low-level robot control, safety filtering, perception inference, or learning algorithms. It coordinates those components and records their behavior.
