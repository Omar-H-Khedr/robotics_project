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

## Baseline v0.1 Trial Logging

`baseline_trial_manager` records one reproducible Gazebo baseline trial per process run. It creates a unique folder under:

```text
results/baseline_trials/
```

Each trial folder contains:

- `trial_metadata.json`
- `joint_states.csv`
- `task_events.csv`
- `safety_events.csv`
- `trial_summary.json`

The logger subscribes to `/joint_states`, `/task_phase`, and `/safety_status`. The summary includes explicit placeholders for task success, insertion success, collision events, maximum contact force, execution time, safety violations, and safe success. Contact and success metrics are intentionally not inferred in v0.1; they will be computed after contact instrumentation and task-state checks are added.

Run the logger directly with:

```bash
ros2 launch experiment_manager run_baseline_logging.launch.py
```
