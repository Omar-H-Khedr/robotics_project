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

## Baseline v0.1 Safety Monitor

`safety_monitor` is the first monitor-only safety layer for the Gazebo KUKA peg-in-hole baseline. It subscribes to:

- `/joint_states`
- `/task_phase`

It publishes human-readable status on:

```text
/safety_status
```

The monitor checks configured joint soft limits for `joint_1` through `joint_6`, NaN/Inf joint values, missing joint-state timeout, and a monitor-only task phase duration timeout. It logs status levels as `OK`, `WARNING`, or `VIOLATION`. It does not command the robot, stop the controller, or implement force control in v0.1.

Run it directly with:

```bash
ros2 launch safety_layer safety_monitor.launch.py
```

The default limits are installed from `config/safety_limits.yaml`.
