# research_baseline_trajectory_command_capture_v1

Status: validated instrumentation fix.

This milestone fixes intermittent retreat-only trajectory logs by adding a
bounded first-command discovery wait in `admittance_insertion_node`. Before the
first trajectory publication, the node now waits up to `2.0 s` for the expected
trajectory-topic subscribers. The default expectation is `2` subscribers: the
active `joint_trajectory_controller` plus the passive
`trajectory_tracking_observer`. If fewer subscribers are discovered before the
timeout, the task logs a warning and continues, so observer availability cannot
block robot motion.

The change does not alter:

- robot target poses;
- trajectory waypoints or durations;
- gains, damping, or effort limits;
- force/contact safety gates;
- the strict 2 mm no-contact above-hole hold gate;
- success/failure criteria.

## Validation

Syntax and build:

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
```

Short headless command-capture run:

```bash
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_trajectory_command_capture_v1
```

The run was intentionally stopped by a 55 s wrapper before task completion. It
is not insertion evidence.

## Evidence

Runtime log:

- `Trajectory topic discovery satisfied: 2/2 subscribers matched.`
- `MOVING_TO_START: trajectory published. duration=40.0s, waypoints=20`
- `trajectory_tracking_observer`: `Observed trajectory command: points=20, duration=40.000s`

Tracking summary:

- observed commands: `1`
- samples: `5602`
- p95 max absolute joint error: `0.027239 rad`
- final max absolute joint error: `0.023885 rad`

Selector-based command attribution:

- result: `OK`
- command index: `0`
- command duration: `40.0 s`
- command target: `[0.520000, -0.200000, 0.885001] m`
- final XY error in the partial run: `0.203393 m`

The large final XY error is expected because the timeout wrapper stopped the
run around the middle of the 120 s `MOVING_TO_START` window. The relevant result
is that the first axis-align command is now captured and attributable.
