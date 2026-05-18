# Research Baseline v0.1

Status: superseded by Research Baseline v0.2 for measurable trial logging. The
v0.1 Gazebo workcell, launch structure, and scripted KUKA task sequence remain
the stable baseline that v0.2 extends.

Research Baseline v0.1 is the first publishable, reproducible Gazebo-based KUKA peg-in-hole pipeline in this repository. It starts the simulation, monitors safety, logs trial data, and executes a scripted joint-space task sequence through the confirmed reliable action interface.

## What It Does

- Launches the Gazebo peg-in-hole workcell with the pedestal-mounted KUKA LBR iisy 3 R760 and simplified gripper.
- Uses `joint_trajectory_controller` through `/joint_trajectory_controller/follow_joint_trajectory`.
- Executes the scripted sequence in `kuka_task_control/config/baseline_task_sequence.yaml`.
- Publishes task phase changes on `/task_phase`.
- Monitors joint states, task phase timing, NaN/Inf values, soft joint limits, and missing joint-state timeout.
- Logs trial metadata, joint states, task events, safety events, and a summary JSON.

## How To Run

Terminal 1:

```bash
ros2 launch thesis_bringup run_research_trial.launch.py
```

Terminal 2:

```bash
ros2 launch kuka_task_control run_task_sequence.launch.py
```

The two-terminal workflow is intentional in v0.1. It lets the researcher confirm the simulator, controllers, safety monitor, and logger are alive before motion starts.

## What Is Logged

Each run of `baseline_trial_manager` creates a folder under:

```text
results/baseline_trials/
```

The folder contains:

- `trial_metadata.json`
- `joint_states.csv`
- `task_events.csv`
- `safety_events.csv`
- `trial_summary.json`

The summary includes placeholders for task success, insertion success, collision events, maximum contact force, safety violations, execution time, and safe success.

## Not Implemented Yet

- Force control.
- Controller stopping from the safety layer.
- Peg/hole contact-force metrics.
- Automatic task-success labeling.
- Perception-based peg and hole state estimation.
- Learning or adaptive policy execution.

## Why This Is Publishable And Reproducible

The baseline separates simulation bringup, task execution, safety monitoring, and logging into explicit ROS 2 packages with stable topics, YAML configuration, and structured trial outputs. The robot motion path uses the action interface that has been confirmed to move the KUKA correctly. Every trial produces metadata and event logs that can be archived, compared, and extended as contact metrics and perception are added.

## Next Milestone

The next milestone after v0.1 is Research Baseline v0.2: structured task events,
trial status, JSON safety status, reproducible trial folders, and explicit
`safe_success` summaries. See `docs/research_baseline_v0_2_metrics.md`.
