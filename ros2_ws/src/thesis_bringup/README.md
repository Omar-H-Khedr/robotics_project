# thesis_bringup

`thesis_bringup` is the orchestration package for the safe adaptive KUKA peg-in-hole research framework.

Its role is to provide reproducible launch entry points for the complete simulation stack: Gazebo, KUKA robot model, controllers, task scene, safety layer, experiment manager, and later perception or learning modules. Launch files in this package should be treated as experiment protocols rather than quick demos.

## Research Responsibilities

- Define canonical launch configurations for baseline, safety-filtered, perception-enabled, and learning-enabled experiments.
- Centralize high-level parameters that select robot model, Gazebo world, controller configuration, logging mode, and trial metadata.
- Keep experiment launch behavior reproducible across machines and publications.
- Avoid embedding controller logic or task logic directly in launch files; delegate those responsibilities to focused packages.

## Unified Baseline

The canonical Phase 2B entry point is:

```bash
ros2 launch thesis_bringup research_baseline.launch.py
```

## Research Baseline v0.3 Trial Workflow

Use `run_research_trial.launch.py` to start the publishable baseline runtime: Gazebo workcell, KUKA controller stack, contact metrics, monitor-only safety layer, and structured trial logger.

The recommended full research trial workflow is now one command:

```bash
ros2 launch thesis_bringup run_full_research_trial.launch.py
```

`run_full_research_trial.launch.py` includes the existing research trial
baseline, waits until
`/joint_trajectory_controller/follow_joint_trajectory` is available, and only
then starts `kuka_task_control/task_trajectory_executor`. This readiness gate
prevents the task sequence from starting before the Gazebo controller action
server exists.

The older two-terminal workflow is still supported for debugging when you want
to inspect the simulator, controllers, contact metrics, safety monitor, and
logger before starting motion manually.

Terminal 1:

```bash
ros2 launch thesis_bringup run_research_trial.launch.py
```

Terminal 2:

```bash
ros2 launch kuka_task_control run_task_sequence.launch.py
```

Research Baseline v0.3 adds `peg_in_hole_metrics/contact_metrics_node` after the baseline launch delay. The node listens to `/task_phase`, `/trial_status`, and bridged Gazebo contact topics, then publishes `/contact_event` and `/insertion_metrics`. This is instrumentation only; it does not auto-launch the task executor and does not modify robot control.

This launch file reads `config/research_baseline.yaml`, resolves `peg_in_hole_description/worlds/peg_in_hole_world.sdf`, adds the task package's `models` directory to `GZ_SIM_RESOURCE_PATH`, and starts the KUKA robot in that world. The upstream KUKA bridge remains unchanged; contact topics are bridged separately through `config/contact_bridge.yaml`.

The default KUKA spawn pose is `[x=0.80, y=-0.75, z=0.75, roll=0.0, pitch=0.0, yaw=1.5708]`. The `x=0.80` coordinate aligns the robot base with the table centerline, while `y=-0.75` keeps the base in front of the table. The table surface is `0.75 m` high; the older floor-mounted `z=0.0` spawn made the arm appear under the table even with correct x/y alignment. The research baseline is therefore pedestal-mounted at `z=0.75`, with the static `robot_pedestal` included by `peg_in_hole_world.sdf` under the KUKA base. The table remains centered at `x=0.80, y=0.0`, and the peg, target plate, and hole fixture stay at their world-defined poses.

Robot base pose and arm posture are separate controls. The launch `x/y/z/roll/pitch/yaw` arguments define where the robot base is placed in the workcell. The initial/home joint pose defines whether the arm starts parked above the table or folded down into it. The research baseline must start with no robot-table, robot-peg, robot-hole-fixture, or robot-target-plate collision.

The baseline `safe_home` joint pose is `[0.0, -0.8, 1.2, 0.0, 0.8, 0.0]`, also aliased as `home`. This raised posture is chosen specifically to avoid the initial table collision that can occur when the arm starts from an all-zero or folded-down joint state. `research_baseline.launch.py` passes `safe_home` into `kuka_gazebo/launch/gazebo_startup.launch.py`, where it becomes the LBR iisy ros2_control initial state before any task trajectory is sent.

`safe_above_table` is available as `[0.0, -0.6, 1.0, 0.0, 0.9, 0.0]`. These poses are intentionally conservative for the approximately 0.75 m table surface.

For headless startup without the Gazebo GUI:

```bash
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false
```

Startup logs identify the world, robot name/model, task frames, insertion axis, and expected controller stack. These logs are intended to make rosbag and terminal records traceable to the active experiment configuration.

## Notes

Existing demo packages remain in the workspace for reference, but this package should become the main entry point for thesis experiments.
