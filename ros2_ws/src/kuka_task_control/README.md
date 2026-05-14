# kuka_task_control

`kuka_task_control` owns task-level control logic for the KUKA robot during peg-in-hole assembly.

The package sits above `joint_trajectory_controller` and below the experiment manager or learning policy. It should translate task plans into controller-compatible commands while preserving a clean interface for safety filtering.

## Research Responsibilities

- Maintain the canonical KUKA joint list: `joint_1` through `joint_6`.
- Provide trajectory generation and command publishing interfaces for baseline insertion experiments.
- Keep controller assumptions explicit, including timing, interpolation, tolerances, and command topics.
- Expose a command interface that can be filtered by `safety_layer` before reaching the robot controller.
- Provide repeatable baseline trajectories that validate simulation, controller wiring, and timing before adaptive policies are introduced.

## Boundary

This package should not own experiment sweeps, safety certificates, perception inference, or reinforcement learning policy code. It should provide deterministic control building blocks used by those packages.

## Phase 1B Baseline Trajectory

Phase 1B adds a research-grade `FollowJointTrajectory` action client for the KUKA Gazebo baseline. The trajectory is configured in `config/baseline_trajectory.yaml` and sends the robot through:

- `home`
- `pre_task`
- `return_home`

The active controller must be `/joint_trajectory_controller`, with its action server available at `/joint_trajectory_controller/follow_joint_trajectory`.

## Running

Start the simulation and controllers first:

```bash
ros2 launch thesis_bringup research_baseline.launch.py
```

In another sourced terminal, run the baseline trajectory client:

```bash
ros2 launch kuka_task_control baseline_trajectory.launch.py
```

Expected behavior:

- The node loads the installed YAML trajectory specification.
- It waits for `/joint_trajectory_controller/follow_joint_trajectory`.
- It sends one full trajectory goal for `joint_1` through `joint_6`.
- The KUKA arm moves from `home` to `pre_task`, then returns to `home`.
- The node reports whether the action completed successfully or failed with the controller error code.
