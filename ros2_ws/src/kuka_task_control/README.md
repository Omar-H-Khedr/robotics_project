# kuka_task_control

`kuka_task_control` owns task-level control logic for the KUKA robot during peg-in-hole assembly.

The package sits above `joint_trajectory_controller` and below the experiment manager or learning policy. It should translate task plans into controller-compatible action goals while preserving a clean interface for safety filtering.

## Research Responsibilities

- Maintain the canonical KUKA joint list: `joint_1` through `joint_6`.
- Provide trajectory generation and action-client command interfaces for baseline insertion experiments.
- Keep controller assumptions explicit, including timing, interpolation, tolerances, and command topics.
- Expose a command interface that can be filtered by `safety_layer` before reaching the robot controller.
- Provide repeatable baseline trajectories that validate simulation, controller wiring, and timing before adaptive policies are introduced.

## Boundary

This package should not own experiment sweeps, safety certificates, perception inference, or reinforcement learning policy code. It should provide deterministic control building blocks used by those packages.

## Research Baseline v0.1 Task-Level Controller

`task_trajectory_executor` is the baseline task-level controller for the peg-in-hole workcell. It sends each named joint-space pose as a `control_msgs/action/FollowJointTrajectory` goal to:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

It loads named poses from `config/baseline_task_sequence.yaml` and executes them in this order:

- `safe_home`
- `observe_scene`
- `pre_grasp`
- `grasp_approach`
- `lift_clearance`
- `pre_insert`
- `insertion_approach`
- `insertion_hold`
- `retreat`
- `return_home`

This executor deliberately uses joint-space named poses only. It does not perform inverse kinematics, gripper actuation, or contact feedback. Each pose contains six KUKA joint values in the canonical order, a conservative `duration_sec`, a `description`, and a `safety_tag`. The node waits for the action server, sends one pose goal at a time, waits for each result, logs accepted/rejected/succeeded/failed outcomes, publishes the current task phase on `/task_phase`, and stops the sequence if any goal is rejected or fails.

Launch the research baseline first:

```bash
ros2 launch thesis_bringup research_baseline.launch.py
```

Then, in another sourced terminal, run the task sequence:

```bash
ros2 launch kuka_task_control run_task_sequence.launch.py
```

Expected behavior:

- The node loads the installed `baseline_task_sequence.yaml` file.
- It waits for `/joint_trajectory_controller/follow_joint_trajectory`.
- It sends the required task poses sequentially as `FollowJointTrajectory` goals.
- It waits for the controller result after each pose.
- It logs whether each pose succeeded or failed.
- It publishes task phase updates on `/task_phase`.
- It exits with failure if a pose goal is rejected or the controller reports an error.

## Baseline Joint Sequence Executor

The first research baseline control layer is `baseline_joint_sequence_executor`. It loads named joint-space task poses from `config/baseline_task_poses.yaml`, validates that every pose contains exactly six KUKA joint values, and sends one complete `control_msgs/action/FollowJointTrajectory` goal to:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

The canonical joint order is fixed in the node:

```text
joint_1, joint_2, joint_3, joint_4, joint_5, joint_6
```

The required task poses are:

- `home`
- `safe_above_table`
- `observe_scene`
- `pre_task`
- `approach_workspace`
- `retreat`

Each pose has `positions` and `duration_sec`. The duration is interpreted as the slow segment time from the previous waypoint; the executor converts those segment durations into cumulative `time_from_start` values for the action goal trajectory.

## Running The Research Baseline

Start the Gazebo research scene and ros2_control controllers first:

```bash
ros2 launch thesis_bringup research_baseline.launch.py
```

In another sourced terminal, run the baseline joint sequence:

```bash
ros2 launch kuka_task_control run_baseline_sequence.launch.py
```

Expected behavior:

- The node loads the installed `baseline_task_poses.yaml` file.
- It waits for `/joint_trajectory_controller/follow_joint_trajectory`.
- It logs each configured pose before sending the full sequence.
- It sends one complete joint trajectory goal for `joint_1` through `joint_6`.
- It logs whether the controller accepted the goal.
- It waits for the controller result, reports the final action status and controller result code, and exits cleanly.

## Editing Task Poses

Edit `config/baseline_task_sequence.yaml` before rebuilding or reinstalling the workspace. Keep every required pose name present, and keep every `positions` list at exactly six numeric joint values in the canonical joint order.

The robot base pose is controlled by the Gazebo launch `x/y/z/roll/pitch/yaw` arguments in `thesis_bringup`; task-control poses only control the arm joints. For the research baseline, `safe_home` and its `home` alias must be raised parked poses that start clear of the approximately 0.75 m table, and no initial collision with the table, target plate, hole fixture, or peg is allowed.

Example pose entry:

```yaml
pre_grasp:
  positions: [0.25, -0.6, 1.0, 0.0, 0.9, 0.0]
  duration_sec: 5.0
  description: Staging posture before the peg grasp approach.
  safety_tag: approach_region
```

Use conservative timing while calibrating the research baseline. Increase `duration_sec` values when validating new poses near the table, target plate, hole fixture, or peg.

## Phase 1B Action Baseline

Phase 1B adds a research-grade `FollowJointTrajectory` action client for the KUKA Gazebo baseline. The trajectory is configured in `config/baseline_trajectory.yaml` and sends the robot through:

- `home`
- `pre_task`
- `return_home`

The active controller must be `/joint_trajectory_controller`, with its action server available at `/joint_trajectory_controller/follow_joint_trajectory`.

## Running The Action Client

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
