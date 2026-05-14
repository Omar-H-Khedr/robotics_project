# Research Baseline v0.2 Metrics

Research Baseline v0.2 upgrades the working Gazebo KUKA peg-in-hole baseline
from repeatable motion to measurable trial execution. It keeps the same workcell,
robot placement, pedestal, simplified gripper, peg, hole fixture, controller, and
two-terminal workflow from v0.1.

## How To Run

Terminal 1 starts Gazebo, the KUKA workcell, the safety monitor, and the trial
logger:

```bash
ros2 launch thesis_bringup run_research_trial.launch.py
```

Terminal 2 starts the scripted motion sequence:

```bash
ros2 launch kuka_task_control run_task_sequence.launch.py
```

The task executor still uses the `FollowJointTrajectory` action server:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

## Topics

- `/joint_states` (`sensor_msgs/msg/JointState`): robot joint positions sampled
  into the trial log.
- `/task_phase` (`std_msgs/msg/String`): current task phase, published at 2 Hz.
- `/task_event` (`std_msgs/msg/String`): JSON task events from the executor.
- `/trial_status` (`std_msgs/msg/String`): one of `idle`, `running`,
  `completed`, or `failed`.
- `/safety_status` (`std_msgs/msg/String`): JSON safety monitor status.

Task event JSON fields:

- `timestamp_ros_sec`
- `event_type`
- `phase`
- `pose_index`
- `total_poses`
- `safety_tag`
- `message`

Safety status JSON fields:

- `timestamp_ros_sec`
- `level`
- `code`
- `phase`
- `message`

## Logged Files

Each run of `baseline_trial_manager` creates a trial folder:

```text
ros2_ws/src/experiment_manager/results/baseline_trials/trial_YYYYMMDD_HHMMSS/
```

The folder contains:

- `trial_metadata.json`: reproducibility metadata for the run.
- `joint_states.csv`: one row per joint-state sample with `ros_time_sec` and
  `joint_1` through `joint_6`.
- `task_events.csv`: structured task events such as `sequence_started`,
  `phase_started`, `goal_sent`, `goal_accepted`, `phase_succeeded`, and
  `sequence_completed`.
- `safety_events.csv`: structured safety status rows with level, code, phase,
  and message.
- `trial_summary.json`: continuously refreshed summary metrics and final state.

## Metrics

- `task_started`: true after a `sequence_started` task event.
- `task_completed`: true after `/trial_status` reports `completed` or a
  `sequence_completed` task event is received.
- `trial_failed`: true after `/trial_status` reports `failed` or a
  `sequence_failed` task event is received.
- `final_trial_status`: latest `/trial_status` value.
- `final_task_phase`: latest `/task_phase` or task-event phase.
- `completed_phases_count`: number of `phase_succeeded` events.
- `total_task_events`: number of parsed task events recorded.
- `safety_warnings_count`: number of safety events with level `WARNING`.
- `safety_violations_count`: number of safety events with level `VIOLATION`.
- `execution_time_sec`: elapsed ROS time from logger startup.
- `safe_success`: true only when `task_completed == true` and
  `safety_violations_count == 0`.

## Phase 3 Placeholders

Contact and insertion metrics are not implemented in v0.2. The summary keeps
these fields as `null` so downstream analysis scripts can depend on a stable
schema:

- `insertion_success`
- `max_contact_force`
- `contact_events_count`

Not implemented yet:

- true peg insertion success
- force/contact metrics
- perception-based localization

## Next Milestone

The next milestone is contact and insertion metrics in Gazebo: validated contact
extraction, expected insertion-contact classification, insertion-depth or
alignment metrics, and then perception-based localization.
