# Research Baseline v0.3 Contact Metrics

Research Baseline v0.3 adds contact and insertion metrics infrastructure to the existing KUKA Gazebo peg-in-hole workflow. It does not change the robot spawn, table, pedestal, peg, fixture, plate geometry, gripper geometry, task sequence poses, or controller behavior.

## Contact Instrumentation

The peg-in-hole SDF models now include contact sensors on the existing workcell bodies:

- `peg_contact_sensor` on `cylindrical_peg::peg_link`, monitoring `peg_collision`.
- `hole_contact_sensor` on `hole_fixture::fixture_link`, monitoring the fixture collision bars.
- `target_contact_sensor` on `target_plate::plate_link`, monitoring the target plate collision bars around the nominal opening.

The tabletop collision is named `table_collision` so later filtering can distinguish table contact from fixture or target contact.

## Contact Topics

Gazebo publishes the configured contact sensor data on:

- `/gazebo/contacts/peg`
- `/gazebo/contacts/hole`
- `/gazebo/contacts/target`

`thesis_bringup/config/contact_bridge.yaml` bridges those topics from `gz.msgs.Contacts` to `ros_gz_interfaces/msg/Contacts`. The metrics node subscribes to the ROS-side topics when `ros_gz_interfaces/msg/Contacts` is available. If contact topics or message types are missing, it logs a warning and continues publishing `/insertion_metrics` with `contact_metrics_available: false`.

## New Metrics

`peg_in_hole_metrics/contact_metrics_node` publishes:

- `/contact_event` as `std_msgs/msg/String` JSON when a non-empty contact message is observed.
- `/insertion_metrics` as `std_msgs/msg/String` JSON at 2 Hz.

`experiment_manager/baseline_trial_manager` records:

- `contact_events.csv`
- contact-related fields in `trial_summary.json`

The summary fields include `contact_events_count`, `max_contact_force`, `insertion_attempted`, `insertion_hold_reached`, `insertion_success`, `insertion_success_estimate`, `contact_metrics_available`, and `notes`.

## Task Completion vs Contact vs Insertion Success

Task completion means the scripted task sequence reached its terminal completed status.

Contact observation means Gazebo reported contact on one or more instrumented bodies. Contact alone does not prove insertion success; it can also represent table contact, rim contact, fixture contact, or incidental collisions.

True insertion success should mean the peg reached a validated insertion depth and alignment tolerance. v0.3 does not yet implement a defensible geometric or force-based success rule, so `insertion_success` remains `null`.

`insertion_success_estimate` also remains `null` in v0.3. It is reserved for a future explicitly documented heuristic if a validated success rule is still unavailable.

## Run

Recommended one-command workflow:

```bash
ros2 launch thesis_bringup run_full_research_trial.launch.py
```

This starts the existing `run_research_trial.launch.py` baseline, waits for
`/joint_trajectory_controller/follow_joint_trajectory` to become available, and
only then starts `kuka_task_control/task_trajectory_executor`. The readiness
gate prevents the task sequence from racing the Gazebo controller startup.

The old two-terminal workflow is still useful for debugging the simulator,
controllers, metrics, safety monitor, or logger before allowing robot motion.

Terminal 1:

```bash
ros2 launch thesis_bringup run_research_trial.launch.py
```

Terminal 2:

```bash
ros2 launch kuka_task_control run_task_sequence.launch.py
```

`run_research_trial.launch.py` starts Gazebo, the KUKA controller stack, contact
metrics, the safety monitor, and the experiment manager.

## Inspect

Trial outputs are written under `ros2_ws/src/experiment_manager/results/baseline_trials/<trial_id>/`.

- `contact_events.csv` contains contact event rows with ROS time, phase, source, contact count, max contact force, and message.
- `trial_summary.json` contains the trial-level v0.2 fields plus the v0.3 contact and insertion metric fields.
