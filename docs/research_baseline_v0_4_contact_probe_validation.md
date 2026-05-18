# Research Baseline v0.4 Contact Probe Validation

Research Baseline v0.3 proves that the Gazebo contact topics for the baseline
workcell can be configured and bridged.

Research Baseline v0.4 proves that real contact messages can be observed by
adding a separate passive contact-probe validation trial. This is not robot
insertion, and it does not prove KUKA peg-hole contact. It validates the contact
instrumentation pipeline only: Gazebo contact sensor, ROS/Gazebo bridge,
`contact_metrics_node`, and `experiment_manager` logging.

The validation launch starts only
`peg_in_hole_description/worlds/peg_in_hole_contact_validation_world.sdf`, the
ROS/Gazebo bridges needed for `/clock` and contact topics, `contact_metrics_node`,
and `baseline_trial_manager`. It does not include `research_baseline.launch.py`
or `run_research_trial.launch.py`.

Gazebo contact sensors publish on fully scoped world/model/link/sensor topics,
for example
`/world/peg_in_hole_contact_validation_world/model/contact_validation_pad/link/contact_validation_pad_link/sensor/contact_validation_sensor/contact`.
The validation bridge maps those scoped Gazebo topics to clean ROS topic names:
`/gazebo/contacts/validation`, `/gazebo/contacts/peg`,
`/gazebo/contacts/hole`, and `/gazebo/contacts/target`.
`contact_metrics_node` subscribes to the clean ROS topic names.

This is not a robot-motion trial. The KUKA robot, KUKA controller stack,
`joint_state_broadcaster`, `joint_trajectory_controller`,
`controller_readiness_gate`, and `task_trajectory_executor` are not launched and
are not required.

The validation world keeps the baseline table, pedestal, peg, hole, and target
layout unchanged. It adds an isolated instrumented pad and a small passive
dynamic probe placed directly above the center of that pad. The probe is not
attached to the robot or fixed by any joint; it falls vertically under gravity
onto the pad to produce a Gazebo contact sensor event.

Run:

```bash
ros2 launch thesis_bringup run_contact_probe_validation_trial.launch.py
```

Expected summary fields after the passive probe contacts the pad:

- `trial_mode=contact_probe_validation`
- `contact_metrics_available=true`
- `contact_topics_connected` includes `validation` when the bridge publishes it
- `contact_topics_seen` includes `validation` after messages are observed
- `contact_messages_observed=true`
- `physical_contact_observed=true`
- `contact_events_count` near `1` for one continuous contact episode
- `contact_samples_count` greater than zero, and possibly large during sustained
  contact
- `task_completed=false` is acceptable for this passive validation mode
- `safe_success=null` when no safety monitor is active
- `max_contact_force=null` unless force extraction has been separately validated

`contact_events_count` counts contact episodes, not every positive contact
message. A contact episode starts on a rising edge from no contact to contact,
so a passive probe resting continuously on the validation pad should produce one
`contact_started` row in `contact_events.csv`, not thousands of rows. The
separate `contact_samples_count` field counts positive `Contacts` messages where
`contact_count > 0`; this value can be large while the probe remains in sustained
contact.

`contact_events.csv` records transition-style contact rows with:
`ros_time_sec`, `event_type`, `phase`, `source`, `contact_count`,
`max_contact_force`, and `message`. `event_type=contact_started` marks the
rising edge for an episode, and `event_type=contact_ended` marks the transition
back to no contact. Short start/end flicker is debounced by the metrics node.

`max_contact_force` is preliminary until validated. When force extraction is
explicitly enabled, the metrics node attempts a conservative magnitude from
available contact wrench force vectors and keeps the maximum observed value.
