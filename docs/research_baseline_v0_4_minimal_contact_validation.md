# Research Baseline v0.4 Minimal Contact Validation

This diagnostic validates Gazebo contact sensor publishing with the smallest
possible passive scene. It is not the robot insertion test and it does not use
the KUKA robot to create contact.

The world is
`peg_in_hole_description/worlds/minimal_contact_validation_world.sdf`. It
contains only a ground plane, one light, a static `contact_validation_pad`, and a
dynamic `contact_probe` sphere. The sphere starts centered over the pad with a
small overlap, then remains governed by gravity, so Gazebo should produce a real
contact sample immediately or after a tiny settling step.

The pad link contains `contact_validation_sensor`, a Gazebo contact sensor bound
to `contact_validation_pad_collision`. Gazebo Sim may expose contact sensors
under world-scoped topic names such as
`/world/<world>/model/<model>/link/<link>/sensor/<sensor>/contact`.
For this diagnostic, the observed Gazebo contact topic is
`/world/minimal_contact_validation_world/model/contact_validation_pad/link/pad_link/sensor/contact_validation_sensor/contact`.
`run_minimal_contact_validation.launch.py` bridges that `gz.msgs.Contacts` topic
to ROS as `ros_gz_interfaces/msg/Contacts` on the same long topic name.

Run the diagnostic with:

```bash
ros2 launch thesis_bringup run_minimal_contact_validation.launch.py
```

Confirm Gazebo is publishing the contact sensor topic before debugging the ROS
side:

```bash
gz topic -l | grep contact
```

Confirm the ROS bridge is carrying `Contacts` messages:

```bash
ros2 topic echo /world/minimal_contact_validation_world/model/contact_validation_pad/link/pad_link/sensor/contact_validation_sensor/contact --once
```

Expected metrics after contact begins:

- `/contact_event` receives a `contact_started` JSON message from the
  `validation_sensor` source.
- `/insertion_metrics` reports `contact_metrics_available=true` once the bridge
  publisher is connected.
- `/insertion_metrics` reports `contact_messages_observed=true` once a
  `Contacts` message arrives.
- `/insertion_metrics` reports `physical_contact_observed=true` when
  `contact_count > 0`.
- `contact_events_count` increments for the positive contact episode.

Use this minimal world before integrating contact validation back into the full
KUKA workcell. A failure here points at Gazebo contact sensor setup, topic
bridging, or metrics handling rather than the KUKA task sequence.
