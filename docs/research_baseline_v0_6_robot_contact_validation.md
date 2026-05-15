# Research Baseline v0.6 Robot Contact Validation

Research Baseline v0.5 validated passive contact and force extraction. Gazebo
`Contacts` messages are received, `/contact_event` publishes contact events,
`contact_metrics_node` extracts `max_contact_force` from `Contacts.wrenches`,
and `trial_summary.json` records contact messages, physical contact, force
extraction availability, and positive maximum contact force.

Research Baseline v0.6 adds a separate robot-generated contact validation
workflow. The KUKA runs a slow, controlled joint-space sequence toward a
dedicated `robot_contact_validation_pad` in
`peg_in_hole_robot_contact_validation_world.sdf`. The validation pad is separate
from the baseline peg, hole, target, table, and pedestal layout.

This is still not final peg insertion. It is a controlled contact validation
step before an insertion policy. The baseline task sequence and stable full
research trial launch remain unchanged.

The robot validation contact sensor is expected on:

```text
/world/peg_in_hole_robot_contact_validation_world/model/robot_contact_validation_pad/link/robot_contact_validation_pad_link/sensor/robot_contact_validation_sensor/contact
```

Run:

```bash
ros2 launch thesis_bringup run_full_robot_contact_validation_trial.launch.py
```
