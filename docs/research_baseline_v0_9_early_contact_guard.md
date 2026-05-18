# Research Baseline v0.9 Early Contact Guard

v0.8 validated real robot-to-object contact, extracted contact force from
`ros_gz_interfaces/msg/Contacts.wrenches`, and proved that the force guard could
cancel the active trajectory. The observed stop was still late for low-force
validation because the controller watched `/insertion_metrics`, which is a
summary/logging topic published at low rate.

v0.9 adds an event-driven contact response layer:

- `peg_in_hole_metrics/contact_metrics_node.py` publishes `/force_guard_status`
  as `std_msgs/msg/String` JSON directly from the Gazebo contact callback.
- `/insertion_metrics` remains unchanged as the experiment logging summary.
- The new status message reports the contact source, contact count,
  `physical_contact_observed`, real extracted `max_contact_force` when present,
  force extraction availability, and warning/violation flags.
- No synthetic force values are generated; force remains `null` when no parsed
  wrench force vector exists in the contact message.

The robot contact validation world contains no robot model; the KUKA is spawned
by launch as the single `kuka_lbr_iisy` entity. The validation pad is placed on
a dedicated static stand to avoid ambiguous tabletop coordinates and prevent a
floating/off-table contact target.

For robot contact validation, `kuka_task_control/task_trajectory_executor.py`
can optionally subscribe to `/force_guard_status`. With
`early_contact_guard_enabled=true` and `stop_on_first_contact=true`, the
executor cancels the active `FollowJointTrajectory` goal as soon as the contact
callback reports first physical contact. It then publishes
`early_contact_guard_triggered`, sets `/trial_status` to
`guarded_contact_stop`, and executes the configured retreat phase when present.

The existing v0.8 force guard remains enabled as a fallback high-force stop, but
the intended v0.9 robot contact validation outcome is an early guarded contact
stop with force below the violation threshold and no safety violations.

This is not insertion control. It is an early safety response layer that proves
the stack can detect first physical contact and stop the approach before large
contact force develops.
