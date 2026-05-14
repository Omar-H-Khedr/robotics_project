# Peg-in-Hole Metrics

`peg_in_hole_metrics/contact_metrics_node` converts task phase, trial status,
and Gazebo contact sensor messages into trial-level insertion metrics.

## Contact Topics

The node subscribes to the ROS-side Gazebo contact topics:

- `/gazebo/contacts/peg`
- `/gazebo/contacts/hole`
- `/gazebo/contacts/target`

These topics are expected to use `ros_gz_interfaces/msg/Contacts` after the
Gazebo bridge. If `ros_gz_interfaces/msg/Contacts` is unavailable, the node logs
a warning, keeps running, and continues publishing `/insertion_metrics`.

## Published Topics

- `/contact_event`: `std_msgs/msg/String` JSON. A message is published whenever
  a contact message is received. Zero-contact messages are throttled; non-empty
  contacts are always published.
- `/insertion_metrics`: `std_msgs/msg/String` JSON with the current task phase,
  trial status, insertion phase flags, contact counts, topic availability, and
  conservative insertion-success fields.

`contact_metrics_available=true` means at least one real contact topic message
was observed. It does not mean physical insertion succeeded.

`max_contact_force` remains `null` by default because force extraction from
Gazebo contact wrenches has not been validated. The node can parse common force
vector fields when explicitly enabled, but force values should not be used as a
research metric until that validation is complete.

`insertion_success` remains `null` until a validated geometric or contact-based
success rule exists. `insertion_success_estimate` is only a heuristic based on
reaching `insertion_hold`, receiving `trial_status=completed`, and seeing no
explicit failure status.
