# Research Baseline v0.7 Controlled Robot Contact

Research Baseline v0.6 proved robot-generated contact. The KUKA completed the
dedicated robot contact validation sequence, Gazebo reported physical
`robot_validation` contact, force extraction was available, and
`max_contact_force` was recorded from `ros_gz_interfaces/msg/Contacts`
wrenches.

Research Baseline v0.7 keeps that validation path but makes it a controlled
low-force workflow. The robot contact sequence is slower and more staged, with
separate pre-approach, near-pad, touch-candidate, hold, and retreat phases. The
validation pad remains in the separate
`peg_in_hole_robot_contact_validation_world.sdf` scene and is positioned to
avoid deep penetration by the gripper geometry. The baseline task sequence and
full research trial launch are unchanged.

`max_contact_force` is now a validation metric, not just a recorded diagnostic.
The metrics node still extracts force from every contact sample, but positive
`/contact_event` messages are throttled to at most 5 Hz per source. Continuous
contact is counted as one contact episode when `contact_count` transitions from
zero to positive, while `contact_samples_count` still records the number of
positive contact messages.

The configured simulation validation thresholds are:

```yaml
robot_validation_warning_force_n: 50.0
robot_validation_violation_force_n: 100.0
```

These thresholds are for simulation validation only and are not human safety
limits. `/insertion_metrics` and `trial_summary.json` include
`contact_episode_count`, `max_contact_force`, `force_threshold_warning`, and
`force_threshold_violation`.

For `trial_mode=robot_contact_validation`,
`robot_contact_validation_success` is true only when:

- `task_completed == true`
- `physical_contact_observed == true`
- `force_extraction_available == true`
- `force_threshold_violation == false`
- `safety_violations_count == 0`

`safe_success` remains limited to task completion and absence of safety
violations. A run can therefore be operationally safe but fail low-force robot
contact validation if the contact force exceeds the configured violation
threshold.

This is still not final peg insertion. v0.7 validates a controlled robot-to-pad
contact measurement workflow before using contact metrics for insertion policy
evaluation.
