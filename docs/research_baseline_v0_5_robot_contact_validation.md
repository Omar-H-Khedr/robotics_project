# Research Baseline v0.5 Robot Contact Validation

Research Baseline v0.4 validated the contact sensing and logging pipeline with a
passive Gazebo contact probe. That run confirmed that contact messages can be
observed, a real physical contact can be detected, contact episodes are counted
once, and continuous contact produces many positive contact samples.

Research Baseline v0.5 adds a separate controlled robot-to-object contact
validation workflow. The KUKA runs a slow scripted joint-space sequence toward a
dedicated `robot_contact_validation_pad` in
`peg_in_hole_robot_contact_validation_world.sdf`. The pad publishes
`/gazebo/contacts/robot_validation`, which is logged separately from the existing
`validation`, `peg`, `hole`, and `target` contact sources.

This is not final peg insertion. The normal baseline task sequence, full
research trial launch, passive contact probe validation, work table, pedestal,
peg, hole, target, and robot spawn pose remain unchanged.

The approach uses conservative joint-space poses:

- `safe_home`
- `robot_contact_pre_approach`
- `robot_contact_approach`
- `robot_contact_hold`
- `robot_contact_retreat`
- `return_home`

The contact pose may need manual tuning after Gazebo observation. If the robot
sequence completes but contact is not observed, the trial should still complete
and report that `physical_contact_observed` is false for the robot validation
contact source.

Run:

```bash
ros2 launch thesis_bringup run_full_robot_contact_validation_trial.launch.py
```
