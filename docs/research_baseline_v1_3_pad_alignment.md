# Research Baseline v1.3: Pad Alignment

## Purpose

Baseline v1.2 proved that the segmented robot contact sequence is stable:
one KUKA is spawned, controllers and safety monitoring run, the executor reaches
`contact_segment_12`, retreat succeeds, and the trial completes with a flushed
summary. The remaining issue is alignment: the stable final robot endpoint does
not physically reach `robot_contact_validation_pad`.

Baseline v1.3 does not add more blind contact motion. It adds endpoint
diagnostics and a visual marker so the validation target can be moved to the
actual final segmented endpoint.

## Added Diagnostics

`segmented_contact_executor` now publishes endpoint fields on segment completion
events:

- `segment_name`
- `segment_index`
- `target_joint_positions`
- `reached_joint_positions`, when the latest `/joint_states` sample contains
  all executor joints
- `joint_position_error`, when reached positions are available

After the final approach segment, `contact_segment_12`, it also publishes:

- `event_type=final_segment_endpoint`
- `phase=contact_segment_12`
- `target_joint_positions`
- `reached_joint_positions`, when available

These fields are passive diagnostics only. They do not change motion behavior,
guard thresholds, retreat behavior, contact extraction, or robot aggressiveness.

## Visual Alignment Aid

`peg_in_hole_robot_contact_validation_world.sdf` includes
`segmented_final_endpoint_marker`, a bright visual-only marker initially placed
near `robot_contact_validation_pad`. It has no collision and is not a contact
object.

Use the marker as a Gazebo tuning reference while aligning the pad. The contact
object remains `robot_contact_validation_pad`, and contact force extraction
continues to use the pad collision and contact sensor.

## Manual Alignment Procedure

1. Run the segmented validation trial normally.
2. Inspect the `final_segment_endpoint` row in the latest `task_events.csv`.
3. Compare the final robot pose observed in Gazebo with
   `segmented_final_endpoint_marker` and `robot_contact_validation_pad`.
4. Move the validation pad and visual marker so the pad face is in front of the
   actual final end-effector location.
5. Re-run the same segmented motion and validate contact through the existing
   contact metrics and force extraction.

Do not extend `contact_segment_12` or add extra blind segments just to force
contact. The v1.3 workflow is to align the target to the measured stable endpoint
first, then validate contact with the existing guarded path.
