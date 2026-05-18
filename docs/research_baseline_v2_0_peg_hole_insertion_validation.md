# Research Baseline v2.0-v2.2: Peg/Hole Insertion Validation

Research Baseline v1.8 validated low-force segmented robot contact behavior:
contact sensing, `ros_gz_bridge` `Contacts` messages, force extraction from
`Contacts.wrenches`, contact event logging, trial summaries, segmented guarded
contact response, and the compliant contact bumper.

v2.0 starts moving from generic robot contact validation toward peg/hole-specific
insertion validation. This step adds a separate insertion-validation world,
peg/hole contact topics, insertion metrics, a conservative task sequence, and a
full trial launch path. It does not modify the stable v1.8 segmented robot
contact validation behavior.

v2.0 validated that the peg/hole instrumentation path works: contact topics,
`Contacts` messages, force extraction, contact diagnostics, and trial summaries
were available. v2.1 corrects the contact semantics. A positive peg contact
sample is not automatically peg-hole insertion contact.

v2.2 cleans the initial insertion-validation world state. v2.2b uses a
suspended/static validation peg with a visible gap above the work table and away
from the hole block, so it does not rest on or fall onto the table before task
execution. Peg collision and peg/hole contact sensors remain active. A valid
instrumentation trial should start without `uninitialized` contact.

This is not final learned insertion or final grasped peg insertion. The first
v2.0 sequence is conservative and may not physically insert the peg. Its purpose
is to validate instrumentation, bridging, logging, safety monitoring, and trial
summaries before deeper insertion or learning-based policies are attempted.
Later versions will attach the peg to, or otherwise represent the peg at, the
robot/tool side.

`insertion_success` remains `null` until a validated success rule is implemented.
The metrics node does not fake insertion depth. When geometry or TF depth is not
available, `insertion_depth_available=false` and `insertion_depth_estimate=null`.

`insertion_success_estimate` is heuristic only. It can become true when
`insertion_hold_reached=true`, an actual peg-hole collision pair has been
observed, no force threshold violation occurred, and the trial status is
`completed` or the guarded low-force stop status accepted for the validation
sequence. Peg-table contact is classified as non-insertion contact, so it keeps
`peg_hole_contact_observed=false` and `insertion_success_estimate=false`.

`peg_contact_observed` means a collision pair included the peg. It does not mean
the hole was contacted. `peg_hole_contact_observed` is true only when the same
collision pair contains a peg collision and a hole or hole-block collision.
`peg_table_contact_observed` records the non-insertion case where the peg
contacts the work table.

Initial contacts are tracked separately with `initial_contact_detected`,
`initial_contact_pairs`, `uninitialized_contact_count`, and
`clean_initial_state`. If contact is observed while the task phase is still
`uninitialized`, it is a scene-cleanliness failure signal, not insertion contact.
The trial summary reports `clean_scene_success` for peg/hole insertion
validation only when the scene starts cleanly, contact topics are connected,
insertion metrics are received, and no safety violations are recorded.

Run the v2.0 validation trial with:

```bash
ros2 launch thesis_bringup run_full_peg_hole_insertion_validation_trial.launch.py
```
