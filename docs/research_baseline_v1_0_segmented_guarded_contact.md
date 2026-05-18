# Research Baseline v1.0 Segmented Guarded Contact

v0.6 and v0.7 proved that the KUKA robot could make real contact with the
dedicated validation pad and that contact force could be extracted from Gazebo
`ros_gz_interfaces/msg/Contacts.wrenches`. Those runs validated the robot contact
scene and instrumentation, but the continuous position-controlled approach could
produce high force after contact.

v0.8 and v0.9 added force and early-contact guards. The guard path worked and
the trial could finish with `final_trial_status=guarded_contact_stop`, but the
robot was still inside one long active trajectory when contact occurred. That
meant cancellation could arrive after additional penetration, so the response was
correct but still late for low-force validation.

v1.0 replaces that long approach with segmented guarded motion:

- move from `safe_home` to `robot_contact_pre_approach`,
- send one short 2-4 second approach segment,
- wait briefly for `/force_guard_status` and `/insertion_metrics`,
- stop immediately on `physical_contact_observed=true` or force above the
  configured contact threshold,
- publish `trial_status=guarded_contact_stop`,
- retreat without sending any further approach segment.

The new executor is
`kuka_task_control/segmented_contact_executor`. It sends each increment as a
separate `FollowJointTrajectory` goal to
`/joint_trajectory_controller/follow_joint_trajectory` and keeps the stable zero
trajectory header stamp used by `task_trajectory_executor`.

Configuration lives in
`kuka_task_control/config/segmented_robot_contact_approach.yaml`. The final
configured approach segment is intentionally shallower than the prior high-force
touch candidate from `robot_contact_validation_sequence.yaml`.

v1.1 keeps the successful v1.0 segments through `contact_segment_06` and adds
two slow, shallow final tuning segments, `contact_segment_07` and
`contact_segment_08`, after the stable no-contact v1.0 run. These extra segments
move only slightly farther toward the validation pad so the guarded approach can
seek first contact without returning to the prior deeper touch candidate.

The v1.1 segmented robot contact trial completed safely end to end: exactly one
KUKA was spawned, controller readiness succeeded, all approach segments through
`contact_segment_08` and retreat executed cleanly, the safety monitor reached a
terminal state without spam, and the trial summary was flushed. That run did not
reach contact; the executor reported that the segmented contact approach
completed without a guarded contact stop.

v1.2 keeps the stable v1.1 segments through `contact_segment_08` and adds four
additional slow, tiny tuning segments, `contact_segment_09` through
`contact_segment_12`. These segments move only slightly farther toward the
validation pad, with a guard check before each next approach segment and an
immediate guard refresh after each successful segment. The intended v1.2 result
is gentle first contact, `final_trial_status=guarded_contact_stop`, no force
threshold violation, and `segmented_contact_success=true`.

v1.4 records the Cartesian end-effector endpoint for segmented contact alignment.
At each completed segment, and especially at the final
`final_segment_endpoint` event, `segmented_contact_executor` attempts to record
the transform from the configured base frame to the tool frame and logs
`end_effector_position_xyz` plus `end_effector_orientation_xyzw` in `/task_event`.
The validation pad should be aligned from these Cartesian endpoint diagnostics,
not by blindly extending the joint-space contact segments.

v1.5 aligns the robot contact validation plate to the measured final tool0
endpoint instead of extending the segmented approach. The measured endpoint is
approximately world `[1.027, -0.267, 1.108]`. The validation target is now a
vertical contact plate on a dedicated grey stand, not a horizontal table pad.

v1.5 also proved that the endpoint-aligned target was still too close, too
large, or both: contact happened early at `robot_contact_pre_approach`, the trial
stopped after only two segments, `early_contact_guard_triggered=true`, and
`max_contact_force` reached about 213 N.

v1.6 keeps the segmented robot motion unchanged and refines only the validation
target. It replaces the large vertical plate with a smaller vertical contact
patch aligned near the final endpoint, keeps the patch clearly above and away
from the measured pre-approach pose, and adds visual-only markers for both the
pre-approach tool0 pose and the final endpoint.

v1.7 distinguishes intended final contact from unexpected pre-approach contact.
If contact is detected before `contact_segment_01` starts, the segmented
executor publishes `unexpected_pre_approach_contact`, reports
`trial_status=failed_pre_contact`, retreats, and does not mark segmented contact
success. Contact metrics now include collision-pair diagnostics so the trial
summary can identify whether contact came from the validation patch, visual
stand placement mistakes, table geometry, or robot geometry.

v1.7 confirmed the intended contact pair and eliminated pre-approach contact:
first contact occurred at `contact_segment_04` between
`robot_contact_validation_pad::robot_contact_validation_pad_link::robot_contact_validation_pad_collision`
and `kuka_lbr_iisy::link_6::link_6_collision`. That proved the contact target
was correct, but the static rigid validation target still allowed the position
controller to push peak force to about 194 N.

v1.8 keeps the segmented robot motion unchanged and replaces only the static
rigid validation target with a lightweight dynamic compliant bumper. The bumper
retains the existing contact sensor and topic, but can move slightly when
touched so the trial can validate low-force robot-generated contact before peg
insertion.

The launch entry points are:

- `kuka_task_control/launch/run_segmented_robot_contact_approach.launch.py`
- `thesis_bringup/launch/run_full_segmented_robot_contact_trial.launch.py`

The full trial uses `trial_mode=segmented_robot_contact_validation`. Its success
summary field is `segmented_contact_success`, which is true only when physical
contact is observed, the final trial status is `guarded_contact_stop`, no force
threshold violation is reported, and the safety monitor reports zero violations.

This remains contact validation, not final peg insertion. The purpose of v1.0 is
to prove that the robot can approach an instrumented object, detect first
contact, avoid commanding deeper penetration, retreat, and produce a clean
low-force validation summary.
