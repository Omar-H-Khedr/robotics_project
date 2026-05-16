# Research Baseline v0.8 Force-Guarded Contact

Research Baseline v0.7 proved robot-generated contact with the dedicated
`robot_contact_validation` workflow. The KUKA reached the validation pad, the
contact pipeline recorded `robot_validation` contact, and force extraction from
`ros_gz_interfaces/msg/Contacts` wrenches produced `max_contact_force`.

The v0.7 result also showed that contact was too aggressive for the intended
low-force validation: `max_contact_force` reached about 304 N, which exceeded
the configured 100 N simulation violation threshold. That made
`force_threshold_violation=true` and
`robot_contact_validation_success=false`. The likely cause is that a
position-controlled joint trajectory continued pushing into a static validation
pad after first contact.

v0.8 adds the first runtime safety response layer for this validation path. The
robot contact validation launch enables a force guard in
`task_trajectory_executor.py`. When enabled, the executor subscribes to
`/insertion_metrics`, reads `max_contact_force`, and watches the active
`FollowJointTrajectory` goal. If the measured force reaches the configured
violation threshold, the executor:

- publishes a `force_guard_triggered` `/task_event` with the force and threshold
- publishes `/trial_status=guarded_stop`
- cancels the active trajectory goal
- sends the configured retreat phase, currently `robot_contact_retreat`

The baseline task remains unchanged. `baseline_task_sequence.yaml` and
`run_full_research_trial.launch.py` do not enable the force guard, so the v0.8
guard is scoped to the dedicated robot contact validation launch.

The validation sequence is also more conservative than v0.7. The approach uses
longer 10-12 second motions, a shallower touch candidate, a 1 second hold, and a
clear retreat pose. The validation pad is moved slightly farther from the
approach direction and uses softer ODE contact parameters in the validation-only
world.

This is still simulation validation, not final peg insertion. v0.8 validates
that the system can observe excessive robot contact force at runtime, stop the
active approach, and retreat while preserving force extraction and trial summary
evidence.
