# Research Baseline v1.0 Segmented Guarded Contact

v0.9 proved the early contact guard path: the robot contact validation trial
observed real physical contact, extracted force from
`ros_gz_interfaces/msg/Contacts.wrenches`, triggered the early guard, and exited
cleanly with `final_trial_status=guarded_contact_stop`.

The remaining issue was contact aggression. The long position trajectory could
reach the validation pad and continue moving into contact before the guard had
enough time to cancel the active goal. In the observed v0.9 result,
`max_contact_force` was still about 208 N even though the guard triggered.

v1.0 replaces the long contact approach with a segmented guarded approach:

- send a short joint-space segment,
- wait briefly for fresh contact and force telemetry,
- decide whether to continue, stop, or retreat,
- never send another approach segment after physical contact or threshold force
  is detected.

The new `kuka_task_control/segmented_guarded_contact_executor` uses the existing
`/joint_trajectory_controller/follow_joint_trajectory` action server and keeps
the stable zero trajectory header stamp behavior. It subscribes to
`/force_guard_status` for low-latency contact guard state and `/insertion_metrics`
for summary force state. It publishes the existing `/task_phase`, `/task_event`,
and `/trial_status` topics so trial logging and force extraction remain intact.

The full v1.0 trial launch is
`thesis_bringup/launch/run_full_segmented_guarded_contact_trial.launch.py`. It
uses the same robot contact validation world and KUKA bringup pattern as the
working robot contact validation launch, then starts safety monitoring, the
baseline trial manager with `trial_mode=segmented_guarded_contact`, contact
metrics, the controller readiness gate, and finally the segmented executor.

This is not peg insertion yet. It is the first controlled low-force contact
strategy before insertion control: prove that the robot can approach an
instrumented object, detect first contact, stop further approach motion, retreat,
and report success only when contact is observed without crossing the 100 N
violation threshold.
