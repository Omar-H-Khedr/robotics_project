# Initial Research Metrics

This file defines the initial metrics for KUKA peg-in-hole assembly experiments. Metric definitions should become stricter as the simulator, contact instrumentation, safety layer, and experiment manager mature.

## Task Success

Binary trial-level outcome indicating whether the full assembly task completed within the trial constraints.

Initial definition: the robot completes the planned insertion procedure without timeout, fatal controller error, or unrecoverable safety stop.

## Insertion Success

Binary trial-level outcome indicating whether the peg reached the target insertion depth and final alignment tolerance.

Initial definition: the peg tip reaches the configured insertion depth along the task insertion axis while remaining within the configured lateral and angular tolerance.

Research Baseline v0.3 status: `insertion_success` remains `null`. Contact observation and task phase progress are not sufficient proof of insertion depth or alignment. `insertion_success_estimate` is a heuristic that can be true only when `insertion_hold_reached` is true, `trial_status` is `completed`, and no explicit failure status was observed.

## Collision Events

Count of collision or contact events that are outside the expected peg-hole interaction.

Initial definition: number of unexpected contacts reported during a trial. The exact Gazebo contact topic and filtering rule will be finalized when contact instrumentation is added.

Research Baseline v0.3 status: Gazebo contact sensors are configured for `/gazebo/contacts/peg`, `/gazebo/contacts/hole`, and `/gazebo/contacts/target`. The metrics node publishes `/contact_event` JSON and the experiment manager records `contact_events.csv`. `contact_metrics_available` means contact instrumentation is connected: at least one configured ROS contact topic has a visible ROS publisher. `contact_messages_observed` means at least one ROS `Contacts` message was received. `physical_contact_observed` means at least one received message contained `contact_count > 0`. `contact_events_count` is the number of positive physical contact events. It can remain zero if the scripted trajectory completes without physically touching the peg, hole, or target contact sensors.

Research Baseline v0.5 status: `contact_events_count` is counted only for positive physical contact events where `contact_count > 0`. Continuous contact is rate controlled with start and update events so `/contact_event` does not flood while still recording meaningful positive contact observations.

Zero contact events are expected for the current scripted joint-space sequence unless that sequence physically touches instrumented objects.

## Maximum Contact Force Placeholder

Maximum measured or estimated contact force during a trial.

Initial status: placeholder metric. The value should remain explicitly marked as unavailable until contact wrench estimation or Gazebo contact-force extraction is implemented and validated.

Research Baseline v0.3 status: `max_contact_force` remains `null` by default because force extraction is disabled/unvalidated. The metrics node does not fake force values.

Research Baseline v0.5 status: `max_contact_force` is extracted from `ros_gz_interfaces/msg/Contacts.wrenches` force vectors. For each available `body_1_wrench.force` and `body_2_wrench.force`, the node computes `sqrt(x^2 + y^2 + z^2)` and tracks the maximum across all contacts and wrenches. If no force vector exists in a message, the value remains `null`; no force values are faked. `force_extraction_available` becomes true after a valid force vector is parsed, and `force_extraction_method` is `ros_gz_interfaces Contacts.wrenches force magnitude`.

## Trajectory Execution Time

Elapsed time from accepted trajectory start to trajectory completion, abort, timeout, or safety stop.

Initial definition: wall-clock ROS time between trial command start and terminal trial state.

## Safety Violations

Count and type of safety constraints violated during a trial.

Initial definition: number of safety events reported by `safety_layer`, grouped by constraint category such as joint limit, velocity limit, workspace limit, contact limit, or command validity.

## Repeatability Over N Trials

Consistency of outcomes and trajectories over a fixed number of repeated trials under the same configuration.

Initial definition: report success statistics, execution-time statistics, final pose error statistics, and safety-violation statistics over N repeated trials.

## Safe Success Rate

Fraction of trials that succeed without safety violations.

Initial definition:

```text
safe_success_rate = successful_trials_without_safety_violations / total_trials
```

This metric should be reported with the trial count, scene configuration, controller configuration, and safety-layer configuration.

## Baseline v0.1 Logged Fields

Research Baseline v0.1 writes the following trial summary fields:

- `task_success`: placeholder, not inferred yet.
- `insertion_success`: placeholder, not inferred yet.
- `collision_events`: placeholder until Gazebo contact filtering is added.
- `max_contact_force`: placeholder until contact-force extraction is validated.
- `safety_violations`: counted from `/safety_status` messages whose level is `VIOLATION`.
- `execution_time_sec`: elapsed logger runtime for the trial process.
- `safe_success`: placeholder until task success and safety violation semantics are combined.

## Baseline v0.3 Contact Fields

Research Baseline v0.3 preserves the v0.2 summary fields and adds:

- `contact_events_count`: number of non-empty contact observations reported through `/contact_event`.
- `max_contact_force`: maximum validated contact force, or `null` when unavailable.
- `insertion_attempted`: true once an insertion phase is observed.
- `insertion_hold_reached`: true once `insertion_hold` is observed.
- `insertion_success`: true/false only after a validated rule exists; currently `null`.
- `insertion_success_estimate`: heuristic based on insertion hold, completed trial status, and absence of explicit failure.
- `contact_topics_configured`: configured contact source names and ROS topic names.
- `contact_topics_connected`: configured contact sources whose ROS topics currently have publishers.
- `contact_messages_observed`: true once at least one ROS `Contacts` message callback is received.
- `physical_contact_observed`: true once at least one received contact message has `contact_count > 0`.
- `contact_metrics_available`: true when at least one configured ROS contact topic has a visible publisher.
- `contact_topics_seen`: contact sources that have produced at least one message.
- `notes`: explanation of unavailable contact force or success semantics.

`task_completed`, `insertion_hold_reached`, `insertion_success_estimate`, and true `insertion_success` are intentionally separate. A trial can complete the scripted trajectory and reach insertion hold without proving the peg reached a validated insertion depth or alignment tolerance.

## Baseline v0.5 Contact Force Fields

Research Baseline v0.5 preserves the v0.3 contact fields and adds validated force extraction:

- `max_contact_force`: maximum extracted contact force magnitude in newtons, or `null` until a force vector is observed.
- `force_extraction_available`: true once a valid force vector has been extracted.
- `force_extraction_method`: extraction method string for reproducibility.
- `contact_events.csv`: includes `ros_time_sec`, `phase`, `source`, `contact_count`, `max_contact_force`, and `message` for each logged contact event.

The minimal contact validation world is the validation source for v0.5. A 0.1 kg passive probe should report approximately `0.1 * 9.81 = 0.981 N`, matching the observed Gazebo contact wrench before applying the same extraction path to KUKA insertion experiments.
