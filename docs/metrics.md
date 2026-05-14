# Initial Research Metrics

This file defines the initial metrics for KUKA peg-in-hole assembly experiments. Metric definitions should become stricter as the simulator, contact instrumentation, safety layer, and experiment manager mature.

## Task Success

Binary trial-level outcome indicating whether the full assembly task completed within the trial constraints.

Initial definition: the robot completes the planned insertion procedure without timeout, fatal controller error, or unrecoverable safety stop.

## Insertion Success

Binary trial-level outcome indicating whether the peg reached the target insertion depth and final alignment tolerance.

Initial definition: the peg tip reaches the configured insertion depth along the task insertion axis while remaining within the configured lateral and angular tolerance.

Research Baseline v0.3 status: `insertion_success` remains `null`. Contact observation and task phase progress are not sufficient proof of insertion depth or alignment. `insertion_success_estimate` is also `null` until a documented heuristic is introduced.

## Collision Events

Count of collision or contact events that are outside the expected peg-hole interaction.

Initial definition: number of unexpected contacts reported during a trial. The exact Gazebo contact topic and filtering rule will be finalized when contact instrumentation is added.

Research Baseline v0.3 status: Gazebo contact sensors are configured for `/gazebo/contacts/peg`, `/gazebo/contacts/hole`, and `/gazebo/contacts/target`. The metrics node publishes `/contact_event` JSON and the experiment manager records `contact_events.csv`.

## Maximum Contact Force Placeholder

Maximum measured or estimated contact force during a trial.

Initial status: placeholder metric. The value should remain explicitly marked as unavailable until contact wrench estimation or Gazebo contact-force extraction is implemented and validated.

Research Baseline v0.3 status: `max_contact_force` remains `null` by default because force extraction is disabled/unvalidated. The metrics node does not fake force values.

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

- `contact_events_count`: number of `/contact_event` rows observed by the trial manager.
- `max_contact_force`: maximum validated contact force, or `null` when unavailable.
- `insertion_attempted`: true once an insertion phase is observed.
- `insertion_hold_reached`: true once `insertion_hold` is observed.
- `insertion_success`: true/false only after a validated rule exists; currently `null`.
- `insertion_success_estimate`: optional future heuristic field; currently `null`.
- `contact_metrics_available`: true once contact metrics are observed, false if topics/message types are missing.
- `notes`: explanation of unavailable contact force or success semantics.
