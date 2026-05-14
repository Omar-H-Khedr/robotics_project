# Initial Research Metrics

This file defines the initial metrics for KUKA peg-in-hole assembly experiments. Metric definitions should become stricter as the simulator, contact instrumentation, safety layer, and experiment manager mature.

## Task Success

Binary trial-level outcome indicating whether the full assembly task completed within the trial constraints.

Initial definition: the robot completes the planned insertion procedure without timeout, fatal controller error, or unrecoverable safety stop.

## Insertion Success

Binary trial-level outcome indicating whether the peg reached the target insertion depth and final alignment tolerance.

Initial definition: the peg tip reaches the configured insertion depth along the task insertion axis while remaining within the configured lateral and angular tolerance.

## Collision Events

Count of collision or contact events that are outside the expected peg-hole interaction.

Initial definition: number of unexpected contacts reported during a trial. The exact Gazebo contact topic and filtering rule will be finalized when contact instrumentation is added.

## Maximum Contact Force Placeholder

Maximum measured or estimated contact force during a trial.

Initial status: placeholder metric. The value should remain explicitly marked as unavailable until contact wrench estimation or Gazebo contact-force extraction is implemented and validated.

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
