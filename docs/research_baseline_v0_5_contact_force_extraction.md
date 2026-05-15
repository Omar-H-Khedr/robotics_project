# Research Baseline v0.5 Contact Force Extraction

Research Baseline v0.5 validates contact-force extraction on the minimal Gazebo contact world before applying the same metrics path to KUKA peg-in-hole insertion.

## Source Message

The metrics node subscribes to bridged `ros_gz_interfaces/msg/Contacts` topics. For each incoming message, it inspects every entry in `contacts_msg.contacts`, then every `contact.wrenches` entry when present.

Each wrench can contain force vectors through:

- `body_1_wrench.force`
- `body_2_wrench.force`

The extractor also accepts closely related field shapes used by ROS/Gazebo message variants, but it does not invent or estimate force values. If no force vector is present, `max_contact_force` remains `null`.

## Force Magnitude

For every available force vector, the node computes Euclidean magnitude:

```text
force_magnitude = sqrt(x^2 + y^2 + z^2)
```

`max_contact_force` is the maximum magnitude across all contacts and all wrenches in the message. `/insertion_metrics` keeps the maximum observed value over the trial so far.

The corresponding metrics fields are:

- `max_contact_force`: maximum extracted force magnitude in newtons.
- `force_extraction_available`: true once at least one valid force vector has been parsed.
- `force_extraction_method`: `ros_gz_interfaces Contacts.wrenches force magnitude`.

## Minimal Contact Validation

The validated minimal world uses a passive 0.1 kg probe pressing on the instrumented contact pad. A static probe under gravity should report approximately:

```text
F = m * g = 0.1 kg * 9.81 m/s^2 = 0.981 N
```

The observed `body_1_wrench.force.z ~= 0.981 N` therefore matches the expected probe weight. This validates the Gazebo contact sensor, ROS bridge, and force extraction path without using KUKA task control.

## Metric Meanings

`contact_count` is the number of contacts in one received `Contacts` message. It is an instantaneous message-level count, not a trial-level success result.

`max_contact_force` is the largest extracted force-vector magnitude. In `/contact_event`, it describes that event message. In `/insertion_metrics` and `trial_summary.json`, it is the maximum observed over the trial so far.

`physical_contact_observed` becomes true after a configured physical contact source reports `contact_count > 0`. It means the contact instrumentation saw real contact, not that insertion succeeded.

`insertion_success` remains a separate task metric. It should only become true or false after a validated insertion-depth and alignment rule exists. Contact alone is not proof of peg insertion.

## Logging Path

The extracted force propagates through:

- `/contact_event` JSON as `max_contact_force`.
- `/insertion_metrics` JSON as `max_contact_force`, `force_extraction_available`, and `force_extraction_method`.
- `contact_events.csv` as `max_contact_force`.
- `trial_summary.json` as `max_contact_force`, `force_extraction_available`, and `force_extraction_method`.

Positive contact events are rate controlled during continuous contact. The node still publishes contact starts and throttled contact updates, and it increments `contact_events_count` only for positive physical contact events where `contact_count > 0`.
