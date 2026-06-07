# research_baseline_handoff_timeout12_gain3000_damping10_25hz_v1

Date: 2026-06-07

Purpose: validate a new diagnostic parameter hook for INSERT handoff timing and
test whether simply waiting longer can unblock strict physical-clearance
stability. The run preserved the fixed `0.0010 m` radial-clearance gate and the
fixed `8` post-command stability ticks.

Implementation under test:

- `insert_handoff_hold_duration_s` launch/node parameter, default `2.0`;
- `insert_handoff_timeout_s` launch/node parameter, default `6.0`;
- final outcome metrics now record both parameters when a fresh outcome JSON is
  written;
- the 8-tick stability requirement is not a launch argument.

Command:

```bash
timeout 220s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  insert_handoff_timeout_s:=12.0 \
  tracking_log_dir:=diagnostics/research_baseline_handoff_timeout12_gain3000_damping10_25hz_v1
```

Runtime result:

- clean single-controller Gazebo startup;
- `MOVING_TO_START` completed after about `40.08 s`;
- `APPROACH` completed with final XY about `0.0019 m`, outside the `0.0010 m`
  physical clearance, so SEARCH was entered;
- SEARCH failed closed at 45 s;
- no INSERT phase was entered, so the extended handoff timeout was not exercised
  by this trial;
- final SEARCH reason in live log: instantaneous XY was inside physical
  clearance, but it was not sustained for 8 post-command ticks;
- launch exited by the outer `timeout` during ABORT retreat, so `/tmp`
  `insertion_trial_outcome.json` remained stale and is not retained.

Passive analysis:

- SEARCH samples: `4500`;
- SEARCH mean XY: `0.001458 m`;
- SEARCH final XY in passive log: `0.001360 m`;
- SEARCH best estimated `0.0010 m` stability: `5` ticks at 25 Hz;
- SEARCH best estimated `0.0020 m` stability: `46` ticks at 25 Hz;
- hold-like command count: `7`;
- hold-like best feedback `0.0010 m` stability: `6` ticks;
- max centered-hold p95 actual XY drift: `0.002304 m`;
- max centered-hold p95 JTC joint-position error: `0.007709 rad`;
- trajectory controller-state p95 max joint-position error: `0.010576 rad`;
- max raw `|Fz|`: `100.745873 N`;
- max force norm: `169.801178 N`;
- positive Gazebo contact-topic samples: `0`.

Decision:

Keep the timing parameter hook because it enables honest future diagnostics
without weakening physical success criteria. Reject this runtime as a task
improvement: longer INSERT handoff waiting cannot matter until SEARCH first
delivers sustained physical clearance. The run improved SEARCH evidence to
`5/8` estimated 1 mm ticks and hold-like feedback to `6/8`, but still failed
closed before INSERT.

Raw CSVs are intentionally not committed.
