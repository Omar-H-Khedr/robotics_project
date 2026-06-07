# research_baseline_search_damping10_gain3000_25hz_v1

Date: 2026-06-07

Purpose: test whether stronger converted SDF joint damping can stabilize the
25 Hz near-hole SEARCH / INSERT handoff behavior without loosening the physical
25 mm peg / 27 mm hole radial-clearance gate (`0.0010 m`).

Command:

```bash
timeout 180s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  tracking_log_dir:=diagnostics/research_baseline_search_damping10_gain3000_25hz_v1
```

Runtime result:

- headless Gazebo launched with a single research `gz_ros2_control` plugin;
- joint damping was doubled relative to the previous 5x damping line;
- `MOVING_TO_START` completed, but slowly, after about `40.08 s`;
- `APPROACH` completed with pre-insertion XY about `0.0003 m`;
- `INSERT` entered and exercised the reordered handoff logic;
- the final descent command was not sent;
- the controller aborted safely after the handoff feedback failed to remain
  within `0.0010 m` for the required `8` ticks;
- the outer `timeout 180s` ended the launch during ABORT retreat, so no fresh
  final outcome JSON was retained.

Passive analysis:

- INSERT duration in the observer log: `6.03 s`;
- INSERT final XY: `0.002123 m`;
- INSERT best estimated `0.0010 m` stability: `4` ticks at 25 Hz;
- INSERT best estimated `0.0020 m` stability: `56` ticks at 25 Hz;
- hold-like best feedback `0.0010 m` stability: `4` ticks;
- max centered-hold p95 actual XY drift: `0.002130 m`;
- max centered-hold p95 JTC joint-position error: `0.007546 rad`;
- trajectory controller-state p95 max joint-position error: `0.012033 rad`;
- max raw `|Fz|`: `102.212636 N`;
- max force norm: `170.998336 N`;
- positive Gazebo contact-topic samples: `0`.

Decision:

This is a useful failed diagnostic, not insertion success. Damping scale `10.0`
is rejected as a default because it still does not satisfy the strict 8-tick
physical-clearance handoff gate and it slows `MOVING_TO_START` substantially.
The result does confirm the 2026-06-07 handoff gate ordering fix: INSERT now
waits in the no-descent handoff settle path instead of immediately applying the
pre-depth no-contact descent abort.

Retained evidence:

- `xy_stability_analysis.md` / `.json`
- `hold_window_reference_analysis.md` / `.json`
- `above_hole_hold_analysis.md` / `.json`
- `endpoint_hold_dynamics_analysis.md` / `.json`
- `search_tracking_sensitivity_analysis.md` / `.json`
- passive observer summaries

Raw CSVs are intentionally not committed.
