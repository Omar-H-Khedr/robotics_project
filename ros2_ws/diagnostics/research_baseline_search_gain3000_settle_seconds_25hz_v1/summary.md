# research_baseline_search_gain3000_settle_seconds_25hz_v1

Date: 2026-06-07

Purpose: retest the previously promising `position_gain:=3000.0`,
`position_derivative_gain:=10.0` 25 Hz controller setting after the
seconds-based SEARCH settling fix.

Command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 180s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=5.0 \
  inject_velocity_state:=true \
  tracking_log_dir:=diagnostics/research_baseline_search_gain3000_settle_seconds_25hz_v1
```

Runtime outcome:

- Correct iisy6 launch path and single research `gz_ros2_control` plugin
  startup.
- `MOVING_TO_START` and `APPROACH` completed.
- The task entered `INSERT` directly because approach finished inside physical
  clearance.
- The run reported `ABORTED`, not success.
- Abort reason: no-contact INSERT XY error `0.0012 m` exceeded the physical
  radial clearance `0.0010 m` before meaningful insertion depth for 3 ticks.
- Insertion depth: `0.0000 m`.
- Peak raw `|Fz|`: `128.62 N`; peak force norm: `207.42 N`.
- Max INSERT contact force: `33.19 N`.
- The outer `timeout` still terminated the launch after the task reached
  `DONE`, so the task outcome is valid but process shutdown remains clumsy.

Passive evidence:

- `xy_stability_analysis.md`: INSERT lasted `0.109 s`; INSERT best estimated
  1 mm window was `1` tick and best 2 mm window was `1` tick.
- `hold_window_reference_analysis.md`: the INSERT handoff hold was the only
  hold-like command and achieved only `1` estimated feedback tick inside
  physical clearance.
- `search_tracking_sensitivity_analysis.md`: centered hold p95 actual XY drift
  reached `0.003254 m`, and linearized drift matched within about
  `0.000016 m`; measured joint tracking error remains sufficient to explain
  the millimeter-scale peg-tip drift.
- Controller-state p95 max joint-position error: `0.012452 rad`.

Decision:

This tuning is rejected as a default. It reached INSERT once, but only because
APPROACH happened to finish inside clearance; it then failed the strict
no-contact physical-clearance gate before meaningful depth. The run exposed a
state-machine sequencing issue: the no-contact INSERT gate could fire before
the configured handoff settle window had a chance to prove stability.
