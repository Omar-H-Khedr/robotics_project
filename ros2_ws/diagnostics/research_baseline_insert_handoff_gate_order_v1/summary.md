# research_baseline_insert_handoff_gate_order_v1

Date: 2026-06-07

Purpose: validate a focused INSERT sequencing fix. The no-contact pre-depth
clearance abort now applies after the final descent command has started, not
during the configured INSERT handoff hold. Side-load-at-depth and force aborts
remain active before descent.

Code change under test:

- In `AdmittanceInsertionNode._handle_insert`, broad XY precondition,
  side-load-at-depth, and force checks still run before descent.
- If the final descent command has not been sent, `_handle_insert_handoff_settle`
  is allowed to run until it either proves `8` stable ticks inside the
  `0.0010 m` physical clearance or times out.
- The no-contact pre-depth clearance abort is evaluated only after the descent
  command has started.

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
  tracking_log_dir:=diagnostics/research_baseline_insert_handoff_gate_order_v1
```

Runtime outcome:

- Correct iisy6 launch path and single research `gz_ros2_control` plugin
  startup.
- `MOVING_TO_START` and `APPROACH` completed.
- APPROACH ended at `0.0019 m` XY error, so the task entered SEARCH instead of
  INSERT.
- The outer timeout ended the launch during SEARCH; no task outcome JSON was
  produced for this run.
- This run did not exercise the new INSERT handoff ordering.
- No insertion success is claimed.

Passive evidence:

- `xy_stability_analysis.md`: SEARCH best estimated 1 mm window was `3` ticks
  (`0.12 s`), best 2 mm window was `7` ticks (`0.28 s`), and final SEARCH XY
  in the partial log was `0.001886 m`.
- `hold_window_reference_analysis.md`: hold-like best feedback 1 mm window was
  `4` ticks.
- `search_tracking_sensitivity_analysis.md`: max centered-hold p95 actual XY
  drift was `0.003940 m`; linearized drift matched within about `0.000020 m`;
  dominant p95 contribution remained `joint_1`.
- `trajectory_tracking_summary.md`: controller-state p95 max joint-position
  error was `0.011439 rad`.
- `wrench_state_summary.md`: peak raw `|Fz|` was `130.294881 N`, and peak force
  norm was `206.332843 N`.

Decision:

Keep the sequencing fix because it aligns the state machine with the existing
handoff safety design and does not relax the physical clearance gate. The
runtime blocker remains no-contact SEARCH/hold feedback stability, not an
INSERT success. More validation is required before claiming any physical
peg-in-hole insertion success.
