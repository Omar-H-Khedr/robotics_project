# INSERT Pre-Depth Recenter 500 Hz Diagnostic

Date: 2026-06-07

Milestone: `research_baseline_insert_predepth_recenter_500hz_v1`

## Command

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 560s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml \
  search_recenter_duration_s:=8.0 \
  search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 \
  tracking_log_dir:=diagnostics/research_baseline_insert_predepth_recenter_500hz_v1
```

The selected package build/install trees were cleaned and rebuilt before this
run, and the installed launch path was checked to use
`lbr_iisy6_r1300_research_gripper.urdf.xacro`, model `lbr_iisy6_r1300`, and
spawn z `0.735`.

## Source Change Under Test

`admittance_insertion_node.py` now treats repeated no-contact XY drift before
meaningful insertion depth as a bounded recovery opportunity. If INSERT descent
feedback leaves the fixed `0.0010 m` physical clearance for the existing
pre-contact tick gate before reaching the side-load depth gate, the node stops
the descent, republishes the INSERT handoff hold, and requires the same
`8`-tick handoff stability gate again. The recovery is capped at `2` attempts;
after that the task aborts instead of relaxing the clearance criterion.

## Result

- final outcome: `SUCCESS`;
- insertion depth: `0.0197 m`;
- final INSERT XY error: `0.0003 m`;
- max task INSERT contact: `49.74 N`;
- max task contact: `66.0 N`;
- max raw `|Fz|`: `98.87 N`;
- max force norm: `171.07 N`;
- INSERT pre-depth recenter attempts: `1`;
- SEARCH gate trace rows: `0`;
- positive Gazebo contact-topic samples: `0`;
- launch teardown: task node exited cleanly after DONE; `ros_gz_bridge` and
  `gzserver` still showed shutdown-time `-11` exits.

## Passive Analysis

- `xy_stability_analysis.md`: INSERT final XY `0.000348 m`, INSERT mean XY
  `0.000575 m`, INSERT p95 XY `0.001138 m`, best INSERT `0.0010 m` window
  `62` estimated task ticks, and best INSERT `0.0020 m` window `760` ticks.
- `hold_window_reference_analysis.md`: the first descent command had only
  `8` feedback ticks inside `0.0010 m`; after the bounded recenter restart,
  the second descent command had `46` feedback ticks inside `0.0010 m`.
- `search_tracking_sensitivity_analysis.md`: max centered-hold p95 actual XY
  drift was `0.000963 m`; max centered-hold p95 JTC joint error was
  `0.003784 rad`.
- `search_gate_trace_analysis.md`: SEARCH was bypassed, so the trace contains
  no online SEARCH decisions.

## Decision

Keep the bounded pre-depth INSERT recenter source change. This run exercised
the new recovery once and then produced a measured insertion-depth event under
the fixed physical clearance gate.

This remained a single simulated insertion-depth event. Follow-up repeat
validation in `research_baseline_insert_predepth_recenter_500hz_repeat_v3`
produced only `1/3` physical successes, with two explicit side-load aborts in
INSERT. SEARCH was bypassed, contact-topic samples were zero, and the result is
not a robust autonomous peg-in-hole claim.
