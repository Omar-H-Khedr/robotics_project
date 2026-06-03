# Insert Physical XY Gate V1

Milestone: `research_baseline_insert_physical_xy_gate_v1`

Purpose: prevent depth plus force contact from being reported as physical
success when the inserted peg is laterally side-loaded beyond the actual
25 mm peg / 27 mm hole clearance.

## Change

`admittance_insertion_node` now records `final_insertion_xy_error_m` at INSERT
completion and requires it to be no larger than the physical radial clearance:

- peg radius: `0.0125 m`;
- hole radius: `0.0135 m`;
- final INSERT XY tolerance: `0.0010 m`.

Depth and INSERT contact evidence are still required. RETREAT contact still
cannot create insertion success.

## Validation Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_insert_physical_xy_gate_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_insert_physical_xy_gate_v1
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer diagnostics/research_baseline_insert_physical_xy_gate_v1
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_insert_physical_xy_gate_v1
ros2 run thesis_bringup insert_retreat_contact_analyzer diagnostics/research_baseline_insert_physical_xy_gate_v1
ros2 run thesis_bringup withdrawal_contact_timing_analyzer diagnostics/research_baseline_insert_physical_xy_gate_v1
```

The launch was allowed to reach task `DONE`; the outer wrapper then exited by
timeout after the completed outcome was already written.

## Runtime Result

Final outcome: `DEGRADED`

Reason: `Final insertion XY error (0.0030m) exceeds physical hole clearance
(0.0010m). Peg is side-loaded; do not count as physical success.`

Key metrics from `trial_outcome.json`:

- insertion depth: `0.0177 m`;
- task-side max INSERT contact: `55.4 N`;
- final insertion XY error: `0.0030 m`;
- final INSERT XY tolerance: `0.0010 m`;
- max raw `|Fz|`: `227.09 N`;
- max raw force norm: `234.96 N`;
- pre-insertion XY error: `0.0018 m`;
- no safety abort.

Passive contact evidence:

- INSERT contact rows: `6`, max `130.162091 N`, peg versus target plate right collision;
- RETREAT contact rows: `1287`, max `589.942680 N`, mostly peg versus target plate left collision;
- withdrawal timing shows RETREAT peak contact at depth `0.017561 m` and XY error `0.005906 m`.

## Interpretation

This is a stricter and more physically credible result than the prior v4
success claim. The controller can produce depth and contact evidence, but the
peg is not centered within the available radial clearance at INSERT completion.
The next implementation should reduce inserted-depth XY drift and side-loaded
extraction contact before any repeated-validation or learning claim.
