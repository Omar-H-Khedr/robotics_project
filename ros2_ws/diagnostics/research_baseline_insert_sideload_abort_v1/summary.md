# Insert Sideload Abort V1

Milestone: `research_baseline_insert_sideload_abort_v1`

Purpose: stop INSERT when the peg becomes laterally side-loaded after entering
the hole, instead of continuing to deeper invalid insertion and high-force
extraction.

## Change

`admittance_insertion_node` now aborts INSERT when all of these are true:

- physical insertion depth is at least `0.0010 m`;
- XY error exceeds the physical radial clearance `0.0010 m`;
- the condition persists for `3` consecutive control ticks.

The gate preserves the existing global force abort, final physical-success
clearance gate, and RETREAT behavior. It does not loosen any safety threshold.

## Validation Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_insert_sideload_abort_v1
ros2 run thesis_bringup moving_to_start_tracking_analyzer diagnostics/research_baseline_insert_sideload_abort_v1
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer diagnostics/research_baseline_insert_sideload_abort_v1
ros2 run thesis_bringup approach_tracking_analyzer diagnostics/research_baseline_insert_sideload_abort_v1
ros2 run thesis_bringup insert_retreat_contact_analyzer diagnostics/research_baseline_insert_sideload_abort_v1
ros2 run thesis_bringup withdrawal_contact_timing_analyzer diagnostics/research_baseline_insert_sideload_abort_v1
```

The launch reached task `DONE`; the outer wrapper then exited by timeout after
the completed outcome was already written.

## Runtime Result

Final outcome: `ABORTED`

Reason: `INSERT aborted: side-loaded peg at depth 0.0011m with XY error
0.0032m, exceeding physical clearance 0.0010m for 3 ticks.`

Key metrics:

- insertion depth metric: `0.0038 m`;
- max task-side INSERT contact: `53.75 N`;
- final insertion XY error: `0.0032 m`;
- max raw `|Fz|`: `128.53 N`;
- max raw force norm: `204.14 N`;
- pre-insertion XY error: `0.0008 m`;
- INSERT observed window before abort: `12.810 s`.

Passive contact evidence:

- contact-topic rows: `2`;
- max contact-topic force: `0.000000 N`;
- RETREAT/ABORT peak contact force from passive contacts: `0.000000 N`;
- max physical depth in insert/retreat analyzer: `0.006160 m`.

## Comparison To Physical XY Gate V1

`research_baseline_insert_physical_xy_gate_v1` continued to INSERT completion,
then reported `DEGRADED` with final XY over clearance. It recorded:

- contact-topic rows: `1293`;
- RETREAT contact rows: `1287`;
- RETREAT max contact force: `589.942680 N`;
- max raw force norm: `234.96 N`;
- final insertion depth: `0.0177 m`.

The side-load abort reduced passive contact rows from `1293` to `2`, reduced
RETREAT max contact-topic force from `589.942680 N` to `0.000000 N`, and
reduced max raw force norm from `234.96 N` to `204.14 N`. This is a safety
improvement and an honest abort, not insertion success.

## Interpretation

The current blocker is no longer false success classification. The baseline now
fails closed when inserted-depth XY drift exceeds physical clearance. The next
implementation should reduce the cause of XY drift during INSERT, likely by
improving inserted-depth centering, contact search behavior, or controller
tracking before attempting repeated validation.
