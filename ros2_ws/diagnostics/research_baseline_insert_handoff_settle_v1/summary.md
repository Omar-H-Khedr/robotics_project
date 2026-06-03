# Insert Handoff Settle Validation

Milestone: `research_baseline_insert_handoff_settle_v1`

Date: 2026-06-03

## Purpose

Add a bounded no-contact handoff hold before publishing the final INSERT
descent command. The descent should start only after feedback remains within
the physical radial clearance. If feedback leaves clearance, the existing
pre-contact gate must still abort before meaningful insertion depth.

## Implementation

- Added a non-descending INSERT handoff hold at current peg Z and centered XY.
- Required `8` stable control ticks inside the `0.0010 m` physical radial
  clearance after a `2.0 s` hold before publishing the final descent.
- Added a `6.0 s` timeout for the handoff hold.
- Preserved the pre-contact clearance abort and side-load abort.

## Validation Commands

```bash
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_insert_handoff_settle_v1
ros2 run thesis_bringup insert_handoff_reference_analyzer diagnostics/research_baseline_insert_handoff_settle_v1 --command-index 2 --output-name insert_handoff_hold_reference_analysis.md --json-name insert_handoff_hold_reference_analysis.json
ros2 run thesis_bringup insert_xy_drift_analyzer diagnostics/research_baseline_insert_handoff_settle_v1 --command-index 2 --output-name insert_handoff_hold_xy_drift_analysis.md --json-name insert_handoff_hold_xy_drift_analysis.json
```

## Runtime Result

The run reached `DONE` and reported a bounded safety failure:

- final outcome: `ABORTED`;
- reason: `INSERT aborted: no-contact XY error 0.0024m exceeds physical clearance 0.0010m before meaningful insertion depth 0.0010m for 3 ticks.`;
- insertion depth: `0.0000 m`;
- max raw `|Fz|`: `127.6 N`;
- max raw force norm: `213.6 N`;
- max task-side contact estimate: `88.7 N`;
- final reported XY error: `0.0004 m`;
- phase sequence: `MOVING_TO_START OK`, `APPROACH OK`, `SEARCH OK`, `INSERT FAIL`.

The canonical launch path did not leave a `trial_outcome.json` in the tracking
directory before the validation wrapper was stopped after `DONE`; the outcome
above is transcribed from the task node's final console report.

## Observer Evidence

- trajectory observer commands: `4`;
- command index `2`: handoff hold, `1` point, `2.000 s`;
- command index `3`: abort retreat, `15` points, `25.000 s`;
- no final descent-to-`z=0.790 m` INSERT command was published;
- contact-topic samples: `0`;
- wrench summary max raw `|Fz|`: `127.950934 N`;
- wrench summary max force norm: `213.563041 N`;
- INSERT-state wrench samples: `31`;
- INSERT-state min XY: `0.000070 m`;
- INSERT-state mean XY: `0.002023 m`.

## Handoff Analyzer Result

Analyzer target: command index `2`, the new handoff hold.

- pre-command final reference XY error: `0.000010 m`;
- pre-command final feedback XY error: `0.002773 m`;
- initial reference and feedback XY error: `0.001747 m`;
- first feedback clearance violation: `0.000 s`;
- max reference XY error in the analyzed window: `0.001747 m`;
- max feedback XY error in the analyzed window: `0.004553 m`;
- max Cartesian reference-feedback error: `0.004938 m`;
- meaningful depth: none;
- nearest contact-topic force at first violation: `0.000000 N`.

## Interpretation

The handoff hold improved safety by preventing the final insertion descent
command from being published when feedback was already outside the physical
clearance. It did not solve insertion. The task still aborts honestly before
meaningful depth.

The root issue has moved upstream: `SEARCH` currently declares convergence from
a transient inside-clearance sample, but feedback is outside clearance by the
INSERT handoff. The next implementation should require sustained, controller-
state-confirmed SEARCH convergence before entering INSERT.
