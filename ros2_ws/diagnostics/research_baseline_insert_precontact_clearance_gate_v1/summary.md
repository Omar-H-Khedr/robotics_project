# Insert Pre-Contact Clearance Gate

Milestone: `research_baseline_insert_precontact_clearance_gate_v1`

Date: 2026-06-03

## Purpose

Prevent the controller from descending toward the hole when the peg-tip XY
feedback has already drifted outside the physical radial clearance of the
25 mm peg / 27 mm hole task.

The previous side-load gate aborted after the peg reached at least `0.001 m`
physical insertion depth. The XY drift diagnostic showed clearance violations
can occur before that depth, so the next safe change was an earlier no-contact
INSERT clearance gate.

## Code Change

`kuka_task_control.admittance_insertion_node` now:

- requires direct INSERT entry to be within `INSERT_FINAL_XY_TOLERANCE`
  (`0.001 m`), not the older nominal `0.002 m` insertion tolerance;
- requires SEARCH convergence to reach the same physical clearance;
- blocks INSERT preconditions when no-contact XY error exceeds physical
  clearance;
- aborts INSERT before meaningful depth if XY error exceeds physical clearance
  for `3` control ticks while depth is still below `0.001 m`;
- preserves the existing inserted-depth side-load abort and hard force abort.

## Validation Commands

```bash
python3 -m py_compile \
  src/kuka_task_control/kuka_task_control/admittance_insertion_node.py

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup

source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  joint_damping_scale:=5.0 \
  tracking_log_dir:=diagnostics/research_baseline_insert_precontact_clearance_gate_v1

ros2 run thesis_bringup moving_to_start_tracking_analyzer \
  diagnostics/research_baseline_insert_precontact_clearance_gate_v1
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer \
  diagnostics/research_baseline_insert_precontact_clearance_gate_v1
ros2 run thesis_bringup approach_tracking_analyzer \
  diagnostics/research_baseline_insert_precontact_clearance_gate_v1
ros2 run thesis_bringup insert_retreat_contact_analyzer \
  diagnostics/research_baseline_insert_precontact_clearance_gate_v1
ros2 run thesis_bringup insert_xy_drift_analyzer \
  diagnostics/research_baseline_insert_precontact_clearance_gate_v1
ros2 run thesis_bringup withdrawal_contact_timing_analyzer \
  diagnostics/research_baseline_insert_precontact_clearance_gate_v1
```

The first sandboxed launch failed before spawn because DDS/Gazebo could not
create local transport sockets under sandbox networking. The same command was
rerun with escalated permissions and completed to task `DONE`.

## Runtime Result

`trial_outcome.json`:

- final outcome: `ABORTED`
- reason: `INSERT aborted: no-contact XY error 0.0027m exceeds physical clearance 0.0010m before meaningful insertion depth 0.0010m for 3 ticks.`
- insertion depth: `0.0000 m`
- pre-insertion XY error after SEARCH: `0.0006 m`
- final insertion XY error at abort: `0.0027 m`
- max raw `|Fz|`: `129.4 N`
- max raw force norm: `205.09 N`
- max task-side INSERT contact estimate: `22.56 N`

Phase sequence:

- MOVING_TO_START: OK, `cart_err=0.002212 m`
- APPROACH: OK, `cart_err=0.012411 m`
- SEARCH: OK, `cart_err=0.000587 m`, `SEARCH converged`
- INSERT: FAIL by no-contact clearance gate after `0.5 s`

## Analyzer Evidence

`insert_xy_drift_analysis.md`:

- INSERT command observed for `0.515 s` before abort;
- command-window initial XY error: `0.001125 m`;
- max XY error: `0.005525 m`;
- first physical-clearance violation: `0.005 s` after command receipt;
- meaningful depth: none;
- side-load event: none.

`insert_retreat_contact_analysis.md`:

- max physical depth: `0.000000 m`;
- final physical depth: `0.000000 m`;
- final feedback z: `0.826544 m`, still above `HOLE_TOP_Z=0.810000 m`;
- contact-state samples unavailable or empty;
- INSERT wrench max force norm: `181.134548 N`.

`withdrawal_contact_timing_analysis.md` and `contact_state_summary.md`:

- positive contact samples: `0`;
- max contact-topic force: `0.000000 N`.

## Interpretation

This is a safety improvement, not task success. The task now prevents descent
past the hole top when no-contact XY feedback violates physical clearance. The
remaining blocker is the one-point INSERT command: after SEARCH reaches
`0.0006 m` XY error, the controller feedback violates clearance almost
immediately. The next implementation should reduce or constrain INSERT path
drift rather than relaxing the clearance gate.
