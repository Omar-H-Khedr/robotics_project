# Insert Cartesian Descent Diagnostic

Milestone: `research_baseline_insert_cartesian_descent_v1`

Date: 2026-06-03

Status: rejected; source reverted.

## Purpose

Test whether replacing the one-point INSERT command with a centered
multi-waypoint Cartesian descent would keep peg-tip XY feedback inside the
`0.001 m` physical radial clearance after SEARCH had centered the peg.

## Tested Change

The trial modified INSERT only:

- reused the existing Cartesian descent trajectory helper;
- generated centered INSERT waypoints from the current peg height to the final
  insertion target;
- used axis-aligned IK for those waypoints;
- used a `20 s` minimum INSERT duration;
- preserved the pre-contact clearance gate, side-load abort, and hard-force
  abort.

The change was reverted after validation because it did not improve the
failure mode.

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
  tracking_log_dir:=diagnostics/research_baseline_insert_cartesian_descent_v1

ros2 run thesis_bringup moving_to_start_tracking_analyzer \
  diagnostics/research_baseline_insert_cartesian_descent_v1
ros2 run thesis_bringup endpoint_hold_dynamics_analyzer \
  diagnostics/research_baseline_insert_cartesian_descent_v1
ros2 run thesis_bringup approach_tracking_analyzer \
  diagnostics/research_baseline_insert_cartesian_descent_v1
ros2 run thesis_bringup insert_retreat_contact_analyzer \
  diagnostics/research_baseline_insert_cartesian_descent_v1
ros2 run thesis_bringup insert_xy_drift_analyzer \
  diagnostics/research_baseline_insert_cartesian_descent_v1
ros2 run thesis_bringup withdrawal_contact_timing_analyzer \
  diagnostics/research_baseline_insert_cartesian_descent_v1
```

## Runtime Result

`trial_outcome.json`:

- final outcome: `ABORTED`
- reason: `INSERT aborted: no-contact XY error 0.0026m exceeds physical clearance 0.0010m before meaningful insertion depth 0.0010m for 3 ticks.`
- insertion depth: `0.0000 m`
- pre-insertion XY after SEARCH: `0.0006 m`
- final insertion XY at abort: `0.0026 m`
- max raw `|Fz|`: `130.64 N`
- max raw force norm: `210.62 N`
- max task-side INSERT contact estimate: `50.49 N`

The INSERT command did publish as a multi-point command:

- command point count: `6`
- command duration: `20.000 s`
- observed before abort: `0.297 s`

## Analyzer Evidence

`insert_xy_drift_analysis.md`:

- command-window initial XY error: `0.000458 m`
- first clearance violation: `0.005 s` after INSERT command receipt
- final XY error: `0.002675 m`
- max XY error: `0.004264 m`
- meaningful depth: none
- side-load event: none

`insert_retreat_contact_analysis.md`:

- max physical depth: `0.000000 m`
- final physical depth: `0.000000 m`
- final feedback z: `0.828234 m`, still above `HOLE_TOP_Z=0.810000 m`

`withdrawal_contact_timing_analysis.md` and `contact_state_summary.md`:

- positive contact samples: `0`
- max contact-topic force: `0.000000 N`

## Interpretation

This diagnostic was safe but not useful enough to keep. A centered Cartesian
multi-waypoint INSERT command still violated physical clearance almost
immediately after command receipt, before meaningful depth. The active source
was reverted to the validated pre-contact clearance gate implementation.

The next implementation should not simply add more INSERT waypoints. It should
address the command handoff/hold dynamics that let XY leave clearance within
the first controller-state samples after INSERT starts.
