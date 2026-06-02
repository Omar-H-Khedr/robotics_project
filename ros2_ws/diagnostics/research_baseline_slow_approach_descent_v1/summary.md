# research_baseline_slow_approach_descent_v1

Date: 2026-06-02

## Purpose

Test whether the `APPROACH` failure after valid above-hole alignment is caused primarily by trajectory timing. The temporary experiment changed only the Cartesian descent duration from the default 15 s minimum to a slower 41.7 s trajectory for the observed 67 mm descent.

This experiment was rejected and the code change was reverted.

## Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control
source install/setup.bash
timeout 260s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_slow_approach_descent_v1
```

## Result

- Trial outcome: `ABORTED`
- Final reason: `APPROACH timeout/degraded failure (90.0s). cart_err=0.070m, joint_err=0.108rad, tolerance=0.050m`
- `MOVING_TO_START`: success, duration `95.6 s`, `cart_error_m=0.012693`, `joint_error_rad=0.064041`
- Initial above-hole XY error: `0.0015 m`
- `APPROACH`: failed by timeout, `cart_error_m=0.070358`, `joint_error_rad=0.108243`
- Insertion depth: `0.0000 m`
- Peak raw `|Fz|`: `570.07 N`
- Peak raw force norm: `627.03 N`
- Max estimated contact force: `362.49 N`
- SEARCH did not run.

## Tracking Interpretation

The slower command was accepted:

- approach command duration: `41.662 s`;
- approach command target FK: approximately `(0.520, -0.200, 0.830)`;
- trajectory observer max joint error: `0.108618 rad`;
- trajectory observer p95 max joint error: `0.103257 rad`.

The peg tip still remained near `z=0.898 m` through the approach phase instead of reaching the `z=0.830 m` target. The result is essentially the same failure mode as `research_baseline_search_fail_closed_v2`.

## Decision

Rejected. A timing-only approach change does not fix the descent blocker and was not retained. The next investigation should target why `joint_2` stalls roughly 0.108 rad away from the approach target despite a valid command, rather than only stretching trajectory time.
