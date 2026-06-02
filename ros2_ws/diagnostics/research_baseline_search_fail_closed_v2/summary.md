# research_baseline_search_fail_closed_v2

Date: 2026-06-02

## Purpose

Validate two safety changes after the axis-aligned start-pose milestone:

- give the safer vertical `MOVING_TO_START` pose enough time to satisfy the strict 2 mm no-contact XY gate;
- prevent the state machine from entering `SEARCH` after a degraded or timed-out `APPROACH`.

## Code Change

- `MOVING_TO_START_TIMEOUT_S = 120.0` was added for the large axis-aligned no-contact move.
- `APPROACH` timeout/degraded behavior now aborts instead of proceeding.
- `SEARCH` is allowed only after a completed approach when:
  - peg Z is at or below `INSERT_PRECONDITION_MAX_Z = 0.845 m`;
  - residual XY error is no larger than `SEARCH_RADIUS_MAX = 0.015 m`.

The strict no-contact descent gate remains `APPROACH_START_XY_TOLERANCE = 0.002 m`. No insertion or force gate was loosened.

## Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control
source install/setup.bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_search_fail_closed_v2
```

## Result

The runtime reached the node's final outcome before the external timeout killed the launch wrapper.

- Trial outcome: `ABORTED`
- Final reason: `APPROACH timeout/degraded failure (90.0s). cart_err=0.072m, joint_err=0.107rad, tolerance=0.050m`
- `MOVING_TO_START`: success, duration `96.3 s`, `cart_error_m=0.012606`, `joint_error_rad=0.063314`
- Initial above-hole XY error: `0.0018 m`
- `APPROACH`: failed by timeout, `cart_error_m=0.072320`, `joint_error_rad=0.107112`
- Insertion depth: `0.0000 m`
- Peak raw `|Fz|`: `691.37 N`
- Peak raw force norm: `703.09 N`
- Max estimated contact force: `479.29 N`
- SEARCH did not run.

## Observer Evidence

- `contact_state_summary.md`: states observed were `MOVING_TO_START`, `APPROACH`, and `ABORT`; no `SEARCH` rows.
- `wrench_state_summary.md`: max raw `|Fz| = 691.368902 N`, max force norm `703.093506 N`.
- `trajectory_tracking_summary.md`: 2 observed trajectory commands, max joint command-vs-feedback error `0.109348 rad`.

## Interpretation

This is a safety improvement, not insertion success. The safer axis-aligned no-contact start pose can now satisfy the strict 2 mm gate if allowed 120 seconds. The previous unsafe behavior, where a failed approach could proceed into local `SEARCH`, has been removed.

The remaining blocker is approach/descent tracking: the controller commands a 67 mm downward Cartesian descent, but the measured peg tip remains near `z=0.90 m` instead of reaching `z=0.83 m`.
