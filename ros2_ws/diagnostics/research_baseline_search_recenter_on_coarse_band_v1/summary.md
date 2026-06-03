# Search Recenter On Coarse Band V1

Date: 2026-06-03

## Change

`admittance_insertion_node.py` now treats SEARCH feedback inside the older
`0.0020 m` pre-contact band as near-centered but not physically ready. Instead
of immediately advancing to another spiral offset when this happens, SEARCH
publishes a centered hold target at the current peg Z for `3.0 s`, then
requires the existing sustained `0.0010 m` physical clearance gate before
INSERT. The `SEARCH` timeout, hard-force abort, and all INSERT gates remain
unchanged.

The SEARCH elapsed counter is also reset when SEARCH is entered so future
state re-entries cannot inherit a stale timeout counter.

## Validation

Syntax and targeted build:

```bash
python3 -m py_compile ros2_ws/src/kuka_task_control/kuka_task_control/admittance_insertion_node.py
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
```

Runtime command:

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source /opt/ros/jazzy/setup.bash
cd /home/omar/code/robotics_project/ros2_ws
source install/setup.bash
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_search_recenter_on_coarse_band_v1
```

The wrapper exited with timeout code `124` after the task had already reported
its final outcome and the observer summaries had flushed.

## Runtime Outcome

- outcome: `ABORTED`
- reason: `SEARCH timeout (45s). XY error 0.0037m remains above tolerance.`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `128.6 N`
- max raw force norm: `209.9 N`
- task contact estimate: `78.3 N`
- final reported pre-insertion XY error: `0.0039 m`
- phase sequence: `MOVING_TO_START OK`, `APPROACH OK`, `SEARCH FAIL`
- INSERT was not entered.
- The console log showed two SEARCH recenter attempts before timeout.

## Passive Observer Evidence

- `contact_state_summary.md`: `0` contact-topic samples.
- `trajectory_tracking_summary.md`: `10` observed commands, `47084` JTC
  controller-state samples, p95 controller-state max abs position error
  `0.011366 rad`.
- `wrench_state_summary.md`: `19200` wrench samples, max raw `|Fz|`
  `128.608358 N`, max force norm `209.874192 N`.
- `xy_stability_analysis.md`: SEARCH best estimated `0.0010 m` clearance window
  improved to `4` controller ticks; SEARCH best estimated `0.0020 m` window was
  `9` controller ticks.

## Decision

Keep the change as a safety-preserving improvement, not as success. It did not
meet the required `8` consecutive SEARCH ticks inside the physical `0.0010 m`
clearance, and it did not enter INSERT. The next milestone should continue
stabilizing near-centered SEARCH feedback, likely by widening the recenter
trigger within the bounded no-contact search radius or reducing oscillation
after recenter commands, while preserving the physical 1 mm insertion gate.
