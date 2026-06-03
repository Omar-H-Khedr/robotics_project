# Search Recenter 4 mm V1

Date: 2026-06-03

## Change

SEARCH recentering was widened from the older `0.0020 m` pre-contact band to a
bounded `0.0040 m` near-center band. This does not change the physical
`0.0010 m` clearance required for INSERT and does not change the SEARCH
timeout, hard-force abort, or INSERT gates.

## Validation

Syntax and targeted build passed:

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
timeout 430s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false joint_damping_scale:=5.0 tracking_log_dir:=diagnostics/research_baseline_search_recenter_4mm_v1
```

The wrapper exited with timeout code `124` after the task reported its final
outcome and passive summaries flushed.

## Runtime Outcome

- outcome: `ABORTED`
- reason: `INSERT aborted: no-contact XY error 0.0035m exceeds physical clearance 0.0010m before meaningful insertion depth 0.0010m for 3 ticks.`
- insertion depth: `0.0000 m`
- max raw `|Fz|`: `132.2 N`
- max raw force norm: `211.5 N`
- contact-topic samples: `0`
- phase sequence: `MOVING_TO_START OK`, `APPROACH OK`, `INSERT FAIL`

This run did not exercise SEARCH because APPROACH completed at `0.0009 m` XY
error and entered INSERT directly. It is therefore not sufficient validation of
the wider SEARCH recenter trigger. It does confirm the active INSERT handoff
gate still aborts before descent when feedback leaves physical clearance.

## Passive Observer Evidence

- `contact_state_summary.md`: `0` contact-topic samples.
- `trajectory_tracking_summary.md`: `4` observed commands, `49597` JTC
  controller-state samples, p95 controller-state max abs position error
  `0.011884 rad`.
- `wrench_state_summary.md`: `20348` wrench samples, max raw `|Fz|`
  `132.211485 N`, max force norm `211.531894 N`.
- `xy_stability_analysis.md`: INSERT best estimated `0.0010 m` window was
  `3` controller ticks before abort.

## Decision

Use `diagnostics/research_baseline_search_recenter_4mm_v1_repeat2` as the
actual SEARCH validation for this source change. This first run is retained as
handoff-drift evidence only.
