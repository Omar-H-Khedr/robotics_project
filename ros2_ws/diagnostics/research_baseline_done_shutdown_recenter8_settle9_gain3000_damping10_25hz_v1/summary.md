# DONE Shutdown Hook Validation Attempt

Date: 2026-06-07

Purpose: validate the new `exit_on_done` / `shutdown_on_task_exit` launch path
after the prior headless runs kept publishing `DONE` until the outer `timeout`
process killed the launch.

## Runtime Command

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
timeout 180s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  search_recenter_duration_s:=8.0 \
  search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 \
  tracking_log_dir:=diagnostics/research_baseline_done_shutdown_recenter8_settle9_gain3000_damping10_25hz_v1
```

## Runtime Result

- outer process result: `timeout` exit code `124`;
- final observed state: `SEARCH`;
- no final task outcome JSON should be inferred from this run;
- no INSERT phase was reached;
- insertion depth: not evaluated;
- positive Gazebo contact-topic samples: `0`;
- max raw `|Fz|`: `102.14 N`;
- max force norm: `168.21 N`.

The run did not exercise the launch shutdown-on-DONE path because the task did
not reach `DONE` before the external 180 s wrapper expired.

## Passive SEARCH Analysis

- SEARCH duration captured before timeout: `35.057 s`;
- SEARCH min XY: `0.000011 m`;
- SEARCH mean XY: `0.001199 m`;
- SEARCH final XY: `0.001312 m`;
- SEARCH best estimated `0.0010 m` window: `7` ticks at 25 Hz;
- SEARCH best estimated `0.0020 m` window: `54` ticks at 25 Hz;
- hold-like best feedback `0.0010 m` window: `6` ticks;
- max centered-hold p95 actual XY drift: `0.002263 m`;
- max centered-hold p95 JTC joint-position error: `0.007683 rad`.

Interpretation: the same tuning that previously reached INSERT is
non-deterministic. This repeat came within one tick of the fixed 8-tick
physical-clearance SEARCH gate, then continued recentering until the external
timeout. The result reinforces that sustained no-contact centering remains the
blocker and that no insertion success should be claimed.

## Direct Exit Hook Smoke Test

Because the full launch did not reach `DONE`, the task-node exit hook was
validated directly:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 8s python3 - <<'PY'
import rclpy
from rclpy.executors import ExternalShutdownException
from kuka_task_control.admittance_insertion_node import AdmittanceInsertionNode

rclpy.init(args=['--ros-args', '-p', 'exit_on_done:=true', '-p', 'done_exit_delay_s:=0.05'])
node = AdmittanceInsertionNode()
node._log_final_outcome()
try:
    rclpy.spin(node)
except ExternalShutdownException:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
print('exit_on_done smoke test completed')
PY
```

Result: process exited with code `0` before the 8 s wrapper. The node logged
`exit_on_done enabled`, then `DONE outcome written; exiting task node.`
