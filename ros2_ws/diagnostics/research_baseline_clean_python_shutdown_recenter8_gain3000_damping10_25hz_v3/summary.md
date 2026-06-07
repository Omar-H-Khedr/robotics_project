# Clean Python Shutdown and SEARCH Gate Trace Validation

Date: 2026-06-07

Milestone: `research_baseline_clean_python_shutdown_recenter8_gain3000_damping10_25hz_v3`

## Scope

This milestone cleans Python-node shutdown behavior after a DONE-reaching task launch. It does not change the physical clearance gates or claim insertion success.

Source changes:

- `trajectory_tracking_observer`, `wrench_state_observer`, and `contact_state_observer` catch `ExternalShutdownException` and only call `rclpy.shutdown()` while the context is still valid.
- `safety_monitor` catches the same external shutdown path and guards `rclpy.shutdown()`.
- `data_logger_node` no longer logs through rosout after context shutdown; it closes the CSV and prints the close message if ROS logging is no longer valid.

## Commands

Build:

```bash
rm -rf build/kuka_task_control install/kuka_task_control build/thesis_bringup install/thesis_bringup build/safety_layer install/safety_layer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control safety_layer thesis_bringup
```

Validation:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 480s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  search_recenter_duration_s:=8.0 \
  search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 \
  tracking_log_dir:=diagnostics/research_baseline_clean_python_shutdown_recenter8_gain3000_damping10_25hz_v3
```

Analysis:

```bash
python3 -m thesis_bringup.xy_stability_analyzer --state-loop-hz 25.0 diagnostics/research_baseline_clean_python_shutdown_recenter8_gain3000_damping10_25hz_v3
python3 -m thesis_bringup.hold_window_reference_analyzer --state-loop-hz 25.0 diagnostics/research_baseline_clean_python_shutdown_recenter8_gain3000_damping10_25hz_v3
python3 -m thesis_bringup.search_tracking_sensitivity_analyzer diagnostics/research_baseline_clean_python_shutdown_recenter8_gain3000_damping10_25hz_v3
```

## Runtime Result

- Launch wrapper exit code: `0`.
- Final task status: `DONE`.
- Final task outcome: `ABORTED`.
- Failed phase: `SEARCH`.
- Reason: `SEARCH timeout (45s). XY error 0.0013m remains above physical clearance 0.0010m.`
- Insertion depth: `0.0000 m`.
- Max raw `|Fz|`: `102.83 N`.
- Max force norm: `168.28 N`.
- Max contact force: `78.44 N`.
- Max INSERT contact force: `0.00 N` because INSERT was not reached.
- SEARCH gate rows: `1125`.
- SEARCH gate decisions: `settling_count=134`, `settling_reset=984`, `post_settle_count=2`, `recenter_command=4`, `timeout_abort=1`.
- SEARCH best estimated `0.0010 m` stability window: `9` ticks in passive replay.
- Hold-like command best feedback `0.0010 m` window: `6` ticks.
- Max centered-hold p95 actual XY drift: `0.002290 m`.
- Max centered-hold p95 JTC joint-position error: `0.007629 rad`.
- Positive contact-topic samples: `0` in the contact observer summary.

The passive replay finding of `9` SEARCH ticks does not override the task result. The online gate trace still ended in `timeout_abort`, so this remains a fail-closed SEARCH instability run.

## Shutdown Evidence

The Python nodes shut down cleanly after the task node exited at DONE:

- `trajectory_tracking_observer`: process finished cleanly.
- `wrench_state_observer`: process finished cleanly.
- `contact_state_observer`: process finished cleanly.
- `safety_monitor`: process finished cleanly.
- `data_logger_node`: process finished cleanly and printed `CSV closed: ...` without rosout context failures.

Remaining shutdown limitation:

- `ros_gz_bridge` nodes still exited with signal `-11`.
- `gzserver` did not terminate after SIGINT/SIGTERM and was killed with signal `-9`.

These remaining failures are in Gazebo/bridge shutdown, not the Python observer/safety/logger nodes addressed by this milestone.

## Decision

Keep the Python shutdown changes. They reduce false runtime noise during DONE-reaching launch validations and make subsequent diagnostics easier to audit. The control blocker remains sustained no-contact centering under the fixed `0.0010 m` physical radial clearance and `8`-tick gate; no insertion success is claimed.
