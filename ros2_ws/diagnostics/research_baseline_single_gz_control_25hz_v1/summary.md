# research_baseline_single_gz_control_25hz_v1

Date: 2026-06-07

Purpose: validate single `gz_ros2_control` plugin startup after stripping the
converted upstream KUKA vendor plugin, and run a 25 Hz task-cadence SEARCH
diagnostic without loosening the 1 mm physical radial clearance gate.

Command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=2000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=5.0 \
  inject_velocity_state:=true \
  tracking_log_dir:=diagnostics/research_baseline_single_gz_control_25hz_v1
```

Result: externally timed out during `SEARCH`; no INSERT was attempted and no
peg-in-hole success is claimed.

Startup evidence:

- `spawn_robot_sdf` removed the converted upstream `gz_ros2_control` plugin
  that referenced `kuka_resources/config/fake_hardware_config_6_axis.yaml`.
- Only the intended research controller manager initialized in the retained
  run: `position_proportional_gain=2000` and `position velocity` state
  interfaces were reported for the JTC.
- The previous duplicate-controller symptoms were absent: no duplicate
  `joint_state_broadcaster` activation failure and no vendor fake-hardware
  controller-manager path in the retained run.

Motion evidence:

- `MOVING_TO_START` completed and transitioned to `APPROACH`.
- `APPROACH` completed enough to enter `SEARCH`.
- `SEARCH` recentering remained fail-closed before INSERT. The external
  150 s timeout interrupted the run while SEARCH was still trying to sustain
  physical clearance.
- Contact observer recorded zero contact samples; this was a no-contact
  centering diagnostic.

Key passive metrics:

- XY stability analyzer cadence: `25.0 Hz`.
- SEARCH best estimated 1 mm window: `2` ticks (`0.08 s`).
- SEARCH best estimated 2 mm window: `8` ticks (`0.32 s`).
- SEARCH final XY error in the partial log: `0.001735 m`.
- Hold-window analyzer best hold-like feedback 1 mm window: `3` ticks.
- JTC controller-state p95 max absolute joint error: `0.012063 rad`.
- Max passive force norm: `210.28 N`; no hard force abort evidence in the
  partial log.

Interpretation: removing the duplicate converted controller plugin fixes a real
startup/configuration fault. It does not solve the physical-clearance SEARCH
blocker: at 25 Hz, the retained run still did not sustain measured 1 mm
clearance long enough to enter INSERT.
