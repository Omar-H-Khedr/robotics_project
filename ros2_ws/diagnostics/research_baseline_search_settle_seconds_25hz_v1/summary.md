# research_baseline_search_settle_seconds_25hz_v1

Date: 2026-06-07

Purpose: validate a cadence-neutral SEARCH settling window after discovering
that the previous SEARCH implementation used a hardcoded `60` state ticks. At
the canonical 10 Hz this was 6.0 s, but at `control_rate:=25.0` it became only
2.4 s, shorter than the 5.0 s recenter command duration.

Code change under test:

- `AdmittanceInsertionNode.SEARCH_SETTLE_DURATION_S = 6.0`
- SEARCH settling now compares elapsed seconds against that duration and keeps
  waiting while the active recenter/search command is still running.
- Final outcome metrics now record `search_settle_duration_s`.

Build/runtime notes:

- `python3 -m py_compile src/kuka_task_control/kuka_task_control/admittance_insertion_node.py`
- `colcon build --symlink-install --packages-select kuka_task_control thesis_bringup`
- A first launch attempt exposed stale generated install data: the installed
  `thesis_bringup` launch file still contained old iisy3 defaults. The retained
  run was executed only after cleaning generated `build/thesis_bringup` and
  `install/thesis_bringup`, rebuilding `thesis_bringup`, and confirming the
  installed launch file referenced `lbr_iisy6_r1300`.
- Retained run command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 180s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=2000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=5.0 \
  inject_velocity_state:=true \
  tracking_log_dir:=diagnostics/research_baseline_search_settle_seconds_25hz_v1
```

Runtime outcome:

- Correct iisy6 launch path was used.
- `spawn_robot_sdf` removed the converted upstream vendor `gz_ros2_control`
  plugin and injected the intended research controller plugin.
- `joint_state_broadcaster` and `joint_trajectory_controller` activated.
- The state machine reached SEARCH.
- The external `timeout` ended the run during SEARCH before INSERT.
- No `/tmp/insertion_trial_outcome.json` was produced.
- No insertion success is claimed.

Evidence:

- SEARCH logs showed recenter holds now reached `settle_elapsed=6.0s` at
  25 Hz before another recenter command was sent.
- `xy_stability_analysis.md`: SEARCH best estimated 1 mm window `4` ticks
  (`0.16 s`) and best estimated 2 mm window `6` ticks (`0.24 s`).
- `hold_window_reference_analysis.md`: hold-like best feedback 1 mm window
  `4` ticks.
- SEARCH final XY in the retained partial log: `0.001588 m`.
- Passive contact observer: `0` samples, `0` positive contact samples.
- Wrench observer peak force norm: `211.930944 N`; max absolute Fz:
  `127.233045 N`.
- Controller-state p95 max joint-position error:
  `0.011514 rad`; final max joint-position error `0.004700 rad`.

Decision:

The hardcoded-tick SEARCH settling bug is fixed for non-default control rates,
but it does not unblock insertion. The sustained 1 mm physical clearance gate
still fails closed. Continue with controller/physics tracking and near-hole
feedback stabilization; do not relax the `0.0010 m` physical gate.
