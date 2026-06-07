# Current-Joint Handoff Candidate Diagnostic

Date: 2026-06-07

Milestone: `research_baseline_current_joint_handoff_recenter8_gain3000_damping10_25hz_v1`

## Command

Before the run, the selected source packages were rebuilt from a clean package
build/install state to avoid stale installed launch files:

```bash
rm -rf build/kuka_task_control install/kuka_task_control build/thesis_bringup install/thesis_bringup
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kuka_task_control thesis_bringup
source install/setup.bash
rg -n "RESEARCH_ROBOT_XACRO|default_value=\"lbr_iisy" install/thesis_bringup/share/thesis_bringup/launch/research_baseline.launch.py
```

Validation command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 210s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  search_recenter_duration_s:=8.0 \
  search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 \
  tracking_log_dir:=diagnostics/research_baseline_current_joint_handoff_recenter8_gain3000_damping10_25hz_v1
```

## Result

- launch model check: `lbr_iisy6_r1300_research_gripper.urdf.xacro`, model `lbr_iisy6_r1300`, spawn z `0.735`
- final observed state sequence: `UNKNOWN -> MOVING_TO_START -> APPROACH -> SEARCH -> ABORT`
- final observed task log reason: `SEARCH timeout (45s). Instantaneous XY error 0.0005m is within physical clearance 0.0010m but was not sustained for 8 post-command ticks.`
- final transition observed in task log: `SEARCH -> ABORT`
- no `INSERT` state samples were recorded
- no INSERT handoff command was published
- outer wrapper: launch was still in ABORT retreat when the `210 s` timeout killed the process, so no fresh final outcome JSON was written for this diagnostic
- insertion depth: `0.0000 m`
- positive Gazebo contact-topic samples: `0`
- max raw `|Fz|` in passive observer: `101.52 N`
- max force norm in passive observer: `171.99 N`

## Passive Analyses

Files retained:

- `xy_stability_analysis.md`
- `xy_stability_analysis.json`
- `hold_window_reference_analysis.md`
- `hold_window_reference_analysis.json`
- `search_tracking_sensitivity_analysis.md`
- `search_tracking_sensitivity_analysis.json`
- `trajectory_tracking_summary.md`
- `wrench_state_summary.md`
- `contact_state_summary.md`

Key metrics:

- SEARCH best estimated `0.0010 m` stability window: `9` task ticks in passive replay.
- SEARCH final XY error in the observer stream: `0.000502 m`.
- The state-machine log still timed out because the inside-clearance samples were not sustained for the task node's own post-command count.
- Hold-like command count: `4`.
- Hold-like best feedback `0.0010 m` window: `7` ticks.
- Max centered-hold p95 actual XY drift: `0.002290 m`.
- Max centered-hold p95 JTC joint-position error: `0.007722 rad`.
- Trajectory observer max abs position error: `0.139830 rad`; controller-state max abs position error: `0.017593 rad`; controller-state p95 max abs position error: `0.010650 rad`.

## Interpretation

This run was started after a candidate source edit that would hold current
measured joints during INSERT handoff instead of solving a fresh centered IK
target. The candidate was not exercised: SEARCH failed closed before INSERT,
and the trajectory log contains no INSERT handoff command.

The source edit was reverted. The useful evidence from this run is that the
strict SEARCH gate still needs robust task-node-visible post-command
stability. Passive replay can find brief windows that appear to satisfy
`8` ticks, but the online task node still saw intermittent samples and timed
out safely. Do not count this run as insertion progress or as validation of a
current-joint handoff strategy.
