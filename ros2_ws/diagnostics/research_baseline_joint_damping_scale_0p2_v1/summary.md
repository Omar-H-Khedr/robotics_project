# Joint Damping Scale 0.2 Diagnostic

Date: 2026-06-02

Command:

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_joint_damping_scale_0p2_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 240s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  joint_damping_scale:=0.2 \
  tracking_log_dir:=diagnostics/research_baseline_joint_damping_scale_0p2_v1
```

Code state:

- `spawn_robot_sdf` scaled converted SDF joint damping only for this diagnostic run.
- The canonical launch default remains `joint_damping_scale:=1.0`.
- Effort limits, position gain, task safety gates, no-contact gates, and hard-force abort were not relaxed.

Observed SDF overrides:

- `joint_1`: damping `30 -> 6`
- `joint_2`: damping `30 -> 6`
- `joint_3`: damping `20 -> 4`
- `joint_4`: damping `10 -> 2`
- `joint_5`: damping `10 -> 2`
- `joint_6`: damping `5 -> 1`
- `ft_sensor_joint`: damping `1 -> 0.2`

Runtime result:

- outcome: `ABORTED`
- reason: `Hard force abort: raw wrench exceeded 1000.0N in state MOVING_TO_START; |Fz|=1181.0N, |F|=1272.7N.`
- insertion depth: `0.0000 m`
- `MOVING_TO_START` failed at about `41.4 s`
- phase Cartesian error at abort: `0.272028 m`
- max raw `|Fz|`: `1181.04 N`
- max raw force norm: `1272.74 N`
- max contact-force estimate: `498.45 N`
- contact observer recorded zero bridged contact samples in this run

Passive observer evidence:

- `trajectory_tracking_summary.md`: max joint error `0.189038 rad`, p95 max joint error `0.113972 rad`, final max joint error `0.046676 rad`.
- `wrench_state_summary.md`: `MOVING_TO_START` max abs Fz `1181.037416 N`, max force norm `1272.736100 N`.
- `contact_state_summary.md`: no contact-topic samples were recorded.
- `trial_outcome.json`: copied from `/tmp/insertion_trial_outcome.json` after the run.

Conclusion:

Reducing converted joint damping by 80% is rejected as a canonical behavior. It did not reach the strict no-contact gate or approach phase, and it increased/triggered an unsafe hard-force event during `MOVING_TO_START`. The next investigation should avoid broad damping reduction and instead localize whether the large `joint_2` descent error comes from joint-specific dynamics, effort limits, trajectory target timing, or interaction between high free-space wrench spikes and controller authority.
