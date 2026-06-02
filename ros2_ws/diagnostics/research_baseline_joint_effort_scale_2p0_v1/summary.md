# Joint Effort Scale 2.0 Diagnostic

Date: 2026-06-02

Command:

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_joint_effort_scale_2p0_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 260s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  joint_effort_scale:=2.0 \
  tracking_log_dir:=diagnostics/research_baseline_joint_effort_scale_2p0_v1
```

Code state:

- `spawn_robot_sdf` scaled converted SDF joint effort limits only for this diagnostic run.
- The canonical launch default remains `joint_effort_scale:=1.0`.
- Damping, position gain, task safety gates, no-contact gates, and hard-force abort were not relaxed.

Observed SDF overrides:

- `joint_1`: effort `221.183 -> 442.366`
- `joint_2`: effort `199.605 -> 399.21`
- `joint_3`: effort `80.2182 -> 160.436`
- `joint_4`: effort `35.688 -> 71.376`
- `joint_5`: effort `35.2384 -> 70.4768`
- `joint_6`: effort `8.56753 -> 17.1351`
- `ft_sensor_joint`: effort `10000 -> 20000`

Runtime result:

- outcome: `ABORTED`
- reason: `Hard force abort: raw wrench exceeded 1000.0N in state APPROACH; |Fz|=968.4N, |F|=1009.7N.`
- insertion depth: `0.0000 m`
- `MOVING_TO_START`: success after `81.4 s`, Cartesian error `0.006955 m`, joint error `0.012782 rad`, initial XY error `0.0007 m`
- `APPROACH`: failed after `0.5 s`, Cartesian error `0.001890 m`
- max raw `|Fz|`: `1020.10 N`
- max raw force norm: `1043.00 N`
- max contact-force estimate: `747.58 N`

Passive observer evidence:

- `trajectory_tracking_summary.md`: max joint error `0.079934 rad`, p95 max joint error `0.048923 rad`, final max joint error `0.028637 rad`.
- `approach_tracking_analysis.md`: command-index 1 analysis of the short `APPROACH` command. The abort happened after only `0.283 s` of observed approach samples; the peg was still at `z=0.890982 m` against the `z=0.830000 m` target, with `0.060988 m` Cartesian error and `joint_2` target-minus-feedback error `0.099473 rad`.
- `wrench_state_summary.md`: `APPROACH` max abs Fz `968.406086 N`, max force norm `1009.722857 N`; total run max abs Fz `1020.099637 N`.
- `contact_state_summary.md`: target-source contact rows were present in `MOVING_TO_START`, `APPROACH`, and `ABORT`; max target contact force `9925.518339 N`.
- `trial_outcome.json`: copied from `/tmp/insertion_trial_outcome.json` after the run.

Conclusion:

Doubling converted joint effort limits is rejected as a canonical behavior. It improved start-gate timing compared with the 96 s canonical fail-closed run and allowed `APPROACH` to begin, but it immediately generated unsafe wrench/contact evidence before insertion. The effort result suggests the joint tracking problem is entangled with physics/contact or force measurement during the transition into descent; it does not justify relaxing safety gates or retaining higher effort limits.
