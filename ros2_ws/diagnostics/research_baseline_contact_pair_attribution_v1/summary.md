# research_baseline_contact_pair_attribution_v1

Date: 2026-06-02

## Purpose

Attribute Gazebo contact messages to exact collision pairs during a force-abort run. The previous doubled-effort diagnostic showed unsafe contact/force evidence but did not identify whether the contact was peg-tip contact, fixture support contact, or a robot-link clearance collision.

## Change Under Test

`contact_state_observer.py` now records a `collision_pairs` CSV column and writes a collision-pair table in `contact_state_summary.md`.

This is passive instrumentation only. It does not publish commands, change controller gains, change task gates, or alter safety behavior.

## Validation Command

```bash
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_contact_pair_attribution_v1
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 260s ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  joint_effort_scale:=2.0 \
  tracking_log_dir:=diagnostics/research_baseline_contact_pair_attribution_v1
```

## Result

The run aborted safely in `MOVING_TO_START`, not `APPROACH`:

- outcome: `ABORTED`
- reason: hard-force abort in `MOVING_TO_START`
- raw `|Fz|`: `1018.9 N`
- raw force norm: `1195.8 N`
- insertion depth: `0.0000 m`
- phase duration before abort: `57.6 s`
- phase Cartesian error at abort: `0.180443 m`

Contact attribution is decisive:

```text
lbr_iisy6_r1300::link_5::link_5_collision <-> target_plate::plate_link::target_plate_collision
```

The contact observer recorded this target-source collision pair in both `MOVING_TO_START` and `ABORT`:

- `MOVING_TO_START`: 230 samples, max contact force `6451.232097 N`
- `ABORT`: 496 samples, max contact force `8886.018914 N`

No peg-source or hole-source rows were recorded in this run.

## Evidence Files

- `trial_outcome.json`
- `contact_state_summary.md`
- `wrench_state_summary.md`
- `trajectory_tracking_summary.md`
- `contact_state_samples.csv` (raw, intentionally not staged)
- `wrench_state_samples.csv` (raw, intentionally not staged)
- `trajectory_tracking_samples.csv` (raw, intentionally not staged)
- `trajectory_commands.csv` (raw, intentionally not staged)

## Conclusion

The unsafe force/contact in this reproduced doubled-effort run is a robot-link clearance collision: `link_5_collision` hits the target plate while moving toward the above-hole posture. This is not valid insertion contact and must not be counted as task progress.

The next safety-critical milestone is clearance-aware motion/geometry validation for the `MOVING_TO_START` path and target fixture, with the preserved no-contact descent gate and hard-force abort unchanged.
