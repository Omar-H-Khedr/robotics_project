# Research Baseline Trajectory Tracking Observer

Date: 2026-06-02

## Milestone

`research_baseline_trajectory_tracking_observer`

## Purpose

Add passive commanded-versus-actual tracking evidence for the canonical KUKA
LBR iisy 6 R1300 Gazebo baseline. This milestone investigates why the strict
above-hole gate remains unsatisfied without weakening any safety gates.

## Implementation

- Added `thesis_bringup.trajectory_tracking_observer`.
- Subscribes passively to:
  - `/joint_trajectory_controller/joint_trajectory`;
  - `/joint_states`;
  - `/joint_trajectory_controller/state` for availability counting.
- Interpolates the active `JointTrajectory` command by joint name and compares
  it against named `/joint_states`.
- Writes:
  - `trajectory_tracking_summary.md`;
  - `trajectory_commands.csv`;
  - `trajectory_tracking_samples.csv`.
- The observer does not publish commands and does not alter task behavior.

## Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
python3 -m py_compile src/thesis_bringup/thesis_bringup/trajectory_tracking_observer.py src/thesis_bringup/launch/research_baseline.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select thesis_bringup
mkdir -p /tmp/ros2_logs /tmp/gz_home diagnostics/research_baseline_trajectory_tracking
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 150s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false tracking_log_dir:=diagnostics/research_baseline_trajectory_tracking
```

## Validation Result

- Python syntax checks passed.
- Targeted `thesis_bringup` build passed.
- Headless Gazebo launched and spawned `lbr_iisy6_r1300`.
- D405 and F/T bridges started.
- `joint_state_broadcaster` and `joint_trajectory_controller` activated.
- The trajectory observer started and logged the task command topic against
  named `/joint_states`.

## Runtime Outcome

The task remained a bounded safety failure:

- `Outcome: ABORTED`;
- `Reason: MOVING_TO_START timeout/failure (90.0s)`;
- `cart_err=0.014 m`;
- `xy_err=0.010 m`;
- `joint_err=0.014 rad`;
- `stable=0/5`;
- `Depth: 0.0000 m`;
- `Max Fz: 168.6 N`;
- no descent, contact search, or insertion was attempted.

## Tracking Evidence

Compact committed evidence:

- `trajectory_tracking_summary.md`;
- `trajectory_commands.csv`;
- this `summary.md`.

The raw local CSV `trajectory_tracking_samples.csv` contained 15,893 samples
and was 4.18 MB. It was not staged in this commit to avoid adding a large raw
generated artifact; the summary records the key metrics:

- observed trajectory commands: 2;
- direct JTC state samples: 0;
- command-vs-joint-state samples: 15,893;
- max absolute position error: 0.045250 rad;
- mean max absolute position error: 0.015565 rad;
- p95 max absolute position error: 0.024368 rad;
- final max absolute position error: 0.014106 rad.

## Interpretation

The previous controller-state topic was discoverable but did not deliver
samples during runtime. The new observer provides useful tracking evidence from
the task command stream and the canonical named `/joint_states` source.

The failed gate is consistent with residual joint tracking/hold error of about
0.014 rad near timeout and Cartesian XY error around 0.010 m. The next control
milestone should improve trajectory timing and final hold stabilization from
this measured tracking evidence, while preserving the strict 2 mm no-contact
descent gate.
