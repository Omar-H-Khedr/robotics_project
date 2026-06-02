# Research Baseline ROS 2 Control Config

Date: 2026-06-02

## Milestone

`research_baseline_ros2_control_config`

## Purpose

Make the canonical research baseline own its Gazebo `ros2_control` controller
parameters instead of hard-coding the upstream `kuka_resources` fake-hardware
YAML in `spawn_robot_sdf.py`.

This is a controller/physics stabilization milestone. It does not loosen the
2 mm no-contact above-hole gate and does not claim insertion success.

## Implementation

- Added `thesis_bringup/config/research_baseline_ros2_control.yaml`.
- Set `controller_manager.update_rate` to 250 Hz for the research baseline.
- Set `joint_trajectory_controller.state_publish_rate` to 100 Hz.
- Set `joint_trajectory_controller.action_monitor_rate` to 50 Hz.
- Set `allow_nonzero_velocity_at_trajectory_end` to `false`.
- Added `--controller-config-package` and `--controller-config-path` to
  `spawn_robot_sdf.py`, retaining the upstream KUKA config as the fallback.
- Updated `research_baseline.launch.py` to pass the project-local controller
  config by default.

## Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
python3 -m py_compile src/thesis_bringup/thesis_bringup/spawn_robot_sdf.py src/thesis_bringup/launch/research_baseline.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select thesis_bringup
mkdir -p /tmp/ros2_logs /tmp/gz_home
export ROS_LOG_DIR=/tmp/ros2_logs
export HOME=/tmp/gz_home
source install/setup.bash
timeout 120s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false
```

## Validation Result

- Python syntax checks passed.
- Targeted `thesis_bringup` build passed.
- Headless Gazebo launched `peg_in_hole_world.sdf`.
- `lbr_iisy6_r1300` spawned successfully.
- D405 and F/T bridges started.
- `joint_state_broadcaster` and `joint_trajectory_controller` activated.
- Controller logs showed both controllers using:
  `/home/omar/code/robotics_project/ros2_ws/install/thesis_bringup/share/thesis_bringup/config/research_baseline_ros2_control.yaml`.
- The controller update warning changed to 0.004 s, matching the 250 Hz
  project-local config.

## Runtime Outcome

The task remained a bounded failure:

- `Outcome: ABORTED`
- `Reason: MOVING_TO_START timeout/failure (90.0s)`
- `cart_err=0.015 m`
- `xy_err=0.011 m`
- `joint_err=0.018 rad`
- `stable=0/5`
- `Depth: 0.0000 m`
- `Max Fz: 171.1 N`
- `Baseline: 64.5 N`
- `Contact: 114.6 N`

The task did not enter descent, contact search, or insertion. This is correct
safe behavior because the strict above-hole stability gate was not satisfied.

## Interpretation

The controller configuration ambiguity is now removed from the canonical
launch. The new config improves ownership and reduces the old 50 Hz mismatch,
but it does not solve stable above-hole convergence. The current blocker is
still tracking/hold stability near the above-hole pose, plus investigation of
remaining free-space F/T behavior.

The next milestone should measure commanded-versus-actual trajectory tracking
and tune trajectory timing/hold behavior or controller/physics parameters from
that evidence without weakening the no-contact safety gates.
