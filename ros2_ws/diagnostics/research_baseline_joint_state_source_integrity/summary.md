# Research Baseline Joint-State Source Integrity

Date: 2026-06-02

## Objective

Make the canonical `research_baseline.launch.py` use one ROS 2 joint-state source. The shared KUKA Gazebo bridge config maps `/joint_states` from Gazebo, while the research baseline also starts `joint_state_broadcaster`. That can corrupt task feedback and tracking metrics.

## Change

- Added `thesis_bringup/config/research_baseline_bridge.yaml`.
- Updated `research_baseline.launch.py` to use the project-local bridge config.
- Omitted `/joint_states` from the canonical research baseline bridge.
- Kept `joint_state_broadcaster` as the only intended `/joint_states` publisher for task logic and diagnostics.
- Kept canonical robot renaming disabled so deterministic F/T bridge paths continue to use `lbr_iisy6_r1300`.

## Verification Commands

```bash
python3 -m py_compile src/thesis_bringup/launch/research_baseline.launch.py
source /opt/ros/jazzy/setup.bash
colcon build --packages-select thesis_bringup --symlink-install
source install/setup.bash
timeout 90s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false
timeout 10s ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
timeout 10s ros2 node info /joint_state_broadcaster
```

## Result

- Syntax check passed.
- Targeted `thesis_bringup` build passed.
- Headless Gazebo launch started with canonical robot name `lbr_iisy6_r1300`.
- `research_baseline_ros_gz_bridge` created bridges for `/clock`, `/cmd_vel`, and D405 topics only; it did not create a `/joint_states` bridge.
- `joint_state_broadcaster` and `joint_trajectory_controller` both activated.
- `/joint_states` sample contained named joints `joint_1` through `joint_6`.
- `ros2 node info /joint_state_broadcaster` listed `/joint_states` as a publisher.

## Runtime Limitation

The trial did not complete insertion. It remained in `MOVING_TO_START` and timed out under the 90 s launch guard with large no-contact tracking error. Observed late status:

```text
MOVING_TO_START t=70.0s peg=(0.466, -0.163, 0.915) cart_err=0.072 xy_err=0.066 stable=0/5
```

This is a safe bounded failure because the controller did not descend while XY alignment was outside the 2 mm gate. The next milestone remains controller/target tracking stabilization, not loosening the alignment gate.
