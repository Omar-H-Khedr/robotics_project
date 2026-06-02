# Research Baseline Cell Model Consistency

Date: 2026-06-02

## Purpose

Validate the current KUKA LBR iisy 6 R1300 workcell model/configuration consistency before further controller or insertion tuning.

This milestone is not an insertion-success claim. It checks that the canonical robot, task targets, D405 topics, world SDF, and headless launch wiring are internally consistent enough to keep investigating the remaining tracking blocker.

## Source Changes Validated

- `peg_hole_cartesian_targets.yaml` now uses the same hole-top convention as the task controller: `hole_center.z = 0.810`, above-hole/staging z = `0.885`, touch z = `0.830`, final insertion z = `0.790`.
- `research_baseline.yaml` names the canonical robot as `KUKA LBR iisy 6 R1300`.
- `rgbd_pipeline.yaml` subscribes to the D405 topics bridged by the canonical baseline: `/d405/color/image_raw`, `/d405/depth/image_rect_raw`, and `/d405/color/camera_info`.
- `peg_in_hole_world.sdf` uses valid SDF box syntax for the D405 visual.
- `proposal_lbr_iisy6_r1300_cell.urdf.xacro` is explicitly marked deprecated because it is a cylinder placeholder, not a kinematic KUKA model.
- `research_parallel_gripper.xacro` supports the fixed grasped 25 mm peg and `peg_tip` frame.
- `lbr_iisy6_r1300_research_gripper.urdf.xacro` disables the optional TF-only camera by default so standalone robot URDF-to-SDF conversion remains self-contained; the real simulated D405 sensor remains in the Gazebo world.

## Verification Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

xacro src/peg_in_hole_description/urdf/lbr_iisy6_r1300_research_gripper.urdf.xacro mode:=gazebo x:=0.80 y:=-0.75 z:=0.735 yaw:=1.5708 > /tmp/lbr_iisy6_research_gripper_default.urdf
xacro src/peg_in_hole_description/urdf/proposal_lbr_iisy6_r1300_cell.urdf.xacro > /tmp/proposal_deprecated_cell.urdf
export GZ_SIM_RESOURCE_PATH=/home/omar/code/robotics_project/ros2_ws/src/peg_in_hole_description/models
export SDF_PATH=/home/omar/code/robotics_project/ros2_ws/src/peg_in_hole_description/models
gz sdf -k src/peg_in_hole_description/worlds/peg_in_hole_world.sdf
gz sdf -p /tmp/lbr_iisy6_research_gripper_default.urdf >/tmp/lbr_iisy6_research_gripper_default.sdf
python3 -m py_compile src/thesis_bringup/launch/research_baseline.launch.py src/thesis_bringup/thesis_bringup/spawn_robot_sdf.py
colcon build --symlink-install --packages-select peg_in_hole_description perception_pipeline thesis_bringup kuka_task_control
timeout 90s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false
```

## Results

- Robot wrapper Xacro expanded successfully.
- Deprecated cylinder placeholder Xacro still expanded successfully for legacy review.
- `gz sdf -k` reported the canonical world as `Valid` when the local model path was set.
- Robot URDF converted to SDF successfully with the default `include_camera:=false`.
- Python syntax check passed for the launch file and `spawn_robot_sdf.py`.
- Targeted build passed for `peg_in_hole_description`, `perception_pipeline`, `thesis_bringup`, and `kuka_task_control`.
- Headless launch spawned `lbr_iisy6_r1300`, started D405 bridges, started F/T bridge, loaded `gz_ros2_control`, and activated both `joint_state_broadcaster` and `joint_trajectory_controller`.
- The D405 sensor advertised `/d405/color/image_raw`, `/d405/color/camera_info`, `/d405/depth/image_rect_raw`, and `/d405/depth/camera_info`.

## Remaining Limitation

The launch timed out after 90 s in `MOVING_TO_START`; this is expected for the current tracking blocker and is not a regression. Observed XY error improved to about `0.027 m` at 60 s but then drifted to about `0.070 m` by 75 s, still far outside the preserved `0.002 m` no-contact descent gate.

The DART mesh-collision warning remains visible for the KUKA mesh collision geometry:

```text
Mesh construction from an SDF has not been implemented yet for dartsim.
The geometry element of collision [link_*_collision] couldn't be created.
```

This should be investigated as part of the next tracking/physics milestone because it may affect physical credibility, although it has not yet been proven to be the sole cause of the above-hole tracking drift.
