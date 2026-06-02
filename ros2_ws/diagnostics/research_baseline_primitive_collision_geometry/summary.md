# Research Baseline Primitive Collision Geometry

Date: 2026-06-02

## Purpose

Address the Gazebo/DART physics-credibility warning where KUKA arm STL mesh collisions were rejected at runtime:

```text
Mesh construction from an SDF has not been implemented yet for dartsim.
The geometry element of collision [link_*_collision] couldn't be created.
```

This milestone does not claim peg-in-hole success. It makes the canonical research baseline use DART-loadable arm collision geometry so later contact and tracking experiments are less compromised by missing robot link collisions.

## Implementation

- Added configurable `collision_geometry:=mesh` support to the project-local `kuka_lbr_iisy6_r1300_robot` macro.
- Kept mesh collision support as the macro default for compatibility.
- Changed the canonical `lbr_iisy6_r1300_research_gripper.urdf.xacro` wrapper to request `collision_geometry="primitive"`.
- Represented `base_link` through `link_6` with simple cylinders/boxes for Gazebo collision loading.
- Kept robot visuals as meshes.
- Pointed the iisy6 macro at the existing iisy11 R1300 mesh assets, with an explicit source comment, because earlier project evidence treats the iisy11 R1300 geometry as the 1300 mm proxy while the iisy6-specific assets are not available in the submodule.

## Verification Commands

```bash
cd /home/omar/code/robotics_project/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

xacro src/peg_in_hole_description/urdf/lbr_iisy6_r1300_research_gripper.urdf.xacro mode:=gazebo x:=0.80 y:=-0.75 z:=0.735 yaw:=1.5708 > /tmp/lbr_iisy6_primitive_collision.urdf
gz sdf -p /tmp/lbr_iisy6_primitive_collision.urdf >/tmp/lbr_iisy6_primitive_collision.sdf
rg -n "meshes/lbr_iisy6_r1300|meshes/lbr_iisy11_r1300|<mesh|<collision>|<box|<cylinder" /tmp/lbr_iisy6_primitive_collision.urdf
colcon build --symlink-install --packages-select kuka_lbr_iisy_support peg_in_hole_description thesis_bringup
timeout 90s ros2 launch thesis_bringup research_baseline.launch.py use_gui:=false
```

## Results

- Xacro expansion passed.
- URDF-to-SDF conversion passed.
- Generated URDF shows primitive collisions for `base_link` through `link_6`.
- Generated URDF keeps visual meshes and points them at existing `lbr_iisy11_r1300` mesh assets.
- Targeted build passed for `kuka_lbr_iisy_support`, `peg_in_hole_description`, and `thesis_bringup`.
- Headless launch spawned `lbr_iisy6_r1300`, loaded `gz_ros2_control`, activated `joint_state_broadcaster`, activated `joint_trajectory_controller`, and started the task node.
- The prior DART mesh-collision rejection messages for KUKA arm link collisions were not observed in the launch console output.

## Runtime Outcome

The physics warning fix changed the trial behavior but did not solve insertion:

- `MOVING_TO_START` eventually reached the strict XY gate: `xy_error=0.0006 m`, `cart_err=0.011 m`, `joint_err=0.056 rad`.
- The node transitioned from `MOVING_TO_START` to `APPROACH`.
- `APPROACH` did not stabilize before the 90 s timeout; late observed approach Cartesian error remained around `0.039 m`.

This is a useful tracking improvement and a physics-credibility fix, but still not a physical insertion success. The next blocker is now approach/descent tracking stability after valid above-hole alignment.
