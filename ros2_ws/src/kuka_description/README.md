# kuka_description

Project-local ROS 2 package for launching KUKA robot descriptions.

The actual KUKA LBR iisy URDF/Xacro, mesh, and joint limit files live in the
external package:

```text
ros2_ws/src/external/kuka_robot_descriptions/kuka_lbr_iisy_support
```

This package keeps our own launch and RViz configuration separate from the
external KUKA files.

## Available LBR iisy models

The external `kuka_lbr_iisy_support` package currently provides these URDF
entrypoints:

- `lbr_iisy3_r760.urdf.xacro`
- `lbr_iisy11_r1300.urdf.xacro`
- `lbr_iisy15_r930.urdf.xacro`

Each model has a corresponding macro xacro, visual meshes, collision meshes,
and joint limit configuration in the same external package.

## Build

From the workspace root:

```bash
cd ros2_ws
colcon build --packages-up-to kuka_description
source install/setup.bash
```

## Run RViz visualization

Default model, `lbr_iisy3_r760`:

```bash
ros2 launch kuka_description display_lbr_iisy.launch.py
```

Select another supported model:

```bash
ros2 launch kuka_description display_lbr_iisy.launch.py model:=lbr_iisy11_r1300
ros2 launch kuka_description display_lbr_iisy.launch.py model:=lbr_iisy15_r930
```

The launch file starts:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`

The xacro is expanded with `mode:=mock`, matching the upstream visualization
launch files and avoiding any connection to real hardware.
