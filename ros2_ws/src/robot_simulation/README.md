# robot_simulation

Beginner-friendly Gazebo simulation package for the `simple_diff_drive` robot.

This package starts Gazebo with an empty world and spawns the robot from:

```text
ros2_ws/src/robot_description/urdf/simple_diff_drive.urdf
```

At runtime, the launch file uses the installed `robot_description` package share
directory, so build and source the workspace before launching.

## Build the workspace

From the workspace root:

```bash
cd ~/code/robotics_project/ros2_ws
colcon build
source install/setup.bash
```

If you already have another ROS 2 environment open, source ROS 2 Jazzy first:

```bash
source /opt/ros/jazzy/setup.bash
```

## Launch Gazebo

```bash
ros2 launch robot_simulation gazebo.launch.py
```

## Command the robot

The URDF includes the Gazebo Sim DiffDrive system. It listens on `/cmd_vel`,
publishes odometry on `/odom`, and uses `odom` as the odometry frame with
`base_link` as the child frame.

Send a simple forward velocity command with:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

## Expected result

Gazebo opens with the empty world, a ground plane, and the simple differential
drive robot spawned near the center of the world.
