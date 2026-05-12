# Robot Description

This is a beginner-friendly ROS 2 package that describes a simple two-wheel differential drive robot.

The model has:

- `base_link`: the main rectangular body
- `left_wheel`: the left drive wheel
- `right_wheel`: the right drive wheel
- `caster_wheel`: a small passive support wheel

## Files

- `urdf/simple_diff_drive.urdf`: the robot model
- `launch/display.launch.py`: starts the display tools
- `rviz/display.rviz`: a simple RViz setup

## Build

From the ROS 2 workspace:

```bash
cd ~/code/robotics_project/ros2_ws
colcon build --packages-select robot_description
source install/setup.bash
```

## Run

```bash
ros2 launch robot_description display.launch.py
```

This starts:

- `robot_state_publisher`, which publishes the robot transforms
- `joint_state_publisher_gui`, which gives sliders for movable joints
- `rviz2`, which shows the robot model

Move the wheel joint sliders in the GUI to see the wheels rotate in RViz.
