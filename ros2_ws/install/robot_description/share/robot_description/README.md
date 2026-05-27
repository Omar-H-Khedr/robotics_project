# Robot Description

This is a beginner-friendly ROS 2 package that describes a simple two-wheel differential drive robot.

The model has:

- `base_footprint`: an inertialess root frame at the robot footprint
- `base_link`: the main rectangular body, fixed to `base_footprint` and carrying the body inertia
- `left_wheel`: the left drive wheel
- `right_wheel`: the right drive wheel
- `caster_wheel`: a small passive support wheel

## Frame Structure

`base_footprint` is the root link of the robot model. It is a dummy link with no visual,
collision, or inertial properties. For a mobile robot, this frame represents the robot's
2D footprint on the ground plane and is a convenient frame for navigation, odometry, and
simulation.

`base_link` is the physical body of the robot. The body visual, collision, and inertial
properties stay on `base_link`, because this is the link that represents the real robot
mass in the model.

The fixed joint from `base_footprint` to `base_link` separates the navigation frame from
the physical body frame. This avoids putting inertia on the root link while keeping the
robot easy to reason about in RViz and ready for later simulation work.

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
