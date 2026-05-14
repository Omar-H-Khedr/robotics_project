# Gazebo Strategy

Gazebo is the canonical simulator for the KUKA peg-in-hole PhD framework because it supports ROS 2 launch integration, ros2_control controllers, versioned task geometry, contact-rich scenes, and later RGB-D sensor simulation.

## Baseline v0.1 Scope

The v0.1 Gazebo baseline uses `thesis_bringup/launch/research_baseline.launch.py` to launch the peg-in-hole world, spawn the pedestal-mounted KUKA LBR iisy 3 R760 with a simplified gripper, and activate `joint_state_broadcaster` plus `joint_trajectory_controller`.

`thesis_bringup/launch/run_research_trial.launch.py` composes that simulation with the monitor-only safety layer and trial logger. Task motion is started separately through `kuka_task_control/launch/run_task_sequence.launch.py`.

## Controller Strategy

The reliable robot motion interface is:

```text
/joint_trajectory_controller/follow_joint_trajectory
```

The baseline controller must use `control_msgs/action/FollowJointTrajectory`. Direct topic publishing to the trajectory command topic is not used for publishable baseline trials.

## Contact Strategy

Contact and force metrics are intentionally deferred in v0.1. The next milestone is to identify stable Gazebo contact topics or plugins, filter expected peg-hole interaction from unexpected collision events, and validate maximum contact force extraction before using those values in published metrics.
