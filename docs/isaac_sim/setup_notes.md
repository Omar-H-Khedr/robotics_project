# Isaac Sim Setup Notes

## Workstation
- Windows 11 Pro
- NVIDIA RTX 2080 Ti
- Isaac Sim 5.1.0 standalone
- ROS 2 Jazzy running inside WSL Ubuntu 24.04

## Completed
- Isaac Sim launched successfully.
- ROS 2 Bridge enabled.
- ROS 2 in WSL detected Isaac Sim /clock topic.

## Test Commands

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /clock --once


