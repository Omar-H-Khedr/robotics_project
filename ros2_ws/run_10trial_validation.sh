#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /home/omar/code/robotics_project/ros2_ws/install/setup.bash
cd /home/omar/code/robotics_project/ros2_ws
exec python3 src/perception_pipeline/perception_pipeline/test_perception_logger_reliability.py
