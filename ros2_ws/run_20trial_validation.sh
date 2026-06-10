#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/omar/code/robotics_project/ros2_ws/install/setup.bash
cd /home/omar/code/robotics_project/ros2_ws
export VALIDATION_TRIALS=20
exec python3 src/perception_pipeline/perception_pipeline/test_perception_logger_reliability.py
