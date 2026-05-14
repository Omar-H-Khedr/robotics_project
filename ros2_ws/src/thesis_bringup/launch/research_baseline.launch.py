"""Phase 1 research baseline: start the KUKA Gazebo simulation stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


KUKA_GAZEBO_LAUNCH_FILE = "gazebo_startup.launch.py"


def generate_launch_description():
    """Launch the existing KUKA Gazebo startup system for Phase 1 research."""
    kuka_gazebo_launch = os.path.join(
        get_package_share_directory("kuka_gazebo"),
        "launch",
        KUKA_GAZEBO_LAUNCH_FILE,
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Phase 1 research baseline: including "
                    f"kuka_gazebo/launch/{KUKA_GAZEBO_LAUNCH_FILE}"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(kuka_gazebo_launch),
            ),
        ]
    )
