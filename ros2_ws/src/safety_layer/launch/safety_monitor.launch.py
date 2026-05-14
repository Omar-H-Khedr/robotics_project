from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("safety_layer"))
    config_path = package_share / "config" / "safety_limits.yaml"

    return LaunchDescription(
        [
            Node(
                package="safety_layer",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
                parameters=[{"config_path": str(config_path)}],
            )
        ]
    )
