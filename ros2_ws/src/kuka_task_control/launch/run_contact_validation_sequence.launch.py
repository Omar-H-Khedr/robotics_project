from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("kuka_task_control"))
    config_path = package_share / "config" / "contact_validation_sequence.yaml"

    return LaunchDescription(
        [
            Node(
                package="kuka_task_control",
                executable="task_trajectory_executor",
                name="task_trajectory_executor",
                output="screen",
                parameters=[
                    {
                        "task_sequence_file": str(config_path),
                    }
                ],
            )
        ]
    )
