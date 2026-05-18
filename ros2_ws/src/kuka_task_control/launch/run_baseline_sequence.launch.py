from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("kuka_task_control"))
    config_path = package_share / "config" / "baseline_task_poses.yaml"

    return LaunchDescription(
        [
            Node(
                package="kuka_task_control",
                executable="baseline_joint_sequence_executor",
                name="baseline_joint_sequence_executor",
                output="screen",
                parameters=[{"config_path": str(config_path)}],
            )
        ]
    )
