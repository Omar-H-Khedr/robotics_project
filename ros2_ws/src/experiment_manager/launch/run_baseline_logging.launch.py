from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="experiment_manager",
                executable="baseline_trial_manager",
                name="baseline_trial_manager",
                output="screen",
            )
        ]
    )
