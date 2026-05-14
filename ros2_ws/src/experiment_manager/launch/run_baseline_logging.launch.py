from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    results_root = (
        Path.home()
        / "code"
        / "robotics_project"
        / "ros2_ws"
        / "results"
        / "baseline_trials"
    )

    return LaunchDescription(
        [
            Node(
                package="experiment_manager",
                executable="baseline_trial_manager",
                name="baseline_trial_manager",
                output="screen",
                parameters=[{"results_root": str(results_root)}],
            )
        ]
    )
