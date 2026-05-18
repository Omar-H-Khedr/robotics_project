from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _workspace_results_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == "ros2_ws":
            return parent / "results" / "baseline_trials"
    return Path.cwd() / "results" / "baseline_trials"


def generate_launch_description():
    results_root = _workspace_results_root()

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
