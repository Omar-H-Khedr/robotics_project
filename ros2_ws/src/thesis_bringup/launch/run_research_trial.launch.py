from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting Research Baseline v0.1: Gazebo baseline, "
                    "safety monitor, and trial logger. Launch the task sequence "
                    "separately with: ros2 launch kuka_task_control "
                    "run_task_sequence.launch.py"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("thesis_bringup"),
                            "launch",
                            "research_baseline.launch.py",
                        ]
                    )
                )
            ),
            TimerAction(
                period=5.0,
                actions=[
                    LogInfo(
                        msg=(
                            "Research Baseline v0.1: baseline launch has been "
                            "included; starting safety monitor and trial logger "
                            "after 5 seconds so Gazebo and KUKA spawning are not "
                            "blocked by auxiliary nodes."
                        )
                    ),
                    Node(
                        package="safety_layer",
                        executable="safety_monitor",
                        name="safety_monitor",
                        output="screen",
                        parameters=[
                            {
                                "config_path": PathJoinSubstitution(
                                    [
                                        FindPackageShare("safety_layer"),
                                        "config",
                                        "safety_limits.yaml",
                                    ]
                                )
                            }
                        ],
                    ),
                    LogInfo(
                        msg=(
                            "Research Baseline v0.1: starting "
                            "experiment_manager/baseline_trial_manager for "
                            "non-blocking baseline trial logging."
                        )
                    ),
                    Node(
                        package="experiment_manager",
                        executable="baseline_trial_manager",
                        name="baseline_trial_manager",
                        output="screen",
                    ),
                ],
            ),
        ]
    )
