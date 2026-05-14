from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
            LogInfo(
                msg=(
                    "Starting Research Baseline v0.3: Gazebo baseline, "
                    "contact metrics, safety monitor, and trial logger. Launch "
                    "the task sequence separately from Terminal 2 with: ros2 launch "
                    "kuka_task_control run_task_sequence.launch.py"
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
                            "Research Baseline v0.3: starting safety monitor "
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
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    LogInfo(
                        msg=(
                            "Research Baseline v0.3: starting "
                            "experiment_manager/baseline_trial_manager for "
                            "non-blocking baseline trial logging. Terminal 2 command: "
                            "ros2 launch kuka_task_control run_task_sequence.launch.py"
                        )
                    ),
                    Node(
                        package="experiment_manager",
                        executable="baseline_trial_manager",
                        name="baseline_trial_manager",
                        output="screen",
                        parameters=[{"results_root": str(results_root)}],
                    ),
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    LogInfo(
                        msg=(
                            "Research Baseline v0.3: starting contact metrics "
                            "after 7 seconds."
                        )
                    ),
                    Node(
                        package="peg_in_hole_metrics",
                        executable="contact_metrics_node",
                        name="contact_metrics_node",
                        output="screen",
                        parameters=[
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("peg_in_hole_metrics"),
                                    "config",
                                    "contact_metrics.yaml",
                                ]
                            )
                        ],
                    ),
                ],
            ),
        ]
    )
