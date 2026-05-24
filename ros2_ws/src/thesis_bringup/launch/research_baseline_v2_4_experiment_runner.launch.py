from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [
            FindPackageShare("thesis_bringup"),
            "config",
            "research_baseline_v2_4_experiment_config.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/research_baseline_v2_4",
                description="Directory for v2.4 dry-run experiment artifacts.",
            ),
            DeclareLaunchArgument(
                "run_batch",
                default_value="false",
                description="When true, generate randomized batch dry-run results.",
            ),
            DeclareLaunchArgument(
                "batch_size",
                default_value="20",
                description="Number of randomized dry-run trials for batch mode.",
            ),
            DeclareLaunchArgument(
                "seed",
                default_value="24",
                description="Random seed for repeatable batch offsets.",
            ),
            LogInfo(
                msg=(
                    "Starting research_baseline_v2_4_experiment_runner in "
                    "diagnostic dry-run mode. No MoveIt, /compute_ik, real robot, "
                    "Gazebo motion, controller execution, or FollowJointTrajectory "
                    "command is used."
                )
            ),
            Node(
                package="experiment_manager",
                executable="research_baseline_v2_4_experiment_runner",
                name="research_baseline_v2_4_experiment_runner",
                output="screen",
                parameters=[
                    {"config_path": config_path},
                    {"output_dir": LaunchConfiguration("output_dir")},
                    {
                        "run_batch": ParameterValue(
                            LaunchConfiguration("run_batch"),
                            value_type=bool,
                        )
                    },
                    {
                        "batch_size": ParameterValue(
                            LaunchConfiguration("batch_size"),
                            value_type=int,
                        )
                    },
                    {
                        "seed": ParameterValue(
                            LaunchConfiguration("seed"),
                            value_type=int,
                        )
                    },
                ],
            ),
        ]
    )
