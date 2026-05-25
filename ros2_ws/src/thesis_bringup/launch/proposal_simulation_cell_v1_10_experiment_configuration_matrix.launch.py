from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [
            FindPackageShare("thesis_bringup"),
            "config",
            "proposal_simulation_cell_v1_10.yaml",
        ]
    )
    experiment_matrix = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_10_experiment_matrix_node",
        name="proposal_simulation_cell_v1_10_experiment_matrix_node",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"output_dir": LaunchConfiguration("output_dir")},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/proposal_simulation_cell_v1_10",
                description="Directory for proposal simulation cell v1.10 diagnostics.",
            ),
            LogInfo(
                msg=(
                    "proposal_simulation_cell_v1_10_experiment_configuration_matrix: "
                    "configuration-only scenario matrix. No datasets, plots, results, motion, "
                    "MoveIt, /compute_ik, controllers, FollowJointTrajectory, or command execution."
                )
            ),
            experiment_matrix,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=experiment_matrix,
                    on_exit=[EmitEvent(event=Shutdown(reason="v1.10 experiment configuration matrix captured"))],
                )
            ),
        ]
    )
