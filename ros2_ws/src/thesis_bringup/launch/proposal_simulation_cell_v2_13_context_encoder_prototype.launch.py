from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("thesis_bringup"), "config", "proposal_simulation_cell_v2_13.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/proposal_simulation_cell_v2_13",
                description="Directory for proposal simulation cell v2.13 diagnostics.",
            ),
            Node(
                package="thesis_bringup",
                executable="proposal_simulation_cell_v2_13_context_encoder_node",
                name="proposal_simulation_cell_v2_13_context_encoder_node",
                output="screen",
                parameters=[{"config_path": config_path}, {"output_dir": LaunchConfiguration("output_dir")}],
            ),
        ]
    )
