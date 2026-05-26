from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("thesis_bringup"), "config", "proposal_simulation_cell_v2_12.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/proposal_simulation_cell_v2_12",
                description="Directory for proposal simulation cell v2.12 diagnostics.",
            ),
            Node(
                package="thesis_bringup",
                executable="proposal_simulation_cell_v2_12_context_vector_extraction_node",
                name="proposal_simulation_cell_v2_12_context_vector_extraction_node",
                output="screen",
                parameters=[{"config_path": config_path}, {"output_dir": LaunchConfiguration("output_dir")}],
            ),
        ]
    )
