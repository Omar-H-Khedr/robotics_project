from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("thesis_bringup"), "config", "proposal_simulation_cell_v1_17.yaml"]
    )
    release_index = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_17_release_index_node",
        name="proposal_simulation_cell_v1_17_release_index_node",
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
                default_value="diagnostics/proposal_simulation_cell_v1_17",
                description="Directory for proposal simulation cell v1.17 diagnostics.",
            ),
            LogInfo(
                msg=(
                    "proposal_simulation_cell_v1_17_release_documentation_index: documentation "
                    "verification only. No scenario execution, datasets, plots, results, motion, "
                    "MoveIt, /compute_ik, controllers, FollowJointTrajectory, or real robot execution."
                )
            ),
            release_index,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=release_index,
                    on_exit=[EmitEvent(event=Shutdown(reason="v1.17 release documentation index captured"))],
                )
            ),
        ]
    )
