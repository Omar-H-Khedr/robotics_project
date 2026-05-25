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
            "proposal_simulation_cell_v1_12.yaml",
        ]
    )
    selector = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_12_scenario_batch_selector_node",
        name="proposal_simulation_cell_v1_12_scenario_batch_selector_node",
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
                default_value="diagnostics/proposal_simulation_cell_v1_12",
                description="Directory for proposal simulation cell v1.12 diagnostics.",
            ),
            LogInfo(
                msg=(
                    "proposal_simulation_cell_v1_12_scenario_batch_selector: "
                    "configuration-only representative scenario batch validation. No datasets, plots, "
                    "results, motion, MoveIt, /compute_ik, controllers, FollowJointTrajectory, or command execution."
                )
            ),
            selector,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=selector,
                    on_exit=[EmitEvent(event=Shutdown(reason="v1.12 scenario batch selector captured"))],
                )
            ),
        ]
    )
