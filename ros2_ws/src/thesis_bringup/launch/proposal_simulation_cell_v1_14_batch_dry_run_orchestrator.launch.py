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
            "proposal_simulation_cell_v1_14.yaml",
        ]
    )
    orchestrator = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_14_batch_dry_run_orchestrator_node",
        name="proposal_simulation_cell_v1_14_batch_dry_run_orchestrator_node",
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
                default_value="diagnostics/proposal_simulation_cell_v1_14",
                description="Directory for proposal simulation cell v1.14 diagnostics.",
            ),
            LogInfo(
                msg=(
                    "proposal_simulation_cell_v1_14_batch_dry_run_orchestrator: "
                    "configuration-only blocked dry-run orchestration. No scenario execution, datasets, plots, "
                    "results, motion, MoveIt, /compute_ik, controllers, FollowJointTrajectory, or command execution."
                )
            ),
            orchestrator,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=orchestrator,
                    on_exit=[EmitEvent(event=Shutdown(reason="v1.14 batch dry-run orchestration captured"))],
                )
            ),
        ]
    )
