from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("thesis_bringup"), "config", "proposal_simulation_cell_v1_15.yaml"]
    )
    generator = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_15_evidence_package_node",
        name="proposal_simulation_cell_v1_15_evidence_package_node",
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
                default_value="diagnostics/proposal_simulation_cell_v1_15",
                description="Directory for proposal simulation cell v1.15 diagnostics.",
            ),
            LogInfo(
                msg=(
                    "proposal_simulation_cell_v1_15_evidence_package_generator: evidence package "
                    "from existing diagnostics only. No scenario execution, datasets, plots, results, "
                    "motion, MoveIt, /compute_ik, controllers, FollowJointTrajectory, or command execution."
                )
            ),
            generator,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=generator,
                    on_exit=[EmitEvent(event=Shutdown(reason="v1.15 evidence package captured"))],
                )
            ),
        ]
    )
