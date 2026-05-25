import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    peg_share = get_package_share_directory("peg_in_hole_description")
    model_path = os.path.join(peg_share, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = (
        model_path
        if not existing_resource_path
        else model_path + os.pathsep + existing_resource_path
    )
    world_path = os.path.join(
        peg_share,
        "worlds",
        "proposal_simulation_cell_v1_3_contact_physics_validation.world.sdf",
    )

    isaac_available = shutil.which("isaac-sim") is not None or shutil.which("isaacsim") is not None
    gazebo_fallback_used = not isaac_available
    ros_gz_bridge_available = shutil.which("parameter_bridge") is not None

    config_path = PathJoinSubstitution(
        [
            FindPackageShare("thesis_bringup"),
            "config",
            "proposal_simulation_cell_v1_5.yaml",
        ]
    )
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("peg_in_hole_description"),
                    "urdf",
                    "proposal_lbr_iisy6_r1300_cell.urdf.xacro",
                ]
            ),
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -s -r -v1"]}.items(),
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="proposal_simulation_cell_v1_5_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="proposal_lbr_iisy6_r1300_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    safety_node = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_5_safety_virtual_force_node",
        name="proposal_simulation_cell_v1_5_safety_virtual_force_node",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"output_dir": LaunchConfiguration("output_dir")},
            {"isaac_available": isaac_available},
            {"gazebo_fallback_used": gazebo_fallback_used},
            {"ros_gz_bridge_available": ros_gz_bridge_available},
            {"world_path": world_path},
        ],
    )

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        LogInfo(
            msg=(
                "proposal_simulation_cell_v1_5_safety_virtual_force_interface: "
                "simulation-only safety and virtual-force diagnostics. Command output "
                "and motion execution are disabled; no MoveIt, /compute_ik, controllers, "
                "real robot, or FollowJointTrajectory is used."
            )
        ),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        safety_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=safety_node,
                on_exit=[EmitEvent(event=Shutdown(reason="v1.5 safety virtual-force evidence captured"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/proposal_simulation_cell_v1_5",
                description="Directory for proposal simulation cell v1.5 diagnostics.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
