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
    world_path = os.path.join(peg_share, "worlds", "proposal_simulation_cell_v1_0.world.sdf")

    isaac_available = shutil.which("isaac-sim") is not None or shutil.which("isaacsim") is not None
    gazebo_fallback_used = not isaac_available

    config_path = PathJoinSubstitution(
        [
            FindPackageShare("thesis_bringup"),
            "config",
            "proposal_simulation_cell_v1_1.yaml",
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
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="proposal_simulation_cell_v1_1_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/proposal_simulation_cell/d405/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/proposal_simulation_cell/d405/depth/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="proposal_lbr_iisy6_r1300_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    validator = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_1_validator",
        name="proposal_simulation_cell_v1_1_validator",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"output_dir": LaunchConfiguration("output_dir")},
            {"isaac_available": isaac_available},
            {"gazebo_fallback_used": gazebo_fallback_used},
        ],
    )

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        LogInfo(
            msg=(
                "proposal_simulation_cell_v1_1_sensor_and_scene_validation: "
                "using ROS 2/Gazebo fallback when Isaac Sim is unavailable. "
                "No MoveIt, /compute_ik, real robot, controller execution, "
                "or FollowJointTrajectory is used."
            )
        ),
        gazebo,
        bridge,
        robot_state_publisher,
        validator,
        RegisterEventHandler(
            OnProcessExit(
                target_action=validator,
                on_exit=[EmitEvent(event=Shutdown(reason="v1.1 validation evidence captured"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/proposal_simulation_cell_v1_1",
                description="Directory for proposal simulation cell v1.1 diagnostics.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
