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
    ros_gz_image_available = shutil.which("image_bridge") is not None

    config_path = PathJoinSubstitution(
        [
            FindPackageShare("thesis_bringup"),
            "config",
            "proposal_simulation_cell_v1_9.yaml",
        ]
    )
    upstream_v1_8_config_path = PathJoinSubstitution(
        [
            FindPackageShare("thesis_bringup"),
            "config",
            "proposal_simulation_cell_v1_8.yaml",
        ]
    )
    upstream_v1_7_config_path = PathJoinSubstitution(
        [
            FindPackageShare("thesis_bringup"),
            "config",
            "proposal_simulation_cell_v1_7.yaml",
        ]
    )
    upstream_v1_6_config_path = PathJoinSubstitution(
        [
            FindPackageShare("thesis_bringup"),
            "config",
            "proposal_simulation_cell_v1_6.yaml",
        ]
    )
    upstream_v1_5_config_path = PathJoinSubstitution(
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
        name="proposal_simulation_cell_v1_9_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/proposal_simulation_cell/d405/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/proposal_simulation_cell/d405/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="proposal_simulation_cell_v1_9_image_bridge",
        output="screen",
        arguments=[
            "/proposal_simulation_cell/d405/color/image_raw",
            "/proposal_simulation_cell/d405/depth/image_rect_raw",
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="proposal_lbr_iisy6_r1300_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    upstream_safety_interface = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_5_safety_virtual_force_node",
        name="proposal_simulation_cell_v1_5_safety_virtual_force_node",
        output="screen",
        parameters=[
            {"config_path": upstream_v1_5_config_path},
            {"output_dir": "diagnostics/proposal_simulation_cell_v1_9/upstream_v1_5"},
            {"isaac_available": isaac_available},
            {"gazebo_fallback_used": gazebo_fallback_used},
            {"ros_gz_bridge_available": ros_gz_bridge_available},
            {"world_path": world_path},
        ],
    )
    upstream_readiness_gate = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_6_readiness_gate_node",
        name="proposal_simulation_cell_v1_6_readiness_gate_node",
        output="screen",
        parameters=[
            {"config_path": upstream_v1_6_config_path},
            {"output_dir": "diagnostics/proposal_simulation_cell_v1_9/upstream_v1_6"},
            {"isaac_available": isaac_available},
            {"gazebo_fallback_used": gazebo_fallback_used},
            {"ros_gz_bridge_available": ros_gz_bridge_available},
            {"ros_gz_image_available": ros_gz_image_available},
        ],
    )
    upstream_pre_control_contract = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_7_pre_control_contract_node",
        name="proposal_simulation_cell_v1_7_pre_control_contract_node",
        output="screen",
        parameters=[
            {"config_path": upstream_v1_7_config_path},
            {"output_dir": "diagnostics/proposal_simulation_cell_v1_9/upstream_v1_7"},
            {"isaac_available": isaac_available},
            {"gazebo_fallback_used": gazebo_fallback_used},
            {"ros_gz_bridge_available": ros_gz_bridge_available},
            {"ros_gz_image_available": ros_gz_image_available},
        ],
    )
    upstream_control_scaffold = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_8_control_scaffold_node",
        name="proposal_simulation_cell_v1_8_control_scaffold_node",
        output="screen",
        parameters=[
            {"config_path": upstream_v1_8_config_path},
            {"output_dir": "diagnostics/proposal_simulation_cell_v1_9/upstream_v1_8"},
            {"isaac_available": isaac_available},
            {"gazebo_fallback_used": gazebo_fallback_used},
            {"ros_gz_bridge_available": ros_gz_bridge_available},
            {"ros_gz_image_available": ros_gz_image_available},
        ],
    )
    dry_run_control_law = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v1_9_control_law_dry_run_node",
        name="proposal_simulation_cell_v1_9_control_law_dry_run_node",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"output_dir": LaunchConfiguration("output_dir")},
            {"isaac_available": isaac_available},
            {"gazebo_fallback_used": gazebo_fallback_used},
            {"ros_gz_bridge_available": ros_gz_bridge_available},
            {"ros_gz_image_available": ros_gz_image_available},
        ],
    )

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        LogInfo(
            msg=(
                "proposal_simulation_cell_v1_9_no_motion_control_law_dry_run: "
                "diagnostic control-law outputs only. All computed commands are blocked; "
                "command outputs and motion execution are disabled; no MoveIt, /compute_ik, "
                "controllers, real robot, FollowJointTrajectory, or trajectory execution is used."
            )
        ),
        gazebo,
        clock_bridge,
        image_bridge,
        robot_state_publisher,
        upstream_safety_interface,
        upstream_readiness_gate,
        upstream_pre_control_contract,
        upstream_control_scaffold,
        dry_run_control_law,
        RegisterEventHandler(
            OnProcessExit(
                target_action=dry_run_control_law,
                on_exit=[EmitEvent(event=Shutdown(reason="v1.9 no-motion control-law dry run captured"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/proposal_simulation_cell_v1_9",
                description="Directory for proposal simulation cell v1.9 diagnostics.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
