import os

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
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    peg_share = get_package_share_directory("peg_in_hole_description")
    model_path = os.path.join(peg_share, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = model_path if not existing_resource_path else model_path + os.pathsep + existing_resource_path
    world_path = os.path.join(peg_share, "worlds", "proposal_simulation_cell_v1_3_contact_physics_validation.world.sdf")

    config_path = PathJoinSubstitution(
        [FindPackageShare("thesis_bringup"), "config", "proposal_simulation_cell_v2_1.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_lbr_iisy_support"),
                    "urdf",
                    "lbr_iisy3_r760.urdf.xacro",
                ]
            ),
            " mode:=gazebo",
            " x:=0.80",
            " y:=-0.75",
            " z:=0.75",
            " yaw:=1.5708",
        ],
        on_stderr="capture",
    )
    robot_description = {"robot_description": robot_description_content}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -s -r -v1"]}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="proposal_simulation_cell_v2_1_gz_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="proposal_simulation_cell_v2_1_robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True, "mode": "gazebo"}],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="proposal_simulation_cell_v2_1_spawn_robot",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "lbr_iisy3_r760_v2_1_validation_suite",
            "-allow_renaming",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="proposal_simulation_cell_v2_1_joint_state_broadcaster_spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="proposal_simulation_cell_v2_1_joint_trajectory_controller_spawner",
        output="screen",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )
    validation_suite = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v2_1_motion_validation_suite_node",
        name="proposal_simulation_cell_v2_1_motion_validation_suite_node",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"output_dir": LaunchConfiguration("output_dir")},
        ],
    )

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        LogInfo(
            msg=(
                "proposal_simulation_cell_v2_1_gazebo_motion_validation_suite: bounded "
                "Gazebo-only joint-space validation. No real robot, MoveIt, /compute_ik, "
                "learning, scenario batch execution, Cartesian motion, peg insertion, or "
                "contact-seeking motion."
            )
        ),
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_trajectory_controller,
                on_exit=[validation_suite],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=validation_suite,
                on_exit=[EmitEvent(event=Shutdown(reason="v2.1 Gazebo motion validation suite captured"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/proposal_simulation_cell_v2_1",
                description="Directory for proposal simulation cell v2.1 diagnostics.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
