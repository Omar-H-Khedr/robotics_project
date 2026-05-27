import os

import yaml
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    peg_share = get_package_share_directory("peg_in_hole_description")
    moveit_share = get_package_share_directory("kuka_lbr_iisy_moveit_config")
    support_share = get_package_share_directory("kuka_lbr_iisy_support")
    model_path = os.path.join(peg_share, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = model_path if not existing_resource_path else model_path + os.pathsep + existing_resource_path
    world_path = os.path.join(peg_share, "worlds", "proposal_simulation_cell_v1_3_contact_physics_validation.world.sdf")
    with open(moveit_share + "/config/ompl_planning.yaml", "r", encoding="utf-8") as planning_file:
        ompl_planning = yaml.safe_load(planning_file)
    with open(support_share + "/config/lbr_iisy3_r760_joint_limits.yaml", "r", encoding="utf-8") as limits_file:
        joint_limits = yaml.safe_load(limits_file)

    config_path = PathJoinSubstitution(
        [FindPackageShare("thesis_bringup"), "config", "proposal_simulation_cell_v2_15.yaml"]
    )
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("kuka_lbr_iisy_support"), "urdf", "lbr_iisy3_r760.urdf.xacro"]
                ),
                " mode:=gazebo",
                " x:=0.80",
                " y:=-0.75",
                " z:=0.75",
                " yaw:=1.5708",
            ],
            on_stderr="capture",
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    "cat ",
                    PathJoinSubstitution(
                        [FindPackageShare("kuka_lbr_iisy_moveit_config"), "urdf", "lbr_iisy3_r760.srdf"]
                    ),
                ]
            ),
            value_type=str,
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": {
            "manipulator": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
                "kinematics_solver_attempts": 1,
            }
        }
    }
    planning_pipeline = {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl", "ompl": ompl_planning}
    execution_gazebo_only = {
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
        "trajectory_execution_allowed": "gazebo_simulation_only",
        "planning_execution_allowed": False,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "use_sim_time": True,
    }
    robot_description_planning = {"robot_description_planning": joint_limits}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -s -r -v1"]}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="proposal_simulation_cell_v2_15_gz_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    contact_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="proposal_simulation_cell_v2_15_contact_bridge",
        output="screen",
        arguments=[
            "/world/proposal_simulation_cell_v1_3_contact_physics_validation/model/"
            "proposal_v2_15_context_action_ablation_contact_pad/link/contact_calibration_pad_link/"
            "sensor/contact_calibration_sensor/contact"
            "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="proposal_simulation_cell_v2_15_robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True, "mode": "gazebo"}],
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="proposal_simulation_cell_v2_15_spawn_robot",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "lbr_iisy3_r760_v2_15_context_action_ablation",
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
        name="proposal_simulation_cell_v2_15_joint_state_broadcaster_spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "30", "--switch-timeout", "20"],
    )
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="proposal_simulation_cell_v2_15_joint_trajectory_controller_spawner",
        output="screen",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--controller-manager-timeout", "30", "--switch-timeout", "20"],
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            robot_description_planning,
            execution_gazebo_only,
        ],
    )
    task_sequence = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v2_15_context_action_ablation_node",
        name="proposal_simulation_cell_v2_15_context_action_ablation_node",
        output="screen",
        parameters=[{"config_path": config_path}, {"output_dir": LaunchConfiguration("output_dir")}],
    )

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        LogInfo(
            msg=(
                "proposal_simulation_cell_v2_15_context_action_ablation_validation: "
                "paired Gazebo-only diagnostic ablation comparing fixed baseline guarded action "
                "parameters with deterministic context-conditioned guarded action parameters. No "
                "policy training, RL training, fake result, real robot, physical endpoint, peg "
                "insertion, or forceful contact."
            )
        ),
        gazebo,
        bridge,
        contact_bridge,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[joint_trajectory_controller])),
        RegisterEventHandler(OnProcessExit(target_action=joint_trajectory_controller, on_exit=[move_group, task_sequence])),
        RegisterEventHandler(
            OnProcessExit(
                target_action=task_sequence,
                on_exit=[EmitEvent(event=Shutdown(reason="v2.15 context action ablation validation captured"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/proposal_simulation_cell_v2_15",
                description="Directory for proposal simulation cell v2.15 diagnostics.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
