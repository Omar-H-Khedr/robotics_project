from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

RESEARCH_ROBOT_XACRO = "lbr_iisy3_r760_research_gripper.urdf.xacro"


def _moveit_config_directory() -> Path:
    return (
        Path(get_package_share_directory("kuka_task_control"))
        / "config"
        / "moveit_lbr_iisy6_r1300"
    )


def _load_yaml(path: Path) -> dict:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}
    return data if isinstance(data, dict) else {}


def _launch_move_group_setup(context, *args, **kwargs):
    if LaunchConfiguration("launch_move_group").perform(context).lower() not in (
        "1",
        "true",
        "yes",
        "on",
    ):
        return []

    config_dir = _moveit_config_directory()
    srdf_path = config_dir / "lbr_iisy6_r1300.srdf"
    kinematics_path = config_dir / "kinematics.yaml"
    ompl_path = config_dir / "ompl_planning.yaml"
    if not (srdf_path.is_file() and kinematics_path.is_file() and ompl_path.is_file()):
        return [
            LogInfo(
                msg=(
                    "move_group diagnostic launch requested, but required "
                    "project-local SRDF, kinematics.yaml, or ompl_planning.yaml "
                    "is missing. Failing safely without launching move_group."
                )
            )
        ]

    robot_description_content = _robot_description_content()
    semantic_text = srdf_path.read_text(encoding="utf-8")
    kinematics = _load_yaml(kinematics_path)
    ompl = _load_yaml(ompl_path)

    return [
        LogInfo(
            msg=(
                "Launching diagnostic-only move_group with "
                "allow_trajectory_execution=false. No trajectory executor, "
                "controller client, FollowJointTrajectory goal, or IK request "
                "is started by this launch."
            )
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                {"robot_description": robot_description_content},
                {"robot_description_semantic": semantic_text},
                {"robot_description_kinematics": kinematics},
                ompl,
                {
                    "allow_trajectory_execution": False,
                    "trajectory_execution_allowed": False,
                    "controller_motion_allowed": False,
                    "diagnostic_only": True,
                    "planning_group": "arm",
                    "planning_frame": "base_link",
                    "tool_link": "tool0",
                    "publish_robot_description": True,
                    "publish_robot_description_semantic": True,
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                },
            ],
        ),
    ]


def _robot_description_content() -> Command:
    robot_description_xacro = PathJoinSubstitution(
        [
            FindPackageShare("peg_in_hole_description"),
            "urdf",
            RESEARCH_ROBOT_XACRO,
        ]
    )
    return Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_xacro,
            " ",
            "mode:=gazebo",
            " ",
            "prefix:=",
            " ",
            "x:=0.80",
            " ",
            "y:=-0.75",
            " ",
            "z:=0.75",
            " ",
            "roll:=0",
            " ",
            "pitch:=0",
            " ",
            "yaw:=1.5708",
        ],
        on_stderr="capture",
    )


def _robot_state_publisher() -> Node:
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": _robot_description_content()},
            {"use_sim_time": False, "mode": "gazebo", "diagnostic_only": True},
        ],
    )


def _diagnostic_nodes():
    return [
        Node(
            package="kuka_task_control",
            executable="move_group_diagnostic_config_builder",
            name="move_group_diagnostic_config_builder",
            output="screen",
        ),
        Node(
            package="kuka_task_control",
            executable="moveit_diagnostic_input_builder",
            name="moveit_diagnostic_input_builder",
            output="screen",
        ),
        Node(
            package="kuka_task_control",
            executable="robot_description_semantic_diagnostics",
            name="robot_description_semantic_diagnostics",
            output="screen",
        ),
        Node(
            package="kuka_task_control",
            executable="semantic_model_validator",
            name="semantic_model_validator",
            output="screen",
        ),
        Node(
            package="kuka_task_control",
            executable="tool_link_validator",
            name="tool_link_validator",
            output="screen",
        ),
        Node(
            package="kuka_task_control",
            executable="moveit_launch_readiness_audit",
            name="moveit_launch_readiness_audit",
            output="screen",
        ),
        Node(
            package="kuka_task_control",
            executable="moveit_config_audit",
            name="moveit_config_audit",
            output="screen",
        ),
        Node(
            package="kuka_task_control",
            executable="ik_backend_audit",
            name="ik_backend_audit",
            output="screen",
        ),
        Node(
            package="kuka_task_control",
            executable="move_group_runtime_audit",
            name="move_group_runtime_audit",
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_move_group",
                default_value="false",
                description=(
                    "When true, start diagnostic-only move_group with "
                    "allow_trajectory_execution=false. Defaults to false."
                ),
            ),
            LogInfo(
                msg=(
                    "Starting v2.14 MoveIt/move_group IK diagnostics. "
                    "Diagnostics always run; robot_state_publisher provides "
                    "robot_description; move_group is blocked by default."
                )
            ),
            _robot_state_publisher(),
            LogInfo(
                msg=(
                    "launch_move_group=false: not launching move_group. "
                    "This is the default safe diagnostic mode."
                ),
                condition=UnlessCondition(LaunchConfiguration("launch_move_group")),
            ),
            LogInfo(
                msg=(
                    "launch_move_group=true: attempting diagnostic-only move_group "
                    "with trajectory execution disabled."
                ),
                condition=IfCondition(LaunchConfiguration("launch_move_group")),
            ),
            *_diagnostic_nodes(),
            OpaqueFunction(function=_launch_move_group_setup),
        ]
    )
