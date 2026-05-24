from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


INSERTION_VALIDATION_WORLD = "peg_in_hole_insertion_validation_world.sdf"
PEG_VALIDATION_TOPIC = (
    "/world/peg_in_hole_insertion_validation_world/model/peg/link/peg_link/"
    "sensor/peg_contact_sensor/contact"
)
HOLE_VALIDATION_TOPIC = (
    "/world/peg_in_hole_insertion_validation_world/model/hole_block/link/"
    "hole_block_link/sensor/hole_contact_sensor/contact"
)
COMPAT_CONTACT_TOPICS = (
    "/gazebo/contacts/peg",
    "/gazebo/contacts/hole",
    "/gazebo/contacts/target",
)


def _contact_metrics_parameters() -> dict[str, object]:
    config_path = (
        Path(get_package_share_directory("peg_in_hole_metrics"))
        / "config"
        / "contact_metrics.yaml"
    )
    with config_path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}

    parameters = config.get("contact_metrics_node", {}).get("ros__parameters", {})
    contact_topics = parameters.get("contact_topics", [])
    normalized_topics = []
    for entry in contact_topics:
        if isinstance(entry, dict):
            name = str(entry.get("name", "")).strip()
            topic = str(entry.get("topic", "")).strip()
            if name and topic:
                normalized_topics.append(f"{name}:{topic}")
        elif entry:
            normalized_topics.append(str(entry))
    parameters["contact_topics"] = normalized_topics
    parameters["physical_contact_sources"] = [
        "peg_validation",
        "hole_validation",
        "peg",
        "hole",
        "target",
    ]
    return parameters


def _contact_bridge(topic: str, name: str) -> Node:
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=name,
        output="screen",
        arguments=[f"{topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"],
    )


def generate_launch_description():
    contact_bridges = [
        _contact_bridge(PEG_VALIDATION_TOPIC, "cartesian_peg_validation_contact_bridge"),
        _contact_bridge(HOLE_VALIDATION_TOPIC, "cartesian_hole_validation_contact_bridge"),
        *[
            _contact_bridge(topic, f"cartesian_compat_contact_bridge_{index}")
            for index, topic in enumerate(COMPAT_CONTACT_TOPICS, start=1)
        ],
    ]

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting Research Baseline v2.7 coordinate-based insertion "
                    "diagnostics, orientation target calculation, execution gates, "
                    "tool-axis audit, Cartesian dry-run planning, and IK backend "
                    "plus MoveIt config and launch readiness audits. This launch "
                    "performs no task trajectory execution."
                )
            ),
            LogInfo(msg="Spawning exactly one KUKA robot entity: kuka_lbr_iisy"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("thesis_bringup"),
                            "launch",
                            "research_baseline.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "world_file": INSERTION_VALIDATION_WORLD,
                    "robot_model": "kuka_lbr_iisy",
                    "allow_robot_renaming": "false",
                    "x": "0.80",
                    "y": "-0.75",
                    "z": "0.75",
                    "yaw": "1.5708",
                }.items(),
            ),
            *contact_bridges,
            TimerAction(
                period=5.0,
                actions=[
                    LogInfo(msg="Starting safety monitor for Cartesian diagnostics."),
                    Node(
                        package="safety_layer",
                        executable="safety_monitor",
                        name="safety_monitor",
                        output="screen",
                        parameters=[
                            {
                                "config_path": PathJoinSubstitution(
                                    [
                                        FindPackageShare("safety_layer"),
                                        "config",
                                        "safety_limits.yaml",
                                    ]
                                )
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    LogInfo(msg="Starting peg/hole contact metrics for diagnostics."),
                    Node(
                        package="peg_in_hole_metrics",
                        executable="contact_metrics_node",
                        name="contact_metrics_node",
                        output="screen",
                        parameters=[_contact_metrics_parameters()],
                    ),
                ],
            ),
            TimerAction(
                period=6.5,
                actions=[
                    LogInfo(msg="Starting peg/hole object frame publisher."),
                    Node(
                        package="kuka_task_control",
                        executable="peg_hole_frame_publisher",
                        name="peg_hole_frame_publisher",
                        output="screen",
                        parameters=[
                            {
                                "config_path": PathJoinSubstitution(
                                    [
                                        FindPackageShare("kuka_task_control"),
                                        "config",
                                        "peg_hole_cartesian_targets.yaml",
                                    ]
                                )
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    LogInfo(msg="Starting Cartesian insertion diagnostics node."),
                    Node(
                        package="kuka_task_control",
                        executable="cartesian_insertion_diagnostics",
                        name="cartesian_insertion_diagnostics",
                        output="screen",
                        parameters=[
                            {
                                "config_path": PathJoinSubstitution(
                                    [
                                        FindPackageShare("kuka_task_control"),
                                        "config",
                                        "peg_hole_cartesian_targets.yaml",
                                    ]
                                )
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=7.5,
                actions=[
                    LogInfo(msg="Starting IK feasibility diagnostics node."),
                    Node(
                        package="kuka_task_control",
                        executable="ik_feasibility_diagnostics",
                        name="ik_feasibility_diagnostics",
                        output="screen",
                        parameters=[
                            {
                                "config_path": PathJoinSubstitution(
                                    [
                                        FindPackageShare("kuka_task_control"),
                                        "config",
                                        "peg_hole_cartesian_targets.yaml",
                                    ]
                                )
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    LogInfo(msg="Starting tool-axis audit node."),
                    Node(
                        package="kuka_task_control",
                        executable="tool_axis_audit",
                        name="tool_axis_audit",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=8.5,
                actions=[
                    LogInfo(msg="Starting Cartesian orientation target calculator."),
                    Node(
                        package="kuka_task_control",
                        executable="cartesian_orientation_target_calculator",
                        name="cartesian_orientation_target_calculator",
                        output="screen",
                        parameters=[
                            {
                                "config_path": PathJoinSubstitution(
                                    [
                                        FindPackageShare("kuka_task_control"),
                                        "config",
                                        "peg_hole_cartesian_targets.yaml",
                                    ]
                                )
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=9.0,
                actions=[
                    LogInfo(msg="Starting Cartesian insertion dry-run planner."),
                    Node(
                        package="kuka_task_control",
                        executable="cartesian_insertion_dry_run_planner",
                        name="cartesian_insertion_dry_run_planner",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=9.5,
                actions=[
                    LogInfo(msg="Starting unified execution gate monitor."),
                    Node(
                        package="kuka_task_control",
                        executable="execution_gate_monitor",
                        name="execution_gate_monitor",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    LogInfo(msg="Starting IK backend audit node."),
                    Node(
                        package="kuka_task_control",
                        executable="ik_backend_audit",
                        name="ik_backend_audit",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=10.5,
                actions=[
                    LogInfo(msg="Starting MoveIt config audit node."),
                    Node(
                        package="kuka_task_control",
                        executable="moveit_config_audit",
                        name="moveit_config_audit",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=10.75,
                actions=[
                    LogInfo(msg="Starting tool-link validator node."),
                    Node(
                        package="kuka_task_control",
                        executable="tool_link_validator",
                        name="tool_link_validator",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=11.75,
                actions=[
                    LogInfo(msg="Starting MoveIt launch readiness audit node."),
                    Node(
                        package="kuka_task_control",
                        executable="moveit_launch_readiness_audit",
                        name="moveit_launch_readiness_audit",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=11.85,
                actions=[
                    LogInfo(
                        msg=(
                            "Starting move_group diagnostic config builder "
                            "without launching move_group."
                        )
                    ),
                    Node(
                        package="kuka_task_control",
                        executable="move_group_diagnostic_config_builder",
                        name="move_group_diagnostic_config_builder",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=11.9,
                actions=[
                    LogInfo(msg="Starting MoveIt diagnostic input builder node."),
                    Node(
                        package="kuka_task_control",
                        executable="moveit_diagnostic_input_builder",
                        name="moveit_diagnostic_input_builder",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=12.0,
                actions=[
                    LogInfo(
                        msg="Starting robot_description_semantic diagnostics node."
                    ),
                    Node(
                        package="kuka_task_control",
                        executable="robot_description_semantic_diagnostics",
                        name="robot_description_semantic_diagnostics",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=12.25,
                actions=[
                    LogInfo(
                        msg=(
                            "Starting move_group runtime audit without launching "
                            "move_group."
                        )
                    ),
                    Node(
                        package="kuka_task_control",
                        executable="move_group_runtime_audit",
                        name="move_group_runtime_audit",
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=12.5,
                actions=[
                    LogInfo(msg="Starting semantic model validator node."),
                    Node(
                        package="kuka_task_control",
                        executable="semantic_model_validator",
                        name="semantic_model_validator",
                        output="screen",
                    ),
                ],
            ),
        ]
    )
