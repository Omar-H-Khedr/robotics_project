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
                    "Starting Research Baseline v2.4 coordinate-based insertion "
                    "diagnostics. This launch performs no task trajectory execution."
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
        ]
    )
