from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
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


def _workspace_results_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == "ros2_ws":
            return parent / "results" / "baseline_trials"
    return Path.cwd() / "results" / "baseline_trials"


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
    results_root = _workspace_results_root()

    readiness_gate_node = Node(
        package="thesis_bringup",
        executable="controller_readiness_gate",
        name="controller_readiness_gate",
        output="screen",
        parameters=[
            {
                "action_server": "/joint_trajectory_controller/follow_joint_trajectory",
                "timeout_sec": 60.0,
            }
        ],
    )

    insertion_sequence_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_task_control"),
                    "launch",
                    "run_peg_hole_insertion_validation_sequence.launch.py",
                ]
            )
        )
    )

    def start_insertion_sequence_when_ready(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Controller readiness confirmed; starting v2.0 peg/hole "
                        "insertion validation sequence."
                    )
                ),
                insertion_sequence_launch,
            ]
        return [
            LogInfo(
                msg=(
                    "Controller readiness gate failed; peg/hole insertion validation "
                    "sequence will not be started automatically."
                )
            )
        ]

    contact_bridges = [
        _contact_bridge(PEG_VALIDATION_TOPIC, "peg_validation_contact_bridge"),
        _contact_bridge(HOLE_VALIDATION_TOPIC, "hole_validation_contact_bridge"),
        *[
            _contact_bridge(topic, f"compat_contact_bridge_{index}")
            for index, topic in enumerate(COMPAT_CONTACT_TOPICS, start=1)
        ],
    ]

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting Research Baseline v2.0 peg/hole insertion validation "
                    "trial."
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
                    LogInfo(msg="Starting safety monitor for peg/hole validation."),
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
                    LogInfo(msg="Starting peg/hole insertion validation trial manager."),
                    Node(
                        package="experiment_manager",
                        executable="baseline_trial_manager",
                        name="baseline_trial_manager",
                        output="screen",
                        parameters=[
                            {
                                "results_root": str(results_root),
                                "trial_mode": "peg_hole_insertion_validation",
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    LogInfo(msg="Starting peg/hole insertion contact metrics."),
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
                period=8.0,
                actions=[
                    LogInfo(msg="Waiting for controller readiness."),
                    readiness_gate_node,
                ],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=readiness_gate_node,
                    on_exit=start_insertion_sequence_when_ready,
                )
            ),
        ]
    )
