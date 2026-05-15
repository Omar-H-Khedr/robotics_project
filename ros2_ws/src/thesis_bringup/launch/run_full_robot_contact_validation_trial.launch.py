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


ROBOT_CONTACT_VALIDATION_WORLD = "peg_in_hole_robot_contact_validation_world.sdf"
ROBOT_CONTACT_VALIDATION_TOPIC = (
    "/world/peg_in_hole_robot_contact_validation_world/model/"
    "robot_contact_validation_pad/link/robot_contact_validation_pad_link/sensor/"
    "robot_contact_validation_sensor/contact"
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
    parameters["physical_contact_sources"] = ["robot_validation"]
    return parameters


def generate_launch_description():
    results_root = _workspace_results_root()

    robot_contact_validation_sequence = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_task_control"),
                    "launch",
                    "run_robot_contact_validation_sequence.launch.py",
                ]
            )
        )
    )

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

    def start_robot_contact_sequence_when_ready(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Controller readiness confirmed; starting robot contact "
                        "validation sequence automatically."
                    )
                ),
                robot_contact_validation_sequence,
            ]

        return [
            LogInfo(
                msg=(
                    "Controller readiness gate failed; robot contact validation "
                    "sequence will not be started automatically."
                )
            )
        ]

    robot_validation_contact_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="robot_contact_validation_ros_gz_bridge",
        output="screen",
        arguments=[
            f"{ROBOT_CONTACT_VALIDATION_TOPIC}"
            "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ],
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting Research Baseline v0.6 robot-to-object contact "
                    "validation trial."
                )
            ),
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
                    "world_file": ROBOT_CONTACT_VALIDATION_WORLD,
                }.items(),
            ),
            robot_validation_contact_bridge,
            TimerAction(
                period=5.0,
                actions=[
                    LogInfo(msg="Starting safety monitor for robot contact validation."),
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
                    LogInfo(msg="Starting robot contact validation trial manager."),
                    Node(
                        package="experiment_manager",
                        executable="baseline_trial_manager",
                        name="baseline_trial_manager",
                        output="screen",
                        parameters=[
                            {
                                "results_root": str(results_root),
                                "trial_mode": "robot_contact_validation",
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    LogInfo(msg="Starting contact metrics for robot validation."),
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
                    on_exit=start_robot_contact_sequence_when_ready,
                )
            ),
        ]
    )
