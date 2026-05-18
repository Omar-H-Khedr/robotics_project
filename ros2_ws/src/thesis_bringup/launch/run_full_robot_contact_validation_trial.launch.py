from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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

    task_trajectory_executor_node = Node(
        package="kuka_task_control",
        executable="task_trajectory_executor",
        name="task_trajectory_executor",
        output="screen",
        parameters=[
            {
                "task_sequence_file": PathJoinSubstitution(
                    [
                        FindPackageShare("kuka_task_control"),
                        "config",
                        "robot_contact_validation_sequence.yaml",
                    ]
                ),
                "force_guard_enabled": True,
                "force_warning_threshold_n": 50.0,
                "force_violation_threshold_n": 100.0,
                "force_guard_topic": "/insertion_metrics",
                "early_contact_guard_enabled": True,
                "stop_on_first_contact": True,
                "early_contact_force_threshold_n": 20.0,
                "early_contact_guard_topic": "/force_guard_status",
            }
        ],
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
                task_trajectory_executor_node,
            ]

        return [
            LogInfo(
                msg=(
                    "Controller readiness gate failed; robot contact validation "
                    "sequence will not be started automatically."
                )
            )
        ]

    def finish_trial_when_task_executor_exits(event, _context):
        if event.returncode == 0:
            exit_message = (
                "Robot contact validation task executor exited cleanly; "
                "completed or controlled terminal state reached."
            )
        else:
            exit_message = (
                "Robot contact validation task executor exited with return code "
                f"{event.returncode}; final logs will be flushed before shutdown."
            )

        return [
            LogInfo(msg=exit_message),
            TimerAction(
                period=2.0,
                actions=[
                    LogInfo(
                        msg=(
                            "Stopping robot contact validation launch after terminal "
                            "task executor exit."
                        )
                    ),
                    EmitEvent(
                        event=Shutdown(
                            reason="robot contact validation task executor exited"
                        )
                    ),
                ],
            ),
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
                    "world_file": ROBOT_CONTACT_VALIDATION_WORLD,
                    "robot_model": "kuka_lbr_iisy",
                    "allow_robot_renaming": "false",
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
            RegisterEventHandler(
                OnProcessExit(
                    target_action=task_trajectory_executor_node,
                    on_exit=finish_trial_when_task_executor_exits,
                )
            ),
        ]
    )
