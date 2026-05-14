from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    return parameters


def generate_launch_description():
    results_root = _workspace_results_root()

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting Research Baseline v0.3: Gazebo baseline, "
                    "contact metrics, safety monitor, and trial logger. Launch "
                    "the task sequence separately from Terminal 2 with: ros2 launch "
                    "kuka_task_control run_task_sequence.launch.py"
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
                )
            ),
            TimerAction(
                period=5.0,
                actions=[
                    LogInfo(
                        msg=(
                            "Research Baseline v0.3: starting safety monitor "
                            "after 5 seconds so Gazebo and KUKA spawning are not "
                            "blocked by auxiliary nodes."
                        )
                    ),
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
                    LogInfo(
                        msg=(
                            "Research Baseline v0.3: starting "
                            "experiment_manager/baseline_trial_manager for "
                            "non-blocking baseline trial logging. Terminal 2 command: "
                            "ros2 launch kuka_task_control run_task_sequence.launch.py"
                        )
                    ),
                    Node(
                        package="experiment_manager",
                        executable="baseline_trial_manager",
                        name="baseline_trial_manager",
                        output="screen",
                        parameters=[{"results_root": str(results_root)}],
                    ),
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    LogInfo(
                        msg=(
                            "Research Baseline v0.3: starting contact metrics "
                            "after 7 seconds."
                        )
                    ),
                    Node(
                        package="peg_in_hole_metrics",
                        executable="contact_metrics_node",
                        name="contact_metrics_node",
                        output="screen",
                        parameters=[_contact_metrics_parameters()],
                    ),
                ],
            ),
        ]
    )
