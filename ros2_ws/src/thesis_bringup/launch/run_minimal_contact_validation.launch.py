from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


MINIMAL_CONTACT_VALIDATION_WORLD = "minimal_contact_validation_world.sdf"
VALIDATION_CONTACT_TOPIC = (
    "/world/minimal_contact_validation_world/model/contact_validation_pad/link/"
    "pad_link/sensor/contact_validation_sensor/contact"
)
VALIDATION_FALLBACK_CONTACT_TOPIC = "/gazebo/contacts/validation"


def _workspace_results_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == "ros2_ws":
            return parent / "results" / "baseline_trials"
    return Path.cwd() / "results" / "baseline_trials"


def _world_path() -> Path:
    return (
        Path(get_package_share_directory("peg_in_hole_description"))
        / "worlds"
        / MINIMAL_CONTACT_VALIDATION_WORLD
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
    parameters["contact_topics"] = [
        f"validation_sensor:{VALIDATION_CONTACT_TOPIC}",
        f"validation:{VALIDATION_FALLBACK_CONTACT_TOPIC}",
    ]
    parameters["physical_contact_sources"] = ["validation_sensor", "validation"]
    return parameters


def generate_launch_description():
    results_root = _workspace_results_root()
    world_path = _world_path()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": [str(world_path), " -r -v1"]}.items(),
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="minimal_contact_validation_ros_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            (
                f"{VALIDATION_CONTACT_TOPIC}"
                "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
            ),
        ],
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "This is a minimal contact sensor diagnostic world, not the "
                    "research workcell."
                )
            ),
            gazebo,
            ros_gz_bridge,
            Node(
                package="peg_in_hole_metrics",
                executable="contact_metrics_node",
                name="contact_metrics_node",
                output="screen",
                parameters=[_contact_metrics_parameters()],
            ),
            Node(
                package="experiment_manager",
                executable="baseline_trial_manager",
                name="baseline_trial_manager",
                output="screen",
                parameters=[
                    {
                        "results_root": str(results_root),
                        "trial_mode": "contact_probe_validation",
                    }
                ],
            ),
        ]
    )
