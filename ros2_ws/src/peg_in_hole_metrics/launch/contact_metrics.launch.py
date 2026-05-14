from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


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
    return LaunchDescription(
        [
            Node(
                package="peg_in_hole_metrics",
                executable="contact_metrics_node",
                name="contact_metrics_node",
                output="screen",
                parameters=[_contact_metrics_parameters()],
            )
        ]
    )
