from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [
            FindPackageShare("kuka_task_control"),
            "config",
            "peg_hole_cartesian_targets.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="kuka_task_control",
                executable="peg_hole_frame_publisher",
                name="peg_hole_frame_publisher",
                output="screen",
                parameters=[{"config_path": config_path}],
            ),
            Node(
                package="kuka_task_control",
                executable="cartesian_insertion_diagnostics",
                name="cartesian_insertion_diagnostics",
                output="screen",
                parameters=[{"config_path": config_path}],
            )
        ]
    )
