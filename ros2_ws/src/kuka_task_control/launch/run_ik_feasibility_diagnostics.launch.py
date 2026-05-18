from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_config = PathJoinSubstitution(
        [
            FindPackageShare("kuka_task_control"),
            "config",
            "peg_hole_cartesian_targets.yaml",
        ]
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting IK feasibility diagnostics only. This launch publishes "
                    "object frames and reachability diagnostics, but executes no motion."
                )
            ),
            Node(
                package="kuka_task_control",
                executable="peg_hole_frame_publisher",
                name="peg_hole_frame_publisher",
                output="screen",
                parameters=[{"config_path": target_config}],
            ),
            Node(
                package="kuka_task_control",
                executable="cartesian_insertion_diagnostics",
                name="cartesian_insertion_diagnostics",
                output="screen",
                parameters=[{"config_path": target_config}],
            ),
            Node(
                package="kuka_task_control",
                executable="ik_feasibility_diagnostics",
                name="ik_feasibility_diagnostics",
                output="screen",
                parameters=[{"config_path": target_config}],
            ),
        ]
    )
