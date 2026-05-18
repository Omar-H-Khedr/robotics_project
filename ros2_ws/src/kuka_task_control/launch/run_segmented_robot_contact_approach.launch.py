from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kuka_task_control",
                executable="segmented_contact_executor",
                name="segmented_contact_executor",
                output="screen",
                parameters=[
                    {
                        "config_path": PathJoinSubstitution(
                            [
                                FindPackageShare("kuka_task_control"),
                                "config",
                                "segmented_robot_contact_approach.yaml",
                            ]
                        ),
                        "action_server": (
                            "/joint_trajectory_controller/follow_joint_trajectory"
                        ),
                        "force_guard_status_topic": "/force_guard_status",
                        "insertion_metrics_topic": "/insertion_metrics",
                    }
                ],
            )
        ]
    )
