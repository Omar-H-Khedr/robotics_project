from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
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
                    }
                ],
            )
        ]
    )
