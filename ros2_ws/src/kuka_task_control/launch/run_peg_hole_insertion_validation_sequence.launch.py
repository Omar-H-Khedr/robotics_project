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
                                "peg_hole_insertion_validation_sequence.yaml",
                            ]
                        ),
                        "force_guard_enabled": True,
                        "force_warning_threshold_n": 50.0,
                        "force_violation_threshold_n": 100.0,
                        "force_guard_topic": "/insertion_metrics",
                        "force_guard_retreat_phase": "insertion_retreat",
                        "early_contact_guard_enabled": True,
                        "stop_on_first_contact": False,
                        "early_contact_force_threshold_n": 20.0,
                        "early_contact_guard_topic": "/force_guard_status",
                        "guarded_contact_retreat_phase": "insertion_retreat",
                        "guarded_contact_success_status": "guarded_contact_stop",
                    }
                ],
            )
        ]
    )
