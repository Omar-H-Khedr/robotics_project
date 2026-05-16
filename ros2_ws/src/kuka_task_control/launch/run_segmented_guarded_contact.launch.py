from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="kuka_task_control",
                executable="segmented_guarded_contact_executor",
                name="segmented_guarded_contact_executor",
                output="screen",
                parameters=[
                    {
                        "action_server": (
                            "/joint_trajectory_controller/follow_joint_trajectory"
                        ),
                        "force_guard_status_topic": "/force_guard_status",
                        "insertion_metrics_topic": "/insertion_metrics",
                        "early_contact_force_threshold_n": 20.0,
                        "force_violation_threshold_n": 100.0,
                        "post_segment_guard_wait_sec": 0.3,
                    }
                ],
            )
        ]
    )
