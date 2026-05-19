from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting IK backend audit only. This launch inspects ROS "
                    "services, package resources, robot model availability, and "
                    "project diagnostics, but executes no motion."
                )
            ),
            Node(
                package="kuka_task_control",
                executable="ik_backend_audit",
                name="ik_backend_audit",
                output="screen",
            ),
        ]
    )
