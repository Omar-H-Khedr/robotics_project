from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    # v2.9 diagnostic-only preparation for a future MoveIt/move_group IK path.
    # This launch intentionally does not start task_trajectory_executor, does not
    # send FollowJointTrajectory goals, and does not launch move_group. A future
    # move_group action must remain blocked until moveit_launch_readiness_audit
    # confirms an exact LBR iisy 6 R1300 semantic model and safe launch inputs.
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting v2.9 MoveIt/move_group IK diagnostic launch "
                    "preparation only. This launch starts audits only and does "
                    "not start move_group or any trajectory executor."
                )
            ),
            Node(
                package="kuka_task_control",
                executable="moveit_launch_readiness_audit",
                name="moveit_launch_readiness_audit",
                output="screen",
            ),
            Node(
                package="kuka_task_control",
                executable="moveit_config_audit",
                name="moveit_config_audit",
                output="screen",
            ),
            Node(
                package="kuka_task_control",
                executable="ik_backend_audit",
                name="ik_backend_audit",
                output="screen",
            ),
        ]
    )
