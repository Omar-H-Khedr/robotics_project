from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    # Diagnostic-only preparation for MoveIt IK availability.
    #
    # This launch intentionally does not start task_trajectory_executor, does not
    # send FollowJointTrajectory goals, and does not launch move_group. Add a
    # move_group node only after moveit_config_audit confirms the correct KUKA
    # LBR iisy MoveIt config package, SRDF, kinematics.yaml, and launch resources
    # are present and the launch remains configured for no trajectory execution.
    #
    # robot_state_publisher is not started here because the main research
    # bringup already owns robot_description for the workcell. If this file is
    # used standalone, start a diagnostic robot_state_publisher separately with
    # the same validated KUKA LBR iisy robot_description.
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "Starting MoveIt IK diagnostic preparation only. This launch "
                    "does not start move_group or any trajectory executor."
                )
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
