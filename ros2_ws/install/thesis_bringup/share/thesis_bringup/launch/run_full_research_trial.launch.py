from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    task_sequence_node = Node(
        package="kuka_task_control",
        executable="task_trajectory_executor",
        name="task_trajectory_executor",
        output="screen",
        parameters=[
            {
                "config_path": PathJoinSubstitution(
                    [
                        FindPackageShare("kuka_task_control"),
                        "config",
                        "baseline_task_sequence.yaml",
                    ]
                )
            }
        ],
    )

    readiness_gate_node = Node(
        package="thesis_bringup",
        executable="controller_readiness_gate",
        name="controller_readiness_gate",
        output="screen",
        parameters=[
            {
                "action_server": "/joint_trajectory_controller/follow_joint_trajectory",
                "timeout_sec": 60.0,
            }
        ],
    )

    def start_task_sequence_when_ready(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Controller readiness confirmed; starting task sequence "
                        "automatically."
                    )
                ),
                task_sequence_node,
            ]

        return [
            LogInfo(
                msg=(
                    "Controller readiness gate failed; task sequence will not be "
                    "started automatically."
                )
            )
        ]

    return LaunchDescription(
        [
            LogInfo(msg="Starting full research trial."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("thesis_bringup"),
                            "launch",
                            "run_research_trial.launch.py",
                        ]
                    )
                )
            ),
            TimerAction(
                period=5.0,
                actions=[
                    LogInfo(msg="Waiting for controller readiness."),
                    readiness_gate_node,
                ],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=readiness_gate_node,
                    on_exit=start_task_sequence_when_ready,
                )
            ),
        ]
    )
