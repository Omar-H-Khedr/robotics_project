from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "run_full_contact_validation_trial.launch.py is retained as "
                    "a compatibility alias. Starting the passive Research Baseline "
                    "v0.4 contact-probe validation trial; no task trajectory "
                    "executor is launched."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("thesis_bringup"),
                            "launch",
                            "run_contact_probe_validation_trial.launch.py",
                        ]
                    )
                )
            ),
        ]
    )
