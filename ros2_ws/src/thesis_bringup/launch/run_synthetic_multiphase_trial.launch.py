"""Multi-phase synthetic trial launch file.

Runs the canonical research baseline Gazebo world (so the D405 camera
and force/torque sensor topics are live) and replaces the
admittance_insertion_node with the synthetic_phase_publisher. Combined
with the multimodal_observation_logger (enable_perception_logging=true)
this produces a multi-phase labeled CSV for v2_14 / v2_15 offline
training.

The arm does NOT execute any controller commands in this trial; the
JTC is still spawned and activated (so /joint_states is real) but
no trajectory is sent. The /task_phase labels are time-window proxies,
not real motor-actuated phases; the dataset is explicitly a
synthetic-phase proxy for offline training.

Total trial duration is controlled by the schedule YAML
(default: 170 s).
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    perception_log_dir = LaunchConfiguration("perception_log_dir")
    schedule_path_default = PathJoinSubstitution([
        FindPackageShare("thesis_bringup"),
        "config",
        "synthetic_phase_schedule_v1.yaml",
    ])

    baseline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("thesis_bringup"),
                "/launch/research_baseline.launch.py",
            ]
        ),
        launch_arguments={
            "use_gui": LaunchConfiguration("use_gui"),
            "enable_perception_logging": "true",
            "perception_log_dir": perception_log_dir,
            "enable_synthetic_phases": "true",
            "synthetic_phase_schedule_path": LaunchConfiguration("schedule_path"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="false"),
        DeclareLaunchArgument(
            "perception_log_dir",
            default_value="diagnostics/perception_pipeline_synthetic_multiphase_v1",
        ),
        DeclareLaunchArgument(
            "schedule_path",
            default_value=schedule_path_default,
        ),
        baseline_launch,
    ])
