"""Live v2_14 inference trial launch file.

Runs the canonical research baseline Gazebo world (D405 + F/T live)
and the synthetic_phase_publisher (170s multi-phase labels) PLUS
the live_v2_14_inference_node which subscribes to the same topics
and publishes predicted phase + target joint pose at 20 Hz.

The trial produces two CSVs:
  - multimodal_observation_log.csv  (D405 + state + wrench + phase)
  - live_v2_14_inference_log.csv    (ground truth + v2_14 predictions)

The two CSVs are then compared offline by
live_v2_14_ablation_analyzer.py to measure confusion matrix
(predicted vs ground-truth phase) and per-phase target-joint MSE.

Total trial duration is ~175s (170s schedule + ~5s startup).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    perception_log_dir = LaunchConfiguration("perception_log_dir")
    live_inference_dir = LaunchConfiguration("live_inference_dir")
    schedule_path_default = PathJoinSubstitution([
        FindPackageShare("thesis_bringup"),
        "config",
        "synthetic_phase_schedule_v1.yaml",
    ])

    baseline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("thesis_bringup"),
            "/launch/research_baseline.launch.py",
        ]),
        launch_arguments={
            "use_gui": LaunchConfiguration("use_gui"),
            "enable_perception_logging": "true",
            "perception_log_dir": perception_log_dir,
            "enable_synthetic_phases": "true",
            "synthetic_phase_schedule_path": LaunchConfiguration("schedule_path"),
            "enable_v2_14_live_inference": "true",
            "v2_14_live_inference_dir": live_inference_dir,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="false"),
        DeclareLaunchArgument(
            "perception_log_dir",
            default_value="diagnostics/perception_pipeline_live_v2_14_v1/multimodal",
        ),
        DeclareLaunchArgument(
            "live_inference_dir",
            default_value="diagnostics/perception_pipeline_live_v2_14_v1/inference",
        ),
        DeclareLaunchArgument(
            "schedule_path",
            default_value=schedule_path_default,
        ),
        baseline_launch,
    ])
