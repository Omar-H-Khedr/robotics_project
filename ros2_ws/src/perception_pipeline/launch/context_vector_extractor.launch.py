"""Offline context vector extraction launch file.

Runs context_vector_extractor as a one-shot subprocess. The extractor
itself does not need ROS; this launch file exists for symmetry with the
other perception_pipeline launch files and to keep all v2_12 entry
points discoverable via `ros2 launch perception_pipeline ...`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_csv",
            default_value="diagnostics/perception_pipeline_d405_smoke/multimodal_observation_log.csv",
            description=(
                "Path to multimodal_observation_log.csv (input)."
            ),
        ),
        DeclareLaunchArgument(
            "output_parquet",
            default_value="diagnostics/perception_pipeline_context_extract/context_log.parquet",
            description=(
                "Path to write context_log.parquet (output)."
            ),
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "perception_pipeline", "context_vector_extractor",
                "--",
                "--input-csv", LaunchConfiguration("input_csv"),
                "--output-parquet", LaunchConfiguration("output_parquet"),
            ],
            output="screen",
        ),
    ])
