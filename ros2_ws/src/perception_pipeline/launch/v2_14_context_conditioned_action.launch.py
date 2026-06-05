"""Offline v2_14 context-conditioned action training launch file.

Runs v2_14_context_conditioned_action as a one-shot subprocess.
This is offline training; the launch file exists for symmetry
with the other perception_pipeline launch files.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_parquet",
            default_value="diagnostics/perception_pipeline_synthetic_multiphase_v1/context_log.parquet",
            description="Path to context_log.parquet (input).",
        ),
        DeclareLaunchArgument(
            "encoder_pt",
            default_value="diagnostics/perception_pipeline_v2_13_encoder_v2/encoder.pt",
            description="Path to v2_13 encoder.pt (encoder-only checkpoint).",
        ),
        DeclareLaunchArgument(
            "scaler_json",
            default_value="diagnostics/perception_pipeline_v2_13_encoder_v2/scaler.json",
            description="Path to v2_13 scaler.json (per-feature min/range).",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="diagnostics/perception_pipeline_v2_14_action",
            description="Directory to write action_classifier.pt, action_metadata.json, "
                        "confusion_matrix.png, per_phase_target_pose.json, "
                        "data_validation_report.json.",
        ),
        DeclareLaunchArgument("epochs", default_value="200"),
        DeclareLaunchArgument("batch_size", default_value="64"),
        DeclareLaunchArgument("lr", default_value="0.001"),
        DeclareLaunchArgument("seed", default_value="0"),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "perception_pipeline", "v2_14_context_conditioned_action",
                "--",
                "--input-parquet", LaunchConfiguration("input_parquet"),
                "--encoder-pt", LaunchConfiguration("encoder_pt"),
                "--scaler-json", LaunchConfiguration("scaler_json"),
                "--output-dir", LaunchConfiguration("output_dir"),
                "--epochs", LaunchConfiguration("epochs"),
                "--batch-size", LaunchConfiguration("batch_size"),
                "--lr", LaunchConfiguration("lr"),
                "--seed", LaunchConfiguration("seed"),
            ],
            output="screen",
        ),
    ])
