"""Offline v2_15 ablation training launch file.

Runs v2_15_context_action_ablation as a one-shot subprocess.
The ablation trains two heads on the same multi-phase dataset:
  A. with v2_13 frozen encoder (input_dim=32)
  B. baseline (no encoder, input_dim=74)
and reports test accuracy and confusion matrices.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_parquet",
            default_value="diagnostics/perception_pipeline_synthetic_multiphase_v1/context_log.parquet",
        ),
        DeclareLaunchArgument(
            "encoder_pt",
            default_value="diagnostics/perception_pipeline_v2_13_encoder_v2/encoder.pt",
        ),
        DeclareLaunchArgument(
            "scaler_json",
            default_value="diagnostics/perception_pipeline_v2_13_encoder_v2/scaler.json",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="diagnostics/perception_pipeline_v2_15_ablation",
        ),
        DeclareLaunchArgument("epochs", default_value="200"),
        DeclareLaunchArgument("batch_size", default_value="64"),
        DeclareLaunchArgument("lr", default_value="0.001"),
        DeclareLaunchArgument("seed", default_value="0"),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "perception_pipeline", "v2_15_context_action_ablation",
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
