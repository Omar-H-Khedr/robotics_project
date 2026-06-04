"""Offline v2_13 context encoder training launch file.

Runs v2_13_context_encoder as a one-shot subprocess. The encoder is
an offline training step; this launch file exists for symmetry with
the other perception_pipeline launch files and to make the v2_13
entry point discoverable via `ros2 launch perception_pipeline ...`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_parquet",
            default_value="diagnostics/perception_pipeline_motion_trial_v3/context_log.parquet",
            description="Path to context_log.parquet (input).",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="diagnostics/perception_pipeline_v2_13_encoder",
            description="Directory to write encoder.pt, autoencoder.pt, scaler.json, "
                        "metadata.json, data_validation_report.json, training_curve.png.",
        ),
        DeclareLaunchArgument("epochs", default_value="200"),
        DeclareLaunchArgument("batch_size", default_value="64"),
        DeclareLaunchArgument("lr", default_value="0.001"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("latent_dim", default_value="32"),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "perception_pipeline", "v2_13_context_encoder",
                "--",
                "--input-parquet", LaunchConfiguration("input_parquet"),
                "--output-dir", LaunchConfiguration("output_dir"),
                "--epochs", LaunchConfiguration("epochs"),
                "--batch-size", LaunchConfiguration("batch_size"),
                "--lr", LaunchConfiguration("lr"),
                "--seed", LaunchConfiguration("seed"),
                "--latent-dim", LaunchConfiguration("latent_dim"),
            ],
            output="screen",
        ),
    ])
