from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the multi-modal synchronized observation logger."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "output_dir",
            default_value="diagnostics/multimodal_observation_log",
            description="Output directory for the observation CSV log.",
        ),
        DeclareLaunchArgument(
            "rate_hz",
            default_value="20.0",
            description="Logging rate in Hz.",
        ),
        DeclareLaunchArgument(
            "log_rgb",
            default_value="true",
            description="If true, encode and write RGB frames.",
        ),
        DeclareLaunchArgument(
            "log_depth",
            default_value="true",
            description="If true, write depth summary statistics.",
        ),
        Node(
            package="perception_pipeline",
            executable="multimodal_observation_logger",
            name="multimodal_observation_logger",
            output="both",
            parameters=[{
                "output_dir": LaunchConfiguration("output_dir"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "log_rgb": LaunchConfiguration("log_rgb"),
                "log_depth": LaunchConfiguration("log_depth"),
            }],
        ),
    ])
