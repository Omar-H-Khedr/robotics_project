import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("peg_in_hole_description")
    model_path = os.path.join(package_share, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = (
        model_path
        if not existing_resource_path
        else model_path + os.pathsep + existing_resource_path
    )

    world_path = os.path.join(package_share, "worlds", "peg_in_hole_world.sdf")
    use_gui = LaunchConfiguration("use_gui")

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -r -v1"]}.items(),
        condition=IfCondition(use_gui),
    )

    gz_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_server.launch.py"])
        ),
        launch_arguments={
            "world_sdf_file": world_path,
            "container_name": "peg_in_hole_gz_container",
            "create_own_container": "False",
            "use_composition": "False",
        }.items(),
        condition=UnlessCondition(use_gui),
    )

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        gz_sim_launch,
        gz_server_launch,
    ]


def generate_launch_description():
    """Launch only the peg-in-hole Gazebo task scene."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="If true, launch gz_sim with GUI. If false, launch gz_server only.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
