import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory('robot_simulation')
    description_share = get_package_share_directory('robot_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(simulation_share, 'worlds', 'empty_world.sdf')
    urdf_file = os.path.join(description_share, 'urdf', 'simple_diff_drive.urdf')

    with open(urdf_file, 'r', encoding='utf-8') as file:
        robot_description = file.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_simple_diff_drive',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'simple_diff_drive',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.10',
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
