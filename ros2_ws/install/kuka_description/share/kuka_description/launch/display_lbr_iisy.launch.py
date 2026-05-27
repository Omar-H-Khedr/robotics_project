from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


SUPPORTED_MODELS = (
    'lbr_iisy3_r760',
    'lbr_iisy11_r1300',
    'lbr_iisy15_r930',
)


def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model').perform(context)
    if model not in SUPPORTED_MODELS:
        supported = ', '.join(SUPPORTED_MODELS)
        raise RuntimeError(f'Unsupported KUKA LBR iisy model "{model}". Use one of: {supported}')

    iisy_share = get_package_share_directory('kuka_lbr_iisy_support')
    package_share = get_package_share_directory('kuka_description')
    xacro_file = f'{iisy_share}/urdf/{model}.urdf.xacro'
    rviz_config = f'{package_share}/rviz/lbr_iisy_display.rviz'

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
        ' ',
        'mode:=mock',
    ])
    robot_description = {'robot_description': robot_description_content}

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[robot_description],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='lbr_iisy3_r760',
            description='KUKA LBR iisy model to visualize.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
