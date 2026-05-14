"""Phase 2B research baseline: launch KUKA in the peg-in-hole Gazebo world."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CONFIG_FILE = "research_baseline.yaml"
RESEARCH_ROBOT_XACRO = "lbr_iisy3_r760_research_gripper.urdf.xacro"


def _load_research_baseline_config():
    config_path = os.path.join(
        get_package_share_directory("thesis_bringup"),
        "config",
        CONFIG_FILE,
    )
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    return config["research_baseline"]["ros__parameters"]


def _prepend_resource_path(model_path):
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if not existing_resource_path:
        return model_path
    return model_path + os.pathsep + existing_resource_path


def _safe_home_pose(robot_config):
    named_poses = robot_config.get("named_poses", {})
    pose = named_poses.get("safe_home", robot_config["home_pose"])
    if len(pose) != 6:
        raise ValueError("research_baseline robot safe_home/home_pose must contain 6 joints")
    return pose


def launch_setup(context, *args, **kwargs):
    """Resolve configured research assets and launch the KUKA Gazebo baseline."""
    params = _load_research_baseline_config()
    simulation = params["simulation"]
    robot = params["robot"]
    task = params["task"]

    world_package = simulation["world_package"]
    world_file = simulation["world_file"]
    world_package_share = get_package_share_directory(world_package)
    world_path = os.path.join(world_package_share, "worlds", world_file)
    model_path = os.path.join(world_package_share, "models")

    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_family = LaunchConfiguration("robot_family").perform(context)
    controller_stack = "joint_state_broadcaster + joint_trajectory_controller"
    safe_home_pose = _safe_home_pose(robot)
    namespace = LaunchConfiguration("namespace")
    tf_prefix = (namespace.perform(context) + "_") if namespace.perform(context) != "" else ""
    robot_description_xacro = PathJoinSubstitution(
        [
            FindPackageShare("peg_in_hole_description"),
            "urdf",
            RESEARCH_ROBOT_XACRO,
        ]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_xacro,
            " ",
            "mode:=gazebo",
            " ",
            "prefix:=",
            tf_prefix,
            " ",
            "x:=",
            LaunchConfiguration("x"),
            " ",
            "y:=",
            LaunchConfiguration("y"),
            " ",
            "z:=",
            LaunchConfiguration("z"),
            " ",
            "roll:=",
            LaunchConfiguration("roll"),
            " ",
            "pitch:=",
            LaunchConfiguration("pitch"),
            " ",
            "yaw:=",
            LaunchConfiguration("yaw"),
            " ",
            "initial_joint_1:=",
            str(safe_home_pose[0]),
            " ",
            "initial_joint_2:=",
            str(safe_home_pose[1]),
            " ",
            "initial_joint_3:=",
            str(safe_home_pose[2]),
            " ",
            "initial_joint_4:=",
            str(safe_home_pose[3]),
            " ",
            "initial_joint_5:=",
            str(safe_home_pose[4]),
            " ",
            "initial_joint_6:=",
            str(safe_home_pose[5]),
        ],
        on_stderr="capture",
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        namespace=namespace,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": simulation["use_sim_time"], "mode": "gazebo"},
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -r -v1"]}.items(),
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_server.launch.py"])
        ),
        launch_arguments={
            "world_sdf_file": world_path,
            "container_name": "ros_gz_container",
            "create_own_container": "False",
            "use_composition": "False",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
    )

    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution(
                [FindPackageShare("kuka_gazebo"), "config", "bridge_config.yaml"]
            ),
            "bridge_name": "ros_gz_bridge",
        }.items(),
    )

    contact_ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution(
                [FindPackageShare("thesis_bringup"), "config", "contact_bridge.yaml"]
            ),
            "bridge_name": "contact_ros_gz_bridge",
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            robot_model,
            "-allow_renaming",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    def controller_spawner(controller_name, activate=False):
        args = [controller_name, "-c", "controller_manager", "-n", namespace]
        if not activate:
            args.append("--inactive")
        return Node(package="controller_manager", executable="spawner", arguments=args)

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _prepend_resource_path(model_path)),
        LogInfo(
            msg=(
                "Phase 2B unified research baseline: launching Gazebo world "
                f"{world_package}/worlds/{world_file} ({world_path})"
            )
        ),
        LogInfo(
            msg=(
                "Phase 2B unified research baseline: spawning robot "
                f"{robot['name']} using project robot description "
                f"peg_in_hole_description/urdf/{RESEARCH_ROBOT_XACRO} "
                f"for KUKA model {robot_model} "
                f"(family: {robot_family}) at xyz/rpy "
                f"[{LaunchConfiguration('x').perform(context)}, "
                f"{LaunchConfiguration('y').perform(context)}, "
                f"{LaunchConfiguration('z').perform(context)}, "
                f"{LaunchConfiguration('roll').perform(context)}, "
                f"{LaunchConfiguration('pitch').perform(context)}, "
                f"{LaunchConfiguration('yaw').perform(context)}]"
            )
        ),
        LogInfo(
            msg=(
                "Phase 2B unified research baseline: task "
                f"{task['name']} uses table frame {task['table_frame']}, "
                f"target frame {task['target_frame']}, insertion axis "
                f"{task['insertion_axis']}"
            )
        ),
        LogInfo(
            msg=(
                "Phase 2B unified research baseline: expected controller stack is "
                f"{controller_stack}"
            )
        ),
        LogInfo(
            msg=(
                "Phase 2B unified research baseline: Gazebo initial arm safe_home "
                f"joint pose is {safe_home_pose}"
            )
        ),
        robot_state_publisher,
        gz_sim,
        gz_server,
        spawn_robot,
        ros_gz_bridge,
        contact_ros_gz_bridge,
        controller_spawner("joint_state_broadcaster", activate=True),
        controller_spawner("joint_trajectory_controller", activate=True),
    ]


def generate_launch_description():
    """Launch the canonical KUKA + peg-in-hole research simulation."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="lbr_iisy3_r760",
                description="KUKA robot model passed through to kuka_gazebo.",
            ),
            DeclareLaunchArgument(
                "robot_family",
                default_value="lbr_iisy",
                description="KUKA robot family passed through to kuka_gazebo.",
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            # Tuned research-cell spawn: the table remains centered at
            # x=0.80, y=0.0 with its work surface at z=0.75 m. A floor-mounted
            # robot spawn at z=0.0 made the arm appear under the table even
            # when x/y alignment was correct. The research baseline is a
            # pedestal-mounted KUKA with x=0.80 aligned to the table centerline,
            # y=-0.75 in front of the table, and z=0.75 at table-surface height.
            DeclareLaunchArgument("x", default_value="0.80"),
            DeclareLaunchArgument("y", default_value="-0.75"),
            DeclareLaunchArgument("z", default_value="0.75"),
            DeclareLaunchArgument("roll", default_value="0"),
            DeclareLaunchArgument("pitch", default_value="0"),
            DeclareLaunchArgument("yaw", default_value="1.5708"),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="If true, launch gz_sim GUI. If false, launch gz_server only.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
