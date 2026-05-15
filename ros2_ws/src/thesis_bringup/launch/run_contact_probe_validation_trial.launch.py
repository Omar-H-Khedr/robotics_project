import os
from pathlib import Path

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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CONTACT_VALIDATION_WORLD = "peg_in_hole_contact_validation_world.sdf"
CONFIG_FILE = "research_baseline.yaml"
RESEARCH_ROBOT_XACRO = "lbr_iisy3_r760_research_gripper.urdf.xacro"
VALIDATION_GZ_CONTACT_TOPIC = (
    "/world/peg_in_hole_contact_validation_world/model/contact_validation_pad/link/"
    "contact_validation_pad_link/sensor/contact_validation_sensor/contact"
)
VALIDATION_ROS_CONTACT_TOPIC = "/gazebo/contacts/validation"


def _workspace_results_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == "ros2_ws":
            return parent / "results" / "baseline_trials"
    return Path.cwd() / "results" / "baseline_trials"


def _prepend_resource_path(model_path: str) -> str:
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if not existing_resource_path:
        return model_path
    return model_path + os.pathsep + existing_resource_path


def _load_research_baseline_config():
    config_path = (
        Path(get_package_share_directory("thesis_bringup"))
        / "config"
        / CONFIG_FILE
    )
    with config_path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    return config["research_baseline"]["ros__parameters"]


def _safe_home_pose(robot_config):
    named_poses = robot_config.get("named_poses", {})
    pose = named_poses.get("safe_home", robot_config["home_pose"])
    if len(pose) != 6:
        raise ValueError("research_baseline robot safe_home/home_pose must contain 6 joints")
    return pose


def _contact_validation_world_path() -> Path:
    return (
        Path(get_package_share_directory("peg_in_hole_description"))
        / "worlds"
        / CONTACT_VALIDATION_WORLD
    )


def _contact_validation_model_path() -> Path:
    return Path(get_package_share_directory("peg_in_hole_description")) / "models"


def _contact_metrics_parameters() -> dict[str, object]:
    config_path = (
        Path(get_package_share_directory("peg_in_hole_metrics"))
        / "config"
        / "contact_metrics.yaml"
    )
    with config_path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}

    parameters = config.get("contact_metrics_node", {}).get("ros__parameters", {})
    parameters["contact_topics"] = [
        f"validation:{VALIDATION_ROS_CONTACT_TOPIC}",
    ]
    return parameters


def launch_setup(context, *args, **kwargs):
    results_root = _workspace_results_root()
    world_path = _contact_validation_world_path()
    model_path = _contact_validation_model_path()
    params = _load_research_baseline_config()
    simulation = params["simulation"]
    robot = params["robot"]

    namespace = LaunchConfiguration("namespace")
    tf_prefix = (namespace.perform(context) + "_") if namespace.perform(context) != "" else ""
    safe_home_pose = _safe_home_pose(robot)
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": [str(world_path), " -r -v1"]}.items(),
    )

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

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            robot["name"],
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

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="contact_probe_validation_ros_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"{VALIDATION_GZ_CONTACT_TOPIC}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ],
        remappings=[
            (VALIDATION_GZ_CONTACT_TOPIC, VALIDATION_ROS_CONTACT_TOPIC),
        ],
    )

    def controller_spawner(controller_name, activate=False):
        args = [controller_name, "-c", "controller_manager", "-n", namespace]
        if not activate:
            args.append("--inactive")
        return Node(package="controller_manager", executable="spawner", arguments=args)

    return [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", _prepend_resource_path(str(model_path))
        ),
        LogInfo(msg="launching contact probe validation world"),
        LogInfo(msg="spawning KUKA for visual/workcell consistency"),
        LogInfo(msg="KUKA task motion disabled"),
        LogInfo(msg="passive contact probe validates contact instrumentation"),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        controller_spawner("joint_state_broadcaster", activate=True),
        controller_spawner("joint_trajectory_controller", activate=True),
        ros_gz_bridge,
        Node(
            package="peg_in_hole_metrics",
            executable="contact_metrics_node",
            name="contact_metrics_node",
            output="screen",
            parameters=[_contact_metrics_parameters()],
        ),
        Node(
            package="experiment_manager",
            executable="baseline_trial_manager",
            name="baseline_trial_manager",
            output="screen",
            parameters=[
                {
                    "results_root": str(results_root),
                    "trial_mode": "contact_probe_validation",
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("x", default_value="0.80"),
            DeclareLaunchArgument("y", default_value="-0.75"),
            DeclareLaunchArgument("z", default_value="0.75"),
            DeclareLaunchArgument("roll", default_value="0"),
            DeclareLaunchArgument("pitch", default_value="0"),
            DeclareLaunchArgument("yaw", default_value="1.5708"),
            OpaqueFunction(function=launch_setup),
        ]
    )
