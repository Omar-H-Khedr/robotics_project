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
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CONFIG_FILE = "research_baseline.yaml"
RESEARCH_ROBOT_XACRO = "lbr_iisy6_r1300_research_gripper.urdf.xacro"


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
    world_file = LaunchConfiguration("world_file").perform(context)
    world_package_share = get_package_share_directory(world_package)
    world_path = os.path.join(world_package_share, "worlds", world_file)
    model_path = os.path.join(world_package_share, "models")

    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_family = LaunchConfiguration("robot_family").perform(context)
    allow_robot_renaming = (
        LaunchConfiguration("allow_robot_renaming").perform(context).strip().lower()
        in ("1", "true", "yes", "on")
    )
    controller_stack = "joint_state_broadcaster + joint_trajectory_controller"
    safe_home_pose = _safe_home_pose(robot)
    namespace = LaunchConfiguration("namespace")
    namespace_value = namespace.perform(context)
    tf_prefix = (namespace_value + "_") if namespace_value != "" else ""
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
                [FindPackageShare("thesis_bringup"), "config", "research_baseline_bridge.yaml"]
            ),
            "bridge_name": "research_baseline_ros_gz_bridge",
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

    ft_sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="bridge_node",
        name="ft_sensor_bridge",
        output="screen",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [FindPackageShare("thesis_bringup"), "config", "ft_sensor_bridge.yaml"]
                ),
            }
        ],
        remappings=[
            (
                "/world/peg_in_hole_world/model/lbr_iisy6_r1300/joint/ft_sensor_joint/sensor/ft_sensor/forcetorque",
                "/ft_sensor_wrench",
            ),
        ],
    )

    # Resolve spawn position from launch configuration
    spawn_x = LaunchConfiguration("x").perform(context)
    spawn_y = LaunchConfiguration("y").perform(context)
    spawn_z = LaunchConfiguration("z").perform(context)
    spawn_roll = LaunchConfiguration("roll").perform(context)
    spawn_pitch = LaunchConfiguration("pitch").perform(context)
    spawn_yaw = LaunchConfiguration("yaw").perform(context)
    xacro_path = robot_description_xacro.perform(context)

    spawn_robot_arguments = [
        "--xacro",
        xacro_path,
        "--name",
        robot_model,
        "--xacro-args",
        "mode:=gazebo",
        "prefix:=",
        f"x:={spawn_x}",
        f"y:={spawn_y}",
        f"z:={spawn_z}",
        f"roll:={spawn_roll}",
        f"pitch:={spawn_pitch}",
        f"yaw:={spawn_yaw}",
        f"initial_joint_1:={safe_home_pose[0]}",
        f"initial_joint_2:={safe_home_pose[1]}",
        f"initial_joint_3:={safe_home_pose[2]}",
        f"initial_joint_4:={safe_home_pose[3]}",
        f"initial_joint_5:={safe_home_pose[4]}",
        f"initial_joint_6:={safe_home_pose[5]}",
        "include_camera:=false",
        "--position-gain",
        LaunchConfiguration("position_gain"),
        "--joint-damping-scale",
        LaunchConfiguration("joint_damping_scale"),
        "--joint-effort-scale",
        LaunchConfiguration("joint_effort_scale"),
        "--controller-config-package",
        LaunchConfiguration("controller_config_package"),
        "--controller-config-path",
        LaunchConfiguration("controller_config_path"),
        "--x",
        "0.0",
        "--y",
        "0.0",
        "--z",
        "0.0",
        "--R",
        "0.0",
        "--P",
        "0.0",
        "--Y",
        "0.0",
    ]
    if allow_robot_renaming:
        spawn_robot_arguments.append("--allow-renaming")

    spawn_robot = Node(
        package="thesis_bringup",
        executable="spawn_robot_sdf",
        arguments=spawn_robot_arguments,
        output="screen",
    )

    def controller_spawner(controller_name, activate=False):
        controller_manager = (
            f"/{namespace_value}/controller_manager"
            if namespace_value
            else "/controller_manager"
        )
        args = [
            controller_name,
            "-c",
            controller_manager,
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "30",
        ]
        if namespace_value:
            args.extend(["-n", namespace])
        if not activate:
            args.append("--inactive")
        return Node(
            package="controller_manager",
            executable="spawner",
            name=f"{controller_name}_spawner",
            arguments=args,
            output="screen",
        )

    joint_state_broadcaster = controller_spawner("joint_state_broadcaster", activate=True)
    joint_trajectory_controller = controller_spawner(
        "joint_trajectory_controller",
        activate=True,
    )

    data_logger = Node(
        package="kuka_task_control",
        executable="data_logger_node",
        parameters=[{"log_rate": 50.0, "log_dir": "/tmp/thesis_logs"}],
        output="screen",
    )

    trajectory_tracking_observer = Node(
        package="thesis_bringup",
        executable="trajectory_tracking_observer",
        parameters=[
            {
                "use_sim_time": simulation["use_sim_time"],
                "state_topic": "/joint_trajectory_controller/controller_state",
                "command_topic": "/joint_trajectory_controller/joint_trajectory",
                "joint_state_topic": "/joint_states",
                "output_dir": LaunchConfiguration("tracking_log_dir"),
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_tracking_observer")),
    )

    wrench_state_observer = Node(
        package="thesis_bringup",
        executable="wrench_state_observer",
        parameters=[
            {
                "use_sim_time": simulation["use_sim_time"],
                "wrench_topic": "/ft_sensor_wrench",
                "state_topic": "/insertion_state",
                "joint_state_topic": "/joint_states",
                "output_dir": LaunchConfiguration("tracking_log_dir"),
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_wrench_observer")),
    )

    contact_state_observer = Node(
        package="thesis_bringup",
        executable="contact_state_observer",
        parameters=[
            {
                "use_sim_time": simulation["use_sim_time"],
                "state_topic": "/insertion_state",
                "contact_topics": [
                    "peg:/gazebo/contacts/peg",
                    "hole:/gazebo/contacts/hole",
                    "target:/gazebo/contacts/target",
                ],
                "output_dir": LaunchConfiguration("tracking_log_dir"),
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_contact_observer")),
    )

    admittance_insertion = Node(
        package="kuka_task_control",
        executable="admittance_insertion_node",
        parameters=[
            {
                "contact_threshold": 5.0,
                "safety_threshold": 350.0,
                "control_rate": 10.0,
                "approach_speed": 0.01,
            }
        ],
        output="screen",
    )

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
        ft_sensor_bridge,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_trajectory_controller,
                on_exit=[admittance_insertion],
            )
        ),
        data_logger,
        trajectory_tracking_observer,
        wrench_state_observer,
        contact_state_observer,
    ]


def generate_launch_description():
    """Launch the canonical KUKA + peg-in-hole research simulation."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="lbr_iisy6_r1300",
                description="Gazebo entity name used when spawning the KUKA robot.",
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
            # y=-0.75 in front of the table, and z=0.735 at the pedestal top
            # plate surface (pedestal model top_plate is at z=0.735).
            DeclareLaunchArgument("x", default_value="0.80"),
            DeclareLaunchArgument("y", default_value="-0.75"),
            DeclareLaunchArgument("z", default_value="0.735"),
            DeclareLaunchArgument("roll", default_value="0"),
            DeclareLaunchArgument("pitch", default_value="0"),
            DeclareLaunchArgument("yaw", default_value="1.5708"),
            DeclareLaunchArgument(
                "world_file",
                default_value=_load_research_baseline_config()["simulation"]["world_file"],
                description="Gazebo world SDF file from peg_in_hole_description/worlds.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="If true, launch gz_sim GUI. If false, launch gz_server only.",
            ),
            DeclareLaunchArgument(
                "allow_robot_renaming",
                default_value="false",
                description=(
                    "If true, allow ros_gz_sim create to rename duplicate robot "
                    "entities instead of failing. Keep false for canonical runs "
                    "because F/T sensor bridge paths include the robot model name."
                ),
            ),
            DeclareLaunchArgument(
                "position_gain",
                default_value="1000.0",
                description=(
                    "Gazebo position_proportional_gain for gz_ros2_control. "
                    "The canonical default keeps the upstream-style value; "
                    "override only for documented tracking experiments."
                ),
            ),
            DeclareLaunchArgument(
                "joint_damping_scale",
                default_value="1.0",
                description=(
                    "Diagnostic multiplier for converted SDF joint damping. "
                    "Default 1.0 preserves canonical robot dynamics."
                ),
            ),
            DeclareLaunchArgument(
                "joint_effort_scale",
                default_value="1.0",
                description=(
                    "Diagnostic multiplier for converted SDF joint effort limits. "
                    "Default 1.0 preserves canonical robot dynamics."
                ),
            ),
            DeclareLaunchArgument(
                "controller_config_package",
                default_value="thesis_bringup",
                description="Package containing the canonical research ros2_control YAML.",
            ),
            DeclareLaunchArgument(
                "controller_config_path",
                default_value="config/research_baseline_ros2_control.yaml",
                description="Path inside controller_config_package for gz_ros2_control parameters.",
            ),
            DeclareLaunchArgument(
                "enable_tracking_observer",
                default_value="true",
                description="If true, passively log joint trajectory controller tracking error.",
            ),
            DeclareLaunchArgument(
                "enable_wrench_observer",
                default_value="true",
                description="If true, passively log F/T wrench by insertion state and peg pose.",
            ),
            DeclareLaunchArgument(
                "enable_contact_observer",
                default_value="true",
                description="If true, passively log contact sensor events by insertion state.",
            ),
            DeclareLaunchArgument(
                "tracking_log_dir",
                default_value="/tmp/thesis_tracking_logs",
                description="Directory for passive trajectory tracking observer CSV and summary.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
