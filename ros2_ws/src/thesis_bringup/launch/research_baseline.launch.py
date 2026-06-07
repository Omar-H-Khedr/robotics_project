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
from launch_ros.parameter_descriptions import ParameterValue
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
    use_position_controller = (
        LaunchConfiguration("use_position_controller").perform(context).strip().lower()
        in ("1", "true", "yes", "on")
    )
    if use_position_controller:
        controller_stack = "joint_state_broadcaster + position_controller (via trajectory_position_bridge)"
    else:
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
    inject_velocity_state = (
        LaunchConfiguration("inject_velocity_state").perform(context).strip().lower()
        in ("1", "true", "yes", "on")
    )
    inject_script = os.path.join(
        get_package_share_directory("thesis_bringup"),
        "scripts",
        "inject_velocity_state_urdf.py",
    )
    xacro_args = [
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
    ]
    if inject_velocity_state:
        # xacro must find the kuka_lbr_iisy_support package; source the
        # workspace's setup.bash first. The Command() runs in a subprocess
        # without our interactive environment, so we need to source it here.
        # launch.substitutions.Command joins all items with '' then shlex.splits
        # the result, so the bash -c argument must be a single shlex-quoted
        # token. We assemble the bash command at this point (launch_setup
        # runs at launch time) so the substitutions are already resolved.
        # Important: join xacro_args with '' (not ' ') so that "x:=" and
        # "0.80" stay adjacent (i.e. "x:=0.80" not "x:= 0.80"); a space
        # would let bash word-split them into two separate xacro arguments.
        workspace_setup_str = PathJoinSubstitution([
            FindPackageShare("thesis_bringup"),
            "..", "..", "..", "..",
            "install", "setup.bash",
        ]).perform(context)
        inject_py_str = FindExecutable(name="python3").perform(context)
        xacro_strs = [
            s.perform(context) if hasattr(s, "perform") else str(s)
            for s in xacro_args
        ]
        xacro_joined = "".join(xacro_strs)

        def _shq(s):
            """Single-quote a string for safe inclusion in a shell command."""
            return "'" + s.replace("'", "'\\''") + "'"

        # Build the bash command. Wrap the workspace_setup in single quotes
        # (its path may contain $ or other shell-special chars), but leave
        # the xacro call and the python pipe unquoted so bash word-splits
        # them into argv correctly.
        bash_inner = (
            "source /opt/ros/jazzy/setup.bash && source "
            + _shq(workspace_setup_str)
            + " && "
            + xacro_joined
            + " 2>/dev/null | "
            + inject_py_str
            + " "
            + inject_script
        )
        # Wrap the bash command in double quotes for shlex (so the
        # launch framework's shlex.split gives us a single argv entry for
        # /bin/bash -c). Escape any embedded double quotes/backslashes.
        bash_arg = (
            '"' + bash_inner.replace("\\", "\\\\").replace('"', '\\"') + '"'
        )
        xacro_args = ["/bin/bash", " ", "-c", " ", bash_arg]
    # DEBUG
    # print("DEBUG xacro_args joined:", ''.join(str(s) for s in xacro_args), flush=True)
    robot_description_content = Command(
        xacro_args,
        on_stderr="ignore",
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

    safety_monitor = Node(
        package="safety_layer",
        executable="safety_monitor",
        name="safety_monitor",
        output="screen",
        parameters=[
            {
                "config_path": PathJoinSubstitution(
                    [FindPackageShare("safety_layer"), "config", "safety_limits.yaml"]
                ),
            }
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

    effective_controller_config_path = (
        LaunchConfiguration("position_controller_config_path").perform(context)
        if use_position_controller
        else LaunchConfiguration("controller_config_path").perform(context)
    )
    if (
        LaunchConfiguration("inject_velocity_state").perform(context).strip().lower()
        in ("1", "true", "yes", "on")
        and not use_position_controller
    ):
        effective_controller_config_path = (
            LaunchConfiguration("velocity_state_controller_config_path").perform(context)
        )

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
        "--position-derivative-gain",
        LaunchConfiguration("position_derivative_gain"),
        "--joint-damping-scale",
        LaunchConfiguration("joint_damping_scale"),
        "--joint-effort-scale",
        LaunchConfiguration("joint_effort_scale"),
        "--controller-config-package",
        LaunchConfiguration("controller_config_package"),
        "--controller-config-path",
        effective_controller_config_path,
    ]
    inject_velocity_state = (
        LaunchConfiguration("inject_velocity_state").perform(context).strip().lower()
        in ("1", "true", "yes", "on")
    )
    spawn_robot_arguments.extend([
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
    ])
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
    if use_position_controller:
        position_controller = controller_spawner(
            "position_controller",
            activate=True,
        )
        joint_trajectory_controller = None
    else:
        joint_trajectory_controller = controller_spawner(
            "joint_trajectory_controller",
            activate=True,
        )
        position_controller = None

    data_logger = Node(
        package="kuka_task_control",
        executable="data_logger_node",
        parameters=[{"log_rate": 50.0, "log_dir": "/tmp/thesis_logs"}],
        output="screen",
    )

    if use_position_controller:
        trajectory_position_bridge = Node(
            package="thesis_bringup",
            executable="trajectory_position_bridge",
            parameters=[
                {
                    "input_trajectory_topic": "/joint_trajectory_controller/joint_trajectory",
                    "output_command_topic": "/position_controller/commands",
                    "joint_names": [
                        "joint_1", "joint_2", "joint_3",
                        "joint_4", "joint_5", "joint_6",
                    ],
                    "interp_rate_hz": 250.0,
                    "publish_on_update": True,
                    "hold_last_point": True,
                }
            ],
            output="screen",
        )
    else:
        trajectory_position_bridge = None

    if use_position_controller:
        observer_state_topic = "/position_controller/controller_state"
        observer_command_topic = "/joint_trajectory_controller/joint_trajectory"
    else:
        observer_state_topic = "/joint_trajectory_controller/controller_state"
        observer_command_topic = "/joint_trajectory_controller/joint_trajectory"

    trajectory_tracking_observer = Node(
        package="thesis_bringup",
        executable="trajectory_tracking_observer",
        parameters=[
            {
                "use_sim_time": simulation["use_sim_time"],
                "state_topic": observer_state_topic,
                "command_topic": observer_command_topic,
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

    perception_observation_logger = Node(
        package="perception_pipeline",
        executable="multimodal_observation_logger",
        parameters=[
            {
                "use_sim_time": simulation["use_sim_time"],
                "output_dir": LaunchConfiguration("perception_log_dir"),
                "rate_hz": 20.0,
                "log_rgb": True,
                "log_depth": True,
                "rgb_topic": "/d405/color/image_raw",
                "depth_topic": "/d405/depth/image_rect_raw",
                "rgb_info_topic": "/d405/color/camera_info",
                "depth_info_topic": "/d405/depth/camera_info",
                "joint_state_topic": "/joint_states",
                "wrench_topic": "/ft_sensor_wrench",
                "task_phase_topic": "/task_phase",
                "safety_status_topic": "/safety_status",
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_perception_logging")),
    )

    synthetic_phase_publisher_node = Node(
        package="thesis_bringup",
        executable="synthetic_phase_publisher",
        parameters=[
            {
                "use_sim_time": simulation["use_sim_time"],
                "schedule_path": LaunchConfiguration("synthetic_phase_schedule_path"),
                "publish_rate_hz": 10.0,
                "loop": False,
                "autostop": True,
                "topic": "/task_phase",
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_synthetic_phases")),
    )

    live_v2_14_inference_node = Node(
        package="perception_pipeline",
        executable="live_v2_14_inference_node",
        parameters=[
            {
                "use_sim_time": simulation["use_sim_time"],
                "encoder_pt": LaunchConfiguration("v2_14_encoder_pt"),
                "scaler_json": LaunchConfiguration("v2_14_scaler_json"),
                "action_classifier_pt": LaunchConfiguration("v2_14_action_classifier_pt"),
                "output_dir": LaunchConfiguration("v2_14_live_inference_dir"),
                "rate_hz": 20.0,
                "rgb_topic": "/d405/color/image_raw",
                "depth_topic": "/d405/depth/image_rect_raw",
                "joint_state_topic": "/joint_states",
                "ft_topic": "/ft_sensor_wrench",
                "task_phase_topic": "/task_phase",
                "safety_status_topic": "/safety_status",
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_v2_14_live_inference")),
    )

    admittance_insertion = Node(
        package="kuka_task_control",
        executable="admittance_insertion_node",
        parameters=[
            {
                "contact_threshold": 5.0,
                "safety_threshold": 350.0,
                "control_rate": ParameterValue(
                    LaunchConfiguration("control_rate"),
                    value_type=float,
                ),
                "approach_speed": 0.01,
                "search_recenter_duration_s": ParameterValue(
                    LaunchConfiguration("search_recenter_duration_s"),
                    value_type=float,
                ),
                "search_settle_duration_s": ParameterValue(
                    LaunchConfiguration("search_settle_duration_s"),
                    value_type=float,
                ),
                "insert_handoff_hold_duration_s": ParameterValue(
                    LaunchConfiguration("insert_handoff_hold_duration_s"),
                    value_type=float,
                ),
                "insert_handoff_timeout_s": ParameterValue(
                    LaunchConfiguration("insert_handoff_timeout_s"),
                    value_type=float,
                ),
                "use_sim_time": simulation["use_sim_time"],
            }
        ],
        output="screen",
    )

    actions: list = [
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
        safety_monitor,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster],
            )
        ),
    ]
    if use_position_controller:
        event_chain = [
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[position_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=position_controller,
                    on_exit=[trajectory_position_bridge, admittance_insertion],
                ),
                condition=UnlessCondition(LaunchConfiguration("enable_synthetic_phases")),
            ),
        ]
    else:
        event_chain = [
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
                ),
                condition=UnlessCondition(LaunchConfiguration("enable_synthetic_phases")),
            ),
        ]
    actions.extend(event_chain)
    actions.append(data_logger)
    actions.extend([
        trajectory_tracking_observer,
        wrench_state_observer,
        contact_state_observer,
        perception_observation_logger,
        synthetic_phase_publisher_node,
        live_v2_14_inference_node,
    ])
    return actions


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
                "control_rate",
                default_value="10.0",
                description=(
                    "Admittance insertion task state-machine rate in Hz. "
                    "Default 10 Hz preserves canonical behavior; override only "
                    "for documented SEARCH/hold cadence diagnostics."
                ),
            ),
            DeclareLaunchArgument(
                "insert_handoff_hold_duration_s",
                default_value="2.0",
                description=(
                    "No-descent INSERT handoff hold command duration. "
                    "Default 2.0 s preserves canonical behavior; this does "
                    "not change the 1 mm clearance or 8-tick stability gate."
                ),
            ),
            DeclareLaunchArgument(
                "search_recenter_duration_s",
                default_value="5.0",
                description=(
                    "Centered SEARCH recenter command duration. Default "
                    "5.0 s preserves canonical behavior; this does not "
                    "change the 1 mm clearance or 8-tick SEARCH gate."
                ),
            ),
            DeclareLaunchArgument(
                "search_settle_duration_s",
                default_value="6.0",
                description=(
                    "SEARCH post-command wait before another recenter/spiral "
                    "command is allowed. Default 6.0 s preserves canonical "
                    "behavior and must be at least the recenter duration."
                ),
            ),
            DeclareLaunchArgument(
                "insert_handoff_timeout_s",
                default_value="6.0",
                description=(
                    "Maximum wait for INSERT handoff feedback to satisfy the "
                    "fixed physical-clearance stability gate before aborting. "
                    "Default 6.0 s preserves canonical behavior."
                ),
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
                "position_derivative_gain",
                default_value="0.0",
                description=(
                    "gz_ros2_control position_derivative_gain. Adds damping "
                    "at the controller level to reduce oscillation during "
                    "centered hold. Default 0.0 preserves canonical behavior."
                ),
            ),
            DeclareLaunchArgument(
                "inject_velocity_state",
                default_value="false",
                description=(
                    "If true, add a velocity state_interface to every joint "
                    "in the URDF (via spawn_robot_sdf.py) so the JTC's D-term "
                    "can use real joint velocity from the Gazebo system "
                    "instead of finite-difference of position. The JTC's "
                    "state_interfaces list (in the ros2_control YAML) must "
                    "include velocity as well; see the diagnostic launch "
                    "for the matching controller config. Default false "
                    "preserves canonical behavior."
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
                "position_controller_config_path",
                default_value="config/research_baseline_position_controller.yaml",
                description=(
                    "Path inside controller_config_package for the position_controller "
                    "gz_ros2_control parameters. Only used when use_position_controller:=true."
                ),
            ),
            DeclareLaunchArgument(
                "velocity_state_controller_config_path",
                default_value="config/research_baseline_velocity_state.yaml",
                description=(
                    "Path inside controller_config_package for the JTC "
                    "gz_ros2_control parameters with velocity state. Only used "
                    "when inject_velocity_state:=true and use_position_controller:=false."
                ),
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
                "use_position_controller",
                default_value="false",
                description=(
                    "If true, spawn position_controllers/JointGroupPositionController "
                    "and the trajectory_position_bridge node, and skip the JTC spawn. "
                    "Default false preserves the canonical JTC path."
                ),
            ),
            DeclareLaunchArgument(
                "tracking_log_dir",
                default_value="/tmp/thesis_tracking_logs",
                description="Directory for passive trajectory tracking observer CSV and summary.",
            ),
            DeclareLaunchArgument(
                "enable_perception_logging",
                default_value="false",
                description=(
                    "If true, spawn the multimodal_observation_logger from "
                    "perception_pipeline. Subscribes to /d405/*, /joint_states, "
                    "/ft_sensor_wrench, /task_phase, /safety_status. The D405 "
                    "RGB-D camera is already defined in peg_in_hole_world.sdf "
                    "and bridged to ROS by research_baseline_bridge.yaml, so "
                    "no Gazebo change is required to enable this."
                ),
            ),
            DeclareLaunchArgument(
                "perception_log_dir",
                default_value="/tmp/thesis_perception_logs",
                description=(
                    "Output directory for multimodal_observation_log.csv "
                    "when enable_perception_logging is true."
                ),
            ),
            DeclareLaunchArgument(
                "enable_synthetic_phases",
                default_value="false",
                description=(
                    "If true, spawn synthetic_phase_publisher from "
                    "thesis_bringup to publish /task_phase on a scripted "
                    "schedule (MOVE_TO_START -> APPROACH -> SEARCH -> "
                    "INSERT -> INSERTED -> ABORT by default). Combined with "
                    "enable_perception_logging=true, this produces a "
                    "multi-phase labeled CSV for v2_14 / v2_15 offline "
                    "training. The synthetic_phase_publisher and the "
                    "admittance_insertion_node both publish /task_phase; "
                    "in synthetic mode, the launch file excludes the "
                    "admittance_insertion_node so there is no conflict."
                ),
            ),
            DeclareLaunchArgument(
                "synthetic_phase_schedule_path",
                default_value="",
                description=(
                    "Optional YAML file with top-level 'schedule:' list of "
                    "{phase, duration_s} entries to override the builtin "
                    "default schedule of synthetic_phase_publisher."
                ),
            ),
            DeclareLaunchArgument(
                "enable_v2_14_live_inference",
                default_value="false",
                description=(
                    "If true, spawn live_v2_14_inference_node from "
                    "perception_pipeline. Subscribes to /d405/*, /joint_states, "
                    "/ft_sensor_wrench, /task_phase, /safety_status; loads the "
                    "v2_13_v2 frozen encoder and v2_14 PhaseHead; publishes "
                    "/v2_14/predicted_phase (String), /v2_14/target_joint_pose "
                    "(Float64MultiArray, 6 floats), and /v2_14/latent "
                    "(Float64MultiArray, 32 floats); logs a CSV with "
                    "ground-truth and predicted phase per tick. This is a "
                    "passive inference node: it does not publish "
                    "JointTrajectory corrections. The integration with the "
                    "JTC is a follow-up milestone."
                ),
            ),
            DeclareLaunchArgument(
                "v2_14_live_inference_dir",
                default_value="diagnostics/perception_pipeline_live_v2_14_v1",
                description=(
                    "Output directory for the live_v2_14_inference_log.csv "
                    "when enable_v2_14_live_inference is true."
                ),
            ),
            DeclareLaunchArgument(
                "v2_14_encoder_pt",
                default_value="diagnostics/perception_pipeline_v2_13_encoder_v2/encoder.pt",
            ),
            DeclareLaunchArgument(
                "v2_14_scaler_json",
                default_value="diagnostics/perception_pipeline_v2_13_encoder_v2/scaler.json",
            ),
            DeclareLaunchArgument(
                "v2_14_action_classifier_pt",
                default_value="diagnostics/perception_pipeline_v2_14_action/action_classifier.pt",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
