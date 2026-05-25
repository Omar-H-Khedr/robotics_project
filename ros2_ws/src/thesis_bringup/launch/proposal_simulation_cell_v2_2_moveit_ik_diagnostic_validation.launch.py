from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    moveit_share = get_package_share_directory("kuka_lbr_iisy_moveit_config")
    with open(moveit_share + "/config/ompl_planning.yaml", "r", encoding="utf-8") as planning_file:
        ompl_planning = yaml.safe_load(planning_file)

    config_path = PathJoinSubstitution(
        [FindPackageShare("thesis_bringup"), "config", "proposal_simulation_cell_v2_2.yaml"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("kuka_lbr_iisy_support"),
                            "urdf",
                            "lbr_iisy3_r760.urdf.xacro",
                        ]
                    ),
                    " mode:=mock",
                ],
                on_stderr="capture",
            ),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    "cat ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("kuka_lbr_iisy_moveit_config"),
                            "urdf",
                            "lbr_iisy3_r760.srdf",
                        ]
                    ),
                ]
            ),
            value_type=str,
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": {
            "manipulator": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_search_resolution": 0.005,
                "kinematics_solver_timeout": 0.05,
                "kinematics_solver_attempts": 1,
            }
        }
    }
    execution_disabled = {
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
        "trajectory_execution_allowed": False,
        "planning_execution_allowed": False,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "use_sim_time": False,
    }
    planning_pipeline_diagnostic = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="proposal_simulation_cell_v2_2_robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False, "mode": "mock"}],
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_diagnostic,
            execution_disabled,
        ],
    )
    diagnostic_node = Node(
        package="thesis_bringup",
        executable="proposal_simulation_cell_v2_2_moveit_ik_diagnostic_node",
        name="proposal_simulation_cell_v2_2_moveit_ik_diagnostic_node",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"output_dir": LaunchConfiguration("output_dir")},
        ],
    )

    return [
        LogInfo(
            msg=(
                "proposal_simulation_cell_v2_2_moveit_ik_diagnostic_validation: "
                "MoveIt IK diagnostics only. /compute_ik may be called once; trajectory "
                "execution, controller execution, planning execution, FollowJointTrajectory "
                "execution, and real robot endpoints remain disabled."
            )
        ),
        robot_state_publisher,
        move_group,
        diagnostic_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=diagnostic_node,
                on_exit=[EmitEvent(event=Shutdown(reason="v2.2 MoveIt IK diagnostic captured"))],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_dir",
                default_value="diagnostics/proposal_simulation_cell_v2_2",
                description="Directory for proposal simulation cell v2.2 diagnostics.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
