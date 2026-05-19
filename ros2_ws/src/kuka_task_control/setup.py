from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = "kuka_task_control"
top_level_config_files = [
    path for path in glob("config/*") if Path(path).is_file()
]
moveit_overlay_files = glob("config/moveit_lbr_iisy6_r1300/*")

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", top_level_config_files),
        (
            "share/" + package_name + "/config/moveit_lbr_iisy6_r1300",
            moveit_overlay_files,
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Omar Khedr",
    maintainer_email="omar.khedr@gu.edu.eg",
    description="Task-level KUKA control interfaces for peg-in-hole assembly experiments.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "baseline_trajectory_client = kuka_task_control.baseline_trajectory_client:main",
            "baseline_joint_sequence_executor = kuka_task_control.baseline_joint_sequence_executor:main",
            "task_trajectory_executor = kuka_task_control.task_trajectory_executor:main",
            "segmented_guarded_contact_executor = kuka_task_control.segmented_guarded_contact_executor:main",
            "segmented_contact_executor = kuka_task_control.segmented_contact_executor:main",
            "peg_hole_frame_publisher = kuka_task_control.peg_hole_frame_publisher:main",
            "cartesian_insertion_diagnostics = kuka_task_control.cartesian_insertion_diagnostics:main",
            "ik_feasibility_diagnostics = kuka_task_control.ik_feasibility_diagnostics:main",
            "tool_axis_audit = kuka_task_control.tool_axis_audit:main",
            "cartesian_orientation_target_calculator = kuka_task_control.cartesian_orientation_target_calculator:main",
            "cartesian_insertion_dry_run_planner = kuka_task_control.cartesian_insertion_dry_run_planner:main",
            "execution_gate_monitor = kuka_task_control.execution_gate_monitor:main",
            "ik_backend_audit = kuka_task_control.ik_backend_audit:main",
            "moveit_config_audit = kuka_task_control.moveit_config_audit:main",
            "moveit_launch_readiness_audit = kuka_task_control.moveit_launch_readiness_audit:main",
            "semantic_model_validator = kuka_task_control.semantic_model_validator:main",
            "robot_description_semantic_diagnostics = kuka_task_control.robot_description_semantic_diagnostics:main",
        ]
    },
)
