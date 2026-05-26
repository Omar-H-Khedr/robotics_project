from glob import glob
from setuptools import find_packages, setup

package_name = "thesis_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Omar Khedr",
    maintainer_email="omar.khedr@gu.edu.eg",
    description="Top-level launch and configuration package for the safe adaptive KUKA peg-in-hole research framework.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "controller_readiness_gate = thesis_bringup.controller_readiness_gate:main",
            "proposal_simulation_cell_monitor = thesis_bringup.proposal_simulation_cell_monitor:main",
            "proposal_simulation_cell_v1_1_validator = thesis_bringup.proposal_simulation_cell_v1_1_validator:main",
            "proposal_simulation_cell_v1_2_rgbd_validator = thesis_bringup.proposal_simulation_cell_v1_2_rgbd_validator:main",
            "proposal_simulation_cell_v1_3_contact_validator = thesis_bringup.proposal_simulation_cell_v1_3_contact_validator:main",
            "proposal_simulation_cell_v1_5_safety_virtual_force_node = thesis_bringup.proposal_simulation_cell_v1_5_safety_virtual_force_node:main",
            "proposal_simulation_cell_v1_6_readiness_gate_node = thesis_bringup.proposal_simulation_cell_v1_6_readiness_gate_node:main",
            "proposal_simulation_cell_v1_7_pre_control_contract_node = thesis_bringup.proposal_simulation_cell_v1_7_pre_control_contract_node:main",
            "proposal_simulation_cell_v1_8_control_scaffold_node = thesis_bringup.proposal_simulation_cell_v1_8_control_scaffold_node:main",
            "proposal_simulation_cell_v1_9_control_law_dry_run_node = thesis_bringup.proposal_simulation_cell_v1_9_control_law_dry_run_node:main",
            "proposal_simulation_cell_v1_10_experiment_matrix_node = thesis_bringup.proposal_simulation_cell_v1_10_experiment_matrix_node:main",
            "proposal_simulation_cell_v1_11_single_scenario_loader_node = thesis_bringup.proposal_simulation_cell_v1_11_single_scenario_loader_node:main",
            "proposal_simulation_cell_v1_12_scenario_batch_selector_node = thesis_bringup.proposal_simulation_cell_v1_12_scenario_batch_selector_node:main",
            "proposal_simulation_cell_v1_13_batch_execution_plan_node = thesis_bringup.proposal_simulation_cell_v1_13_batch_execution_plan_node:main",
            "proposal_simulation_cell_v1_14_batch_dry_run_orchestrator_node = thesis_bringup.proposal_simulation_cell_v1_14_batch_dry_run_orchestrator_node:main",
            "proposal_simulation_cell_v1_15_evidence_package_node = thesis_bringup.proposal_simulation_cell_v1_15_evidence_package_node:main",
            "proposal_simulation_cell_v1_16_reproducibility_checklist_node = thesis_bringup.proposal_simulation_cell_v1_16_reproducibility_checklist_node:main",
            "proposal_simulation_cell_v1_17_release_index_node = thesis_bringup.proposal_simulation_cell_v1_17_release_index_node:main",
            "proposal_simulation_cell_v2_0_motion_smoke_test_node = thesis_bringup.proposal_simulation_cell_v2_0_motion_smoke_test_node:main",
            "proposal_simulation_cell_v2_1_motion_validation_suite_node = thesis_bringup.proposal_simulation_cell_v2_1_motion_validation_suite_node:main",
            "proposal_simulation_cell_v2_2_moveit_ik_diagnostic_node = thesis_bringup.proposal_simulation_cell_v2_2_moveit_ik_diagnostic_node:main",
            "proposal_simulation_cell_v2_3_model_alignment_plan_only_node = thesis_bringup.proposal_simulation_cell_v2_3_model_alignment_plan_only_node:main",
            "proposal_simulation_cell_v2_4_moveit_gazebo_execution_node = thesis_bringup.proposal_simulation_cell_v2_4_moveit_gazebo_execution_node:main",
            "proposal_simulation_cell_v2_5_guarded_task_sequence_node = thesis_bringup.proposal_simulation_cell_v2_5_guarded_task_sequence_node:main",
            "proposal_simulation_cell_v2_6_contact_gated_guarded_approach_node = thesis_bringup.proposal_simulation_cell_v2_6_contact_gated_guarded_approach_node:main",
            "proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_node = thesis_bringup.proposal_simulation_cell_v2_7_contact_triggered_guarded_touch_node:main",
            "proposal_simulation_cell_v2_8_contact_reachability_trigger_node = thesis_bringup.proposal_simulation_cell_v2_8_contact_reachability_trigger_node:main",
        ]
    },
)
