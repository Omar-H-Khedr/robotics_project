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
        ]
    },
)
