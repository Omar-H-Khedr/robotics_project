from glob import glob
from setuptools import find_packages, setup

package_name = "peg_in_hole_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/meshes", glob("meshes/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Omar Khedr",
    maintainer_email="omar.khedr@gu.edu.eg",
    description="Task-scene description package for KUKA peg-in-hole assembly in Gazebo.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": []},
)
