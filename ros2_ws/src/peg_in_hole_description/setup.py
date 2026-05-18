from glob import glob
import os
from setuptools import find_packages, setup

package_name = "peg_in_hole_description"


def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        files = [os.path.join(path, filename) for filename in filenames]
        if files:
            paths.append((os.path.join("share", package_name, path), files))
    return paths


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
    ]
    + package_files("models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Omar Khedr",
    maintainer_email="omar.khedr@gu.edu.eg",
    description="Task-scene description package for KUKA peg-in-hole assembly in Gazebo.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": []},
)
