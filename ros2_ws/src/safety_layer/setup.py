from glob import glob
from setuptools import setup

package_name = "safety_layer"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
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
    description="Safety filtering and constraint monitoring package for adaptive peg-in-hole control.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "safety_monitor = safety_layer.safety_monitor:main",
        ]
    },
)
