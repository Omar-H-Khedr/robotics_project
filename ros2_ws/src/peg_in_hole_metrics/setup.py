from glob import glob
from setuptools import find_packages, setup

package_name = "peg_in_hole_metrics"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Omar Khedr",
    maintainer_email="omar.khedr@gu.edu.eg",
    description="Contact and insertion metrics nodes for peg-in-hole research trials.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "contact_metrics_node = peg_in_hole_metrics.contact_metrics_node:main",
        ]
    },
)
