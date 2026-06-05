from glob import glob
from setuptools import find_packages, setup

package_name = "perception_pipeline"

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
    install_requires=["setuptools", "numpy", "pandas", "pyarrow", "Pillow", "torch"],
    zip_safe=True,
    maintainer="Omar Khedr",
    maintainer_email="omar.khedr@gu.edu.eg",
    description="Gazebo RGB-D and perception interface package for peg-in-hole state estimation.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "multimodal_observation_logger = perception_pipeline.multimodal_observation_logger:main",
            "context_vector_extractor = perception_pipeline.context_vector_extractor:main",
            "v2_13_context_encoder = perception_pipeline.v2_13_context_encoder:main",
            "v2_14_context_conditioned_action = perception_pipeline.v2_14_context_conditioned_action:main",
            "v2_15_context_action_ablation = perception_pipeline.v2_15_context_action_ablation:main",
            "live_v2_14_inference_node = perception_pipeline.live_v2_14_inference_node:main",
        ],
    },
)
