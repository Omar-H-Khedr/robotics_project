from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'kuka_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omar',
    maintainer_email='omar.khedr@gu.edu.eg',
    description='Project-local launch and visualization assets for KUKA LBR iisy robots.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
