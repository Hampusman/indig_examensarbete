import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_controller_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hampus',
    maintainer_email='hampus.h@live.se',
    description='Nodes for controlling and retrieving data from robot and gripper',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'controller = robot_controller_py.controller:main',
            'receiver = robot_controller_py.receiver:main',
            'gripper = robot_controller_py.gripper:main',
        ],
    },
)
