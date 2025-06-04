import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pose_monitoring'

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
    maintainer='my',
    maintainer_email='hampus.h@live.se',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_stability_tracker_node = pose_monitoring.pose_stability_tracker_node:main',
            'feasibility_filter_node = pose_monitoring.feasibility_filter_node:main',

        ],
    },
)
