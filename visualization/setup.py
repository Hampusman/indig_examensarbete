import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hampus',
    maintainer_email='hampus.h@live.se',
    description='Node for visualizing images',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'rgb_image_visualization = visualization.rgb_image_visualization:main',
            'depth_image_visualization = visualization.depth_image_visualization:main',
            'keypoints_image_visualization = visualization.keypoints_image_visualization:main',
            'poses_image_visualization = visualization.poses_image_visualization:main',
            'visualization_node = visualization.visualization_node:main',
        ],
    },
)
