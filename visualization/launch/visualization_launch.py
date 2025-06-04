from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'keypoints',
            default_value='true',
            description='Enable keypoints drawing.'),
        DeclareLaunchArgument(
            'estimations',
            default_value='true',
            description='Enable estimations drawing.'),
        DeclareLaunchArgument(
            'feasible',
            default_value='true',
            description='Enable feasible drawing.'),

        Node(
            package='visualization',
            executable='visualization_node',
            namespace='visualization',
            name='visualization',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'keypoints': LaunchConfiguration('keypoints'),
                 'estimations': LaunchConfiguration('estimations'),
                 'feasible': LaunchConfiguration('feasible'), }]
        ),
    ])
