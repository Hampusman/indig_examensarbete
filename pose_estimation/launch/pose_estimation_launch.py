from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug for node.'),

        Node(
            package='pose_estimation',
            executable='pose_estimation_node',
            namespace='vision',
            name='pose_estimation',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'debug': LaunchConfiguration('debug')}
            ]
        ),
    ])
