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

        DeclareLaunchArgument(
            'auto',
            default_value='true',
            description='Enable automatic mode.'),

        Node(
            package='pose_monitoring',
            executable='feasibility_filter_node',
            namespace='monitoring',
            name='feasibility_filter',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'debug': LaunchConfiguration('debug')}
            ]
        ),
        Node(
            package='pose_monitoring',
            executable='pose_stability_tracker_node',
            namespace='monitoring',
            name='stability_tracker',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'debug': LaunchConfiguration('debug'),
                 'auto': LaunchConfiguration('auto')},
            ]
        ),
    ])
