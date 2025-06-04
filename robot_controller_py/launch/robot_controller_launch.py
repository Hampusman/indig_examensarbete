from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ip',
            default_value='192.168.1.102',
            description='Robot ip.'),

        DeclareLaunchArgument(
            'port',
            default_value='63352',
            description='Gripper port.'),

        DeclareLaunchArgument(
            'force',
            default_value='50',
            description='Gripper force.'),

        Node(
            package='robot_controller_py',
            executable='controller',
            namespace='robot',
            name='controller',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'ip': LaunchConfiguration('ip')}
            ]
        ),
        Node(
            package='robot_controller_py',
            executable='receiver',
            namespace='robot',
            name='receiver',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'ip': LaunchConfiguration('ip')}
            ]
        ),
        Node(
            package='robot_controller_py',
            executable='gripper',
            namespace='robot',
            name='gripper',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'ip': LaunchConfiguration('ip')},
                {'port': LaunchConfiguration('port')},
                {'force': LaunchConfiguration('force')}
            ]
        )
    ])
