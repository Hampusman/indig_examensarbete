from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pose_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pose_estimation'), 'launch', 'pose_estimation_launch.py')
        )
    )

    pose_monitoring_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pose_monitoring'), 'launch', 'pose_monitoring_launch.py')
        )
    )

    motion_executor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('motion_executor'), 'launch', 'motion_executor_launch.py')
        )
    )

    robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_controller_py'), 'launch', 'robot_controller_launch.py')
        )
    )

    return LaunchDescription([
        pose_estimation_launch,
        pose_monitoring_launch,
        motion_executor_launch,
        robot_controller_launch,
    ])
