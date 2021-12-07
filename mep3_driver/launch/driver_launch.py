import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_driver')

    robot_description = pathlib.Path(os.path.join(
        package_dir, 'resource', 'mep3_big_config.urdf')).read_text()
    ros2_control_params = os.path.join(
        package_dir, 'resource', 'mep3_big_ros2_control.yaml')

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            ros2_control_params
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel')
        ],
        output='screen'
    )

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        arguments=['diffdrive_controller']
    )

    return LaunchDescription([
        controller_manager_node,
        diffdrive_controller_spawner
    ])
