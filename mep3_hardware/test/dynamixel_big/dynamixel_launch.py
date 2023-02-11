import os
import pathlib
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = os.path.dirname(os.path.abspath(__file__))
    controller_params_file = os.path.join(
        package_dir, 'controllers.yaml')
    robot_description = pathlib.Path(os.path.join(
        package_dir, 'description.urdf')).read_text()

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controller_params_file
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=['joint_position_controller']
        )
    ])
