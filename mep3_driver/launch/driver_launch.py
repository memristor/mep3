import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_driver')

    namespace = LaunchConfiguration('namespace', default='big')

    controller_params_file = LaunchConfiguration(
        'controller_params',
        default=os.path.join(get_package_share_directory('mep3_bringup'), 'resource', 'ros2_control_big.yaml')
    )

    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'config_big.urdf')).read_text()

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ],
        remappings=[
            ('/big/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('/odom', 'odom'),
            ('/tf', 'tf')
        ],
        namespace=namespace,
        output='screen'
    )

    tf_base_link_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'laser'],
        namespace=namespace,
        remappings=[
            ('/tf_static', 'tf_static')
        ],
    )

    socketcan_bridge = Node(
        package='mep3_driver',
        executable='socketcan_bridge',
        output='screen',
        namespace=namespace
    )

    return LaunchDescription([
        controller_manager_node,
        tf_base_link_laser,
        socketcan_bridge
    ])
