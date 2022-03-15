from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np


def generate_launch_description():
    return LaunchDescription([
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=[
                 '-0.141', '1.212', '1.184', '-0.005', '0.962,', '-0.272',
                 '0.008', 'map', 'camera_static'
             ]),
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=[
                 '0', '0.205', '0', '0', '0', '0', '1', 'marker_[42]', 'map'
             ]),
        Node(package='mep3_localization', executable='robot_detection'),
    ])
