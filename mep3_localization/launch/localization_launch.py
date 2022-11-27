#!/usr/bin/env python3

# Copyright 2022 Memristor Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    """
    It is necessary to have a prediction of the camera position
    in order to detect incorrect orientations of ArUco markers.

    Debug launch parameter is used to publish static marker positions.
    """
    debug = LaunchConfiguration('debug', default=False)
    robot_localization_file_path = os.path.join(
        get_package_share_directory('mep3_localization'), 'config', 'ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    return LaunchDescription([
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=[
                 '-0.1', '1.50976', '1.05','-4.85921e-06', '-0.965927', '0.258816', '1.32679e-06', 'map', 'camera_prediction'
             ],
             condition = IfCondition(debug)),
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=[
                 '-0.430', '0.925', '0', '0', '0', '0', '1', 'map', 'marker_[20]_static'
             ],
             condition = IfCondition(debug)),
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=[
                 '0.430', '0.925', '0', '0', '0', '0', '1', 'map', 'marker_[21]_static',
             ],
             condition = IfCondition(debug)),
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=[
                 '-0.430', '-0.925', '0', '0', '0', '0', '1', 'map', 'marker_[22]_static',
             ],
             condition = IfCondition(debug)),
        Node(package='tf2_ros',
             executable='static_transform_publisher',
             arguments=[
                 '0.430', '-0.925', '0', '0', '0', '0', '1', 'map', 'marker_[23]_static',
             ],
             condition = IfCondition(debug)),
        Node(package='mep3_localization', executable='aruco_detector',
             output='screen',  # debugging
             emulate_tty=True,  # debugging
             parameters=[
                 {'debug': debug}
             ]
             ),
        # Start robot localization using an Extended Kalman filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[robot_localization_file_path,
                        {'use_sim_time': use_sim_time}])

    ])
