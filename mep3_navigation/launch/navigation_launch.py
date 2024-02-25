# Copyright 2021 Memristor Robotics
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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    mep3_navigation_dir = get_package_share_directory('mep3_navigation')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    nav2_bt_xml_file = LaunchConfiguration('nav2_bt_xml_file')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator']
    remappings = [('/tf', ['tf']),
                  ('/tf_static', ['tf_static'])]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    nav2_bt_xml_file_cmd = DeclareLaunchArgument(
        'nav2_bt_xml_file',
        default_value=os.path.join(
            mep3_navigation_dir,
            'behavior_trees',
            'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml',
        ),
        description='Full path to the YAML file used with navigation behavior'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='big',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Webots) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            mep3_navigation_dir, 'params', 'nav2_params_big.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # TODO: Switch to warn later
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='log level')

    load_composable_nodes = ComposableNodeContainer(
        namespace=namespace,
        remappings=remappings,
        parameters=[params_file],
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                namespace=namespace,
                parameters=[
                    params_file
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                namespace=namespace,
                parameters=[
                    params_file
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                namespace=namespace,
                parameters=[
                    params_file
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                namespace=namespace,
                parameters=[
                    params_file,
                    {
                        'default_nav_to_pose_bt_xml': nav2_bt_xml_file,
                        'default_nav_through_poses_bt_xml': nav2_bt_xml_file,
                    }
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                namespace=namespace,
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': True,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    move = Node(
        package='mep3_navigation',
        executable='move',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'angular.max_velocity': 1.5,
                'angular.max_acceleration': 1.3,
                'linear.max_velocity': 0.5,
                'linear.max_acceleration': 0.45,
                'angular.tolerance': 0.03,
                'linear.tolerance': 0.01,
                'update_rate': 100,
                'angular.stuck_coeff': 50.0,
                'linear.stuck_coeff': 6.0,
                'linear.kp': 14.0,
                'angular.kp': 15.0,
            }
        ],
        namespace=namespace,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    ) 

    return LaunchDescription([
        stdout_linebuf_envvar,
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        nav2_bt_xml_file_cmd,
        declare_log_level_cmd,
        load_composable_nodes,
        move
    ])
