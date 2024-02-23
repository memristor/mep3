"""Runs the simulation and robots."""

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription
from launch.actions.set_environment_variable import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_behavior_tree = LaunchConfiguration('bt', default=True)
    big_strategy = LaunchConfiguration('big_strategy', default='try_translate')
    small_strategy = LaunchConfiguration('small_strategy', default='wait_everytime')
    color = LaunchConfiguration('color', default='blue')
    namespace = LaunchConfiguration('namespace', default='big')
    use_opponents = LaunchConfiguration('opponents', default=False)
    debug = LaunchConfiguration('debug', default=False)
    use_localization = LaunchConfiguration('localization', default=False)

    set_color_action = SetEnvironmentVariable('MEP3_COLOR', color)
    set_use_opponents = SetEnvironmentVariable('MEP3_OPPONENTS', use_opponents)

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mep3_simulation'),
                         'launch', 'simulation_launch.py')),)

    big_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mep3_bringup'), 'launch',
                         'robot_launch.py')),
        launch_arguments=[
            ('sim', 'true'),
            ('namespace', 'big'),
            ('bt', use_behavior_tree),
            ('strategy', big_strategy),
            ('color', color),
        ],
    )

    small_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mep3_bringup'), 'launch',
                         'robot_launch.py')),
        launch_arguments=[
            ('sim', 'true'),
            ('namespace', 'small'),
            ('bt', use_behavior_tree),
            ('strategy', small_strategy),
            ('color', color),
        ],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mep3_localization'),
                         'launch', 'localization_launch.py')),
        launch_arguments=[
            ('debug', debug),
        ],
        condition=IfCondition(use_localization),
    )

    move = Node(
        package='mep3_navigation',
        executable='move',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                # 'angular.max_velocity': 0.5,
                # 'angular.max_acceleration': 0.3,
                'angular.tolerance': 0.001,
                'update_rate': 100,
            }
        ],
        namespace=namespace,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    ) 

    return launch.LaunchDescription([
        big_robot,
        small_robot,
        move,
        # The easiest way to get pass variables to Webots controllers
        # is to use environment variables.
        # localization,
        set_color_action,
        set_use_opponents,
        simulation,
    ])
