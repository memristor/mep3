"""Runs the simulation and robots."""

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription
from launch.actions.set_environment_variable import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_behavior_tree = LaunchConfiguration('bt', default=True)
    big_strategy = LaunchConfiguration('big_strategy', default='purple_strategy')
    small_strategy = LaunchConfiguration('small_strategy', default='purple_strategy')
    color = LaunchConfiguration('color', default='purple')
    use_opponents = LaunchConfiguration('opponents', default=False)
    namespace = LaunchConfiguration('namespace', default='big')

    set_color_action = SetEnvironmentVariable('MEP3_COLOR', color)
    set_use_opponents = SetEnvironmentVariable('MEP3_OPPONENTS', use_opponents)

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mep3_simulation'),
            'launch',
            'simulation_launch.py'
        )),
    )

    big_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mep3_bringup'),
            'launch',
            'robot_launch.py'
        )),
        launch_arguments=[
            ('sim', 'true'),
            ('namespace', namespace),
            ('bt', use_behavior_tree),
            ('strategy', big_strategy),
            ('color', color),
        ],
    )

    small_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mep3_bringup'),
            'launch',
            'robot_launch.py'
        )),
        launch_arguments=[
            ('sim', 'true'),
            ('namespace', 'small'),
            ('bt', use_behavior_tree),
            ('strategy', small_strategy),
            ('color', color),
        ],
    )

    return launch.LaunchDescription([
        big_robot,
        small_robot,

        # The easiest way to get pass variables to Webots controllers
        # is to use environment variables.
        set_color_action,
        set_use_opponents,
        simulation,
    ])
