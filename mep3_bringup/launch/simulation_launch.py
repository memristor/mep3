"""Runs the simulation and robots."""

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_behavior_tree = LaunchConfiguration('bt', default=True)
    use_bt_strategy = LaunchConfiguration('strategy', default='first_strategy')
    use_opponent_controller = LaunchConfiguration('controller', default='opponent_robot')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mep3_simulation'),
            'launch',
            'simulation_launch.py'
        ))
    )

    big_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mep3_bringup'),
            'launch',
            'robot_launch.py'
        )),
        launch_arguments=[
            ('sim', 'true'),
            ('namespace', 'big'),
            ('bt', use_behavior_tree),
            ('strategy', use_bt_strategy),
            ('controller', use_opponent_controller),
        ],
    )

    return launch.LaunchDescription([
        simulation,
        big_robot
    ])
