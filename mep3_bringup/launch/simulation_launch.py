"""Runs the simulation and robots."""

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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
            ('namespace', 'big')
        ],
    )

    return launch.LaunchDescription([
        simulation,
        big_robot
    ])
