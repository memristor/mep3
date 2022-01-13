import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_bringup')

    use_rviz = LaunchConfiguration('rviz', default=False)
    use_simulation = LaunchConfiguration('sim', default=False)

    rviz_config = os.path.join(
        package_dir,
        'resource',
        'default.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_simulation}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return launch.LaunchDescription([rviz])
