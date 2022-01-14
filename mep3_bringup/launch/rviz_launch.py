import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_bringup')

    use_simulation = LaunchConfiguration('sim', default=False)
    namespace = LaunchConfiguration('namespace', default='big')

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
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')
        ],
        namespace=namespace
    )

    return launch.LaunchDescription([rviz])
