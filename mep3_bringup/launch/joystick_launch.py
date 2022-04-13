import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    cmd_vel = launch.substitutions.LaunchConfiguration('cmd_vel', default='/big/cmd_vel')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('mep3_bringup'),
                    'resource',
                    'joystick.yaml'
                ),
            ],
            remappings={('/cmd_vel', cmd_vel)},
        ),
    ])
