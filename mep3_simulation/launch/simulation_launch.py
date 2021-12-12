import os
import pathlib

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_core.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_simulation')

    robot_description = pathlib.Path(os.path.join(
        package_dir, 'resource', 'webots_robot_description.urdf'
    )).read_text()

    ros2_control_params = os.path.join(
        package_dir, 'resource', 'ros2_control_configuration.yml'
    )

    webots = WebotsLauncher(world=os.path.join(
        package_dir, 'webots_data',
        'worlds', 'eurobot_2022.wbt'
    ))

    # The node which interacts with a robot in the Webots simulation is located
    # in the `webots_ros2_driver` package under name `driver`.
    # It is necessary to run such a node for each robot in the simulation.
    # Typically, we provide it the `robot_description` parameters from a URDF
    # file and `ros2_control_params` from the `ros2_control`
    # configuration file.
    webots_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        parameters=[
            {'robot_description': robot_description},
            ros2_control_params
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel')
        ]
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        # Start the Webots node
        webots,

        # Start the Webots robot driver
        webots_robot_driver,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
