import os
import pathlib

from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_simulation')

    controller_params_file = LaunchConfiguration(
        'controller_params_big',
        default=os.path.join(
            get_package_share_directory('mep3_bringup'),
            'resource',
            'ros2_control_big.yaml'
        )
    )

    robot_description = pathlib.Path(os.path.join(
        package_dir, 'resource', 'config_big.urdf'
    )).read_text()

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
    webots_robot_driver_big = Node(
        package='webots_ros2_driver',
        executable='driver',
        parameters=[
            {'robot_description': robot_description},
            controller_params_file,
            # Override some values from the `controller_params_file`
            os.path.join(package_dir, 'resource', 'ros2_control_big.yaml')
        ],
        remappings=[
            ('/big/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('/odom', 'odom'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', 'scan'),
            ('/scan/point_cloud', 'scan/point_cloud'),
        ],
        namespace='big'
    )

    # This transform is slightly different from the one in the physical robot.
    # That's why it is defined here.
    tf_base_link_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'laser'],
        namespace='big',
        remappings=[
            ('/tf_static', 'tf_static')
        ],
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        # Start the Webots node
        webots,

        # Start the Webots robot driver
        webots_robot_driver_big,

        tf_base_link_laser,

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
