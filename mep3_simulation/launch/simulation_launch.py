import os
import pathlib

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_simulation')

    controller_params_file_big = LaunchConfiguration(
        'controller_params_big',
        default=os.path.join(get_package_share_directory('mep3_bringup'),
                             'resource', 'ros2_control_big.yaml'))
    controller_params_file_small = LaunchConfiguration(
        'controller_params_small',
        default=os.path.join(get_package_share_directory('mep3_bringup'),
                             'resource', 'ros2_control_small.yaml'))

    robot_description_big = pathlib.Path(
        os.path.join(package_dir, 'resource', 'config_big.urdf')).read_text()
    robot_description_small = pathlib.Path(
        os.path.join(package_dir, 'resource',
                     'config_small.urdf')).read_text()
    camera_description = pathlib.Path(
        os.path.join(package_dir, 'resource', 'config_camera.urdf')).read_text()

    webots = WebotsLauncher(world=os.path.join(package_dir, 'webots_data',
                                               'worlds', 'eurobot_2023.wbt'))

    ros2_supervisor = Ros2SupervisorLauncher()

    # The node which interacts with a robot in the Webots simulation is located
    # in the `webots_ros2_driver` package under name `driver`.
    # It is necessary to run such a node for each robot in the simulation.
    # Typically, we provide it the `robot_description` parameters from a URDF
    # file and `ros2_control_params` from the `ros2_control`
    # configuration file.
    webots_robot_driver_big = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',  # debugging
        emulate_tty=True,  # debugging
        parameters=[
            {
                'robot_description': robot_description_big
            },
            controller_params_file_big,
            # Override some values from the `controller_params_file`
            os.path.join(package_dir, 'resource', 'ros2_control_big.yaml')
        ],
        remappings=[
            ('/big/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('/big/diffdrive_controller/odom', 'odom'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', 'scan'),
            ('/scan/point_cloud', 'scan/point_cloud'),
        ],
        ros_arguments=['--log-level', 'warn'],
        additional_env={'WEBOTS_CONTROLLER_URL': 'robot_big'},
        namespace='big')

    webots_robot_driver_small = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',  # debugging
        emulate_tty=True,  # debugging
        parameters=[
            {
                'robot_description': robot_description_small
            },
            controller_params_file_small,
            # Override some values from the `controller_params_file`
            os.path.join(package_dir, 'resource', 'ros2_control_small.yaml')
        ],
        remappings=[
            ('/small/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('/small/diffdrive_controller/odom', 'odom'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', 'scan'),
            ('/scan/point_cloud', 'scan/point_cloud'),
        ],
        ros_arguments=['--log-level', 'warn'],
        additional_env={'WEBOTS_CONTROLLER_URL': 'robot_small'},
        namespace='small')

    # Camera driver for the Central Tracking Device
    webots_camera_driver_central = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',  # debugging
        emulate_tty=True,  # debugging
        parameters=[{
            'robot_description': camera_description
        }, {
            'use_sim_time': True
        }],
        ros_arguments=['--log-level', 'warn'],
        additional_env={'WEBOTS_CONTROLLER_URL': 'camera_central'},
        namespace='camera',
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        # Start the Webots node
        webots,
        ros2_supervisor,

        # Start the Webots robot drivers
        webots_robot_driver_big,
        webots_robot_driver_small,
        webots_camera_driver_central,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.
        RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        ))
    ])
