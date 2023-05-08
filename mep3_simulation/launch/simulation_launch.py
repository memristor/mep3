import os
import pathlib

from ament_index_python.packages import get_package_share_directory
import launch
from mep3_bringup.launch_utils import get_controller_spawners
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    # HOTFIX: https://github.com/cyberbotics/webots_ros2/issues/567
    os.environ['LD_LIBRARY_PATH'] += ':/opt/ros/humble/lib/controller'

    package_dir = get_package_share_directory('mep3_simulation')

    controller_params_file_big = os.path.join(get_package_share_directory('mep3_hardware'),
                             'resource', 'big_controllers.yaml')
    controller_params_file_small = os.path.join(get_package_share_directory('mep3_hardware'),
                             'resource', 'small_controllers.yaml')
    controller_params_file_big_override = os.path.join(get_package_share_directory('mep3_simulation'),
                                'resource', 'big_controllers.yaml')
    controller_params_file_small_override = os.path.join(get_package_share_directory('mep3_simulation'),
                             'resource', 'small_controllers.yaml')

    robot_description_big = pathlib.Path(
        os.path.join(package_dir, 'resource', 'big_description.urdf')).read_text()
    robot_description_small = pathlib.Path(
        os.path.join(package_dir, 'resource',
                     'small_description.urdf')).read_text()
    camera_description = pathlib.Path(
        os.path.join(package_dir, 'resource', 'camera_description.urdf')).read_text()

    webots = WebotsLauncher(world=os.path.join(package_dir, 'webots_data',
                                               'worlds', 'eurobot.wbt'), ros2_supervisor=True)

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
                'use_sim_time': True,
                'robot_description': robot_description_big
            },
            controller_params_file_big,
            controller_params_file_big_override
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
                'use_sim_time': True,
                'robot_description': robot_description_small
            },
            controller_params_file_small,
            controller_params_file_small_override
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
            'robot_description': camera_description,
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
        webots._supervisor,

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
    ] + get_controller_spawners(controller_params_file_big) + get_controller_spawners(controller_params_file_small))
