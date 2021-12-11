import os
import pathlib

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


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

    controller_manager_timeout = ['--controller-manager-timeout', '50']

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        arguments=['diffdrive_controller', ] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

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

    # Often we want to publish robot transforms, so we use the
    # `robot_state_publisher` node for that.
    # If robot model is not specified in the URDF file then Webots can help
    # us with the URDF exportation feature.
    # Since the exportation feature is available only once the simulation has
    # started and the `robot_state_publisher` node requires a
    # `robot_description` parameter before we have to specify a dummy robot.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    use_rviz = LaunchConfiguration('rviz', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    nav2_map = os.path.join(package_dir, 'resource', 'map.yml')

    rviz_config = os.path.join(
        get_package_share_directory('mep3_simulation'),
        'resource',
        'default.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'bringup_launch.py'
        )),
        launch_arguments=[
            ('map', nav2_map),
            ('use_sim_time', use_sim_time),
        ],
        condition=launch.conditions.IfCondition(use_nav)
    )

    map_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([

        # Wheel controller
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,

        # Start the Webots node
        webots,

        # 3D Visualization
        rviz,

        # Start the Webots robot driver
        webots_robot_driver,

        # Start the robot_state_publisher
        robot_state_publisher,

        # Navigation 2
        nav2,
        map_odom_publisher,

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
