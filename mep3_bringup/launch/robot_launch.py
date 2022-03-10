"""Brings up a single robot."""

from math import pi
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_bringup')

    use_nav = LaunchConfiguration('nav', default=True)
    use_behavior_tree = LaunchConfiguration('bt', default=True)
    use_regulator = LaunchConfiguration('regulator', default=True)

    use_simulation = LaunchConfiguration('sim', default=False)
    namespace = LaunchConfiguration('namespace', default='big')

    nav2_map = os.path.join(package_dir, 'resource', 'map.yml')

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        output='screen',
        arguments=[
            'diffdrive_controller',
            '--controller-manager-timeout',
            '50',
            '--controller-manager',
            ['/', namespace, '/controller_manager'],
        ],
        parameters=[{
            'use_sim_time': use_simulation
        }]
    )

    behavior_tree = Node(
        package='mep3_behavior_tree',
        executable='mep3_behavior_tree',
        output='screen',
        arguments=['first_strategy'],
        parameters=[{
            'use_sim_time': use_simulation
        }],
        namespace=namespace,
        condition=launch.conditions.IfCondition(use_behavior_tree))

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mep3_navigation'),
                         'launch', 'navigation_launch.py')),
        launch_arguments=[
            ('map', nav2_map),
            ('use_sim_time', use_simulation),
            ('namespace', namespace),
            ('use_namespace', 'true'),
        ],
        condition=launch.conditions.IfCondition(use_nav))

    regulator = Node(package='mep3_navigation',
                     executable='distance_angle_regulator',
                     output='screen',
                     parameters=[{
                         'use_sim_time': use_simulation
                     }],
                     namespace=namespace,
                     remappings=[('/tf_static', 'tf_static'),
                                 ('/tf', 'tf')],
                     condition=launch.conditions.IfCondition(use_regulator))

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mep3_driver'), 'launch',
                         'driver_launch.py')),
        launch_arguments=[('namespace', namespace)],
        condition=launch.conditions.UnlessCondition(use_simulation))

    # We are going to work with TFs until we create URDF.
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['1.21', '0.17', '0',
                   str(pi), '0', '0', 'map', 'odom'],
        namespace=namespace,
        remappings=[('/tf_static', 'tf_static')],
    )
    tf_base_link_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'laser'],
        namespace='big',
        remappings=[('/tf_static', 'tf_static')],
    )

    # We want to avoid silent failures.
    # If any node fails, we want to crash the entire launch.
    on_exit_events = []
    critical_nodes = [
        behavior_tree,
        regulator,
    ]
    for node in critical_nodes:
        on_exit_event = RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            ))
        on_exit_events.append(on_exit_event)

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        behavior_tree,

        # Wheel controller
        diffdrive_controller_spawner,

        # Navigation 2
        nav2,
        regulator,
        tf_map_odom,
        tf_base_link_laser,
        driver,
    ] + on_exit_events)
