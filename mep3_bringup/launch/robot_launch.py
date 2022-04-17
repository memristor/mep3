"""Brings up a single robot."""

from math import pi
import os
import sys

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import EmitEvent, \
    IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

INITIAL_POSE_MATRIX = [
    ('big', 'purple', [1.2491, 0.1, -pi / 2]),
    ('big', 'yellow', [-1.2491, 0.1, -pi / 2]),
    ('small', 'purple', [-1.21, 0.17, 0.0]),
    ('small', 'yellow', [-1.21, 0.17, 0.0]),
]


def verify_color(context, *args, **kwargs):
    if LaunchConfiguration('color').perform(context) \
            not in ['purple', 'yellow']:
        print(
            'ERROR: The `color` parameter must be either `purple` or `yellow`.'
        )
        sys.exit(1)


def verify_namespace(context, *args, **kwargs):
    if LaunchConfiguration('namespace').perform(context) \
            not in ['big', 'small']:
        print(
            'ERROR: The `namespace` parameter must be either `big` or `small`.'
        )
        sys.exit(1)


def and_condition(pairs):
    condition = []
    for pair in pairs:
        if len(condition) > 0:
            condition.append(' and ')
        condition += ['"', pair[0], '" == "', pair[1], '"']
    return launch.conditions.IfCondition(PythonExpression(condition))


def get_initial_pose_transform(namespace, color):
    transforms = []
    for row in INITIAL_POSE_MATRIX:
        row_namespace = row[0]
        row_color = row[1]
        row_pose = row[2]

        transforms.append(
            Node(package='tf2_ros',
                 executable='static_transform_publisher',
                 output='screen',
                 arguments=[
                     str(row_pose[0]),
                     str(row_pose[1]), '0',
                     str(row_pose[2]), '0', '0', 'map', 'odom'
                 ],
                 namespace=namespace,
                 remappings=[('/tf_static', 'tf_static')],
                 condition=and_condition([(color, row_color),
                                          (namespace, row_namespace)])))
    return transforms


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_bringup')

    use_nav = LaunchConfiguration('nav', default=True)
    use_behavior_tree = LaunchConfiguration('bt', default=True)
    use_regulator = LaunchConfiguration('regulator', default=True)
    use_simulation = LaunchConfiguration('sim', default=False)

    # Implementation wise, it would probably be easier to use
    # environment variables (for namespace and color).
    # However, we use parameters for consistency.
    namespace = LaunchConfiguration('namespace',
                                    default=os.environ['MEP3_NAMESPACE'] if
                                    'MEP3_NAMESPACE' in os.environ else None)
    strategy = LaunchConfiguration('strategy',
                                   default=os.environ['MEP3_STRATEGY']
                                   if 'MEP3_STRATEGY' in os.environ else None)
    color = LaunchConfiguration('color')

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
        }])

    behavior_tree = Node(
        package='mep3_behavior_tree',
        executable='mep3_behavior_tree',
        output='screen',
        arguments=[strategy],
        parameters=[{
            'use_sim_time': use_simulation,
            'color': color
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
                     remappings=[('/tf_static', 'tf_static'), ('/tf', 'tf')],
                     condition=launch.conditions.IfCondition(use_regulator))

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mep3_driver'), 'launch',
                         'driver_launch.py')),
        launch_arguments=[('namespace', namespace)],
        condition=launch.conditions.UnlessCondition(use_simulation))

    tf_base_link_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0', '0', '0.3',
            str(-pi / 2), '0', '0', 'base_link', 'laser'
        ],
        namespace=namespace,
        remappings=[('/tf_static', 'tf_static')],
    )

    laser_inflator = Node(package='mep3_navigation',
                          executable='laser_inflator',
                          parameters=[{
                              'inflation_radius': 0.05,
                              'inflation_angular_step': 0.09
                          }],
                          remappings=[('/tf_static', 'tf_static'),
                                      ('/tf', 'tf')],
                          output='screen',
                          namespace=namespace)

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
        OpaqueFunction(function=verify_color),
        OpaqueFunction(function=verify_namespace),
        behavior_tree,

        # Wheel controller
        diffdrive_controller_spawner,

        # Lidar inflation
        laser_inflator,

        # Navigation 2
        nav2,
        regulator,
        tf_base_link_laser,
        driver,
    ] + on_exit_events + get_initial_pose_transform(namespace, color))
