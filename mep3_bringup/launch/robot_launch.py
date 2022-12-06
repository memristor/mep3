# Brings up a single robot.
#
# Example usage:
#
#   ros2 launch mep3_bringup robot_launch.py sim:=true color:=yellow namespace:=big strategy:=purple_strategy
#

from math import pi
import os
import sys

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import EmitEvent, \
    IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.actions.set_environment_variable import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution

INITIAL_POSE_MATRIX = [
    ('big', 'purple', [1.249, 0.102, pi/2]),
    ('small', 'purple', [1.2755, 0.443, pi]),
    ('big', 'yellow', [-1.249, 0.102, pi/2]),
    ('small', 'yellow', [-1.2755, 0.443, 0]),
]

PREDEFINED_TABLE_NAMES = [
    'table1',
    'table2'
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
                    '--x', str(row_pose[0]),
                    '--y', str(row_pose[1]),
                    '--yaw', str(row_pose[2]),
                    '--frame-id', 'map',
                    '--child-frame-id', 'odom'
                 ],
                 ros_arguments=['--log-level', 'warn'],
                 namespace=namespace,
                 remappings=[('/tf_static', 'tf_static')],
                 condition=and_condition([(color, row_color),
                                          (namespace, row_namespace)])))
    return transforms


def generate_launch_description():
    use_nav = LaunchConfiguration('nav', default=True)
    use_behavior_tree = LaunchConfiguration('bt', default=True)
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
    table = LaunchConfiguration('table', default='')

    set_colorized_output = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
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
    
    joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_position_controller',
            '--controller-manager-timeout',
            '50',
            '--controller-manager',
            ['/', namespace, '/controller_manager'],
        ],
        parameters=[{
            'use_sim_time': use_simulation
        }],
        condition=IfCondition(PythonExpression(["'", namespace, "' == 'big'"]))
    )

    behavior_tree = Node(
        package='mep3_behavior',
        executable='mep3_behavior',
        name=['behavior', namespace],
        output='screen',
        parameters=[{
            'use_sim_time': use_simulation,
            'color': color,
            'table': table,
            'strategy': strategy,
            'predefined_tables': PREDEFINED_TABLE_NAMES
        }],
        namespace=namespace,
        condition=launch.conditions.IfCondition(use_behavior_tree))

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mep3_navigation'),
                         'launch', 'navigation_launch.py')),
        launch_arguments=[
            ('use_sim_time', use_simulation),
            ('namespace', namespace),
            ('use_namespace', 'true'),
            ('params_file', [
                get_package_share_directory('mep3_navigation'),
                '/params',
                '/nav2_params_', namespace, '.yaml'
            ])
        ],
        condition=launch.conditions.IfCondition(use_nav))

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mep3_hardware'), 'launch',
                         'driver_launch.py')),
        launch_arguments=[('namespace', namespace)],
        condition=launch.conditions.UnlessCondition(use_simulation))

    tf_base_link_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--z', '0.3',
            '--yaw', str(-pi/2),
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ],
        ros_arguments=['--log-level', 'warn'],
        namespace=namespace,
        remappings=[('/tf_static', 'tf_static')],
    )

    laser_filters = Node(package='laser_filters',
                         executable='scan_to_scan_filter_chain',
                         parameters=[
                             PathJoinSubstitution([
                                 get_package_share_directory('mep3_navigation'),
                                 'params', 'laser_filters.yaml',
                             ])
                         ],
                         remappings=[('/tf_static', 'tf_static'),
                                     ('/tf', 'tf')
                                     ],
                         output='screen',
                         namespace=namespace)

    domain_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        output='screen',
        arguments=[
            os.path.join(
                get_package_share_directory('mep3_bringup'),
                'resource',
                'domain_bridge.yaml'
            )],
        condition=IfCondition(PythonExpression(["'", namespace, "' == 'big'"]))
    )

    # We want to avoid silent failures.
    # If any node fails, we want to crash the entire launch.
    on_exit_events = []
    critical_nodes = [
        behavior_tree,
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
        set_colorized_output,
        behavior_tree,
        domain_bridge_node,

        # Controllers
        diffdrive_controller_spawner,
        joint_controller_spawner,

        # Lidar filtering
        laser_filters,

        # Navigation 2
        nav2,
        tf_base_link_laser,
        driver,
    ] + on_exit_events)
