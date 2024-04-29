# Brings up a single robot.
#
# Example usage:
#
#   ros2 launch mep3_bringup robot_launch.py sim:=true color:= namespace:=big strategy:=blue_strategy
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


INITIAL_POSE_MATRIX = [
    # ('big', 'blue', [0.03, -1.15, pi/2]), # centralno polje
    ('big', 'blue', [0.815, -1.33, pi/2]), # polje kod panela
    ('big', 'yellow', [0.80, 1.33, -pi/2]), # polje kod panela
    
    ('big', 'blue_a', [0.045, 1.31, -pi/2]), # centralno polje
    ('big', 'yellow_a', [0.045, -1.31, pi/2]), # centralno polje

    ('big', 'blue_b', [0.81, -1.34, -pi/2]), # unazad kod panela
    ('big', 'yellow_b', [0.045, -1.31, pi/2]), # unazad kod panela

    ('small', 'blue', [0.72, -1.16, pi/2])
]
PREDEFINED_TABLE_NAMES = [
    'table1',
    'table2'
]


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


def launch_setup(context, *args, **kwargs):
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
    should_live_reload = ('live' in strategy.perform(context))
    if color.perform(context) not in ['blue', 'yellow', 'blue_a', 'yellow_a', 'blue_b', 'yellow_b']:
        print('ERROR: The `color` parameter must be either `blue` or `yellow`.')
        sys.exit(1)
    if namespace.perform(context) not in ['big', 'small']:
        print('ERROR: The `namespace` parameter must be either `big` or `small`.')
        sys.exit(1)

    set_colorized_output = SetEnvironmentVariable(
        'RCUTILS_COLORIZED_OUTPUT', '1')

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
        condition=launch.conditions.IfCondition(use_behavior_tree),
        respawn=should_live_reload,
    )

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
                         'hardware_launch.py')),
        launch_arguments=[
            ('namespace', namespace),
        ],
        condition=launch.conditions.UnlessCondition(use_simulation),
    )

    tf_base_link_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--z', '0.3',
            # '--yaw', str(pi),
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ],
        ros_arguments=['--log-level', 'warn'],
        namespace=namespace,
        remappings=[('/tf_static', 'tf_static')],
    )

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
    critical_nodes = []
    if not should_live_reload:
        critical_nodes.append(behavior_tree)
    for node in critical_nodes:
        on_exit_event = RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            ))
        on_exit_events.append(on_exit_event)

    # Standard ROS 2 launch description
    return [
        set_colorized_output,
        behavior_tree,
        domain_bridge_node,
        nav2,
        tf_base_link_laser,
        driver,
    ] + on_exit_events + get_initial_pose_transform(namespace, color)


def generate_launch_description():
    return launch.LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
