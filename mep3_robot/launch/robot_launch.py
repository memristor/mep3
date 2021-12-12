import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_robot')

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
    use_simulation = LaunchConfiguration('simulation', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_behavior_tree = LaunchConfiguration('bt', default=False)

    nav2_map = os.path.join(package_dir, 'resource', 'map.yml')

    rviz_config = os.path.join(
        package_dir,
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

    rviz = Node(
        package='mep3_planning',
        executable='mep3_planning',
        output='screen',
        arguments=['ros_demo'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_behavior_tree)
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

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mep3_simulation'),
            'launch',
            'simulation_launch.py'
        )),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
        ],
        condition=launch.conditions.IfCondition(use_simulation)
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        simulation,

        # Wheel controller
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,

        # 3D Visualization
        rviz,

        # Start the robot_state_publisher
        robot_state_publisher,

        # Navigation 2
        nav2,
        map_odom_publisher
    ])
