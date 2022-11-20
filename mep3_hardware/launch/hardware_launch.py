import os
import pathlib
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def enable_can_interface():
    is_can_up = False
    if os.path.isfile('/sys/class/net/can0/operstate'):
        with open('/sys/class/net/can0/operstate') as f:
            if f.read().strip() == 'up':
                is_can_up = True
    if not is_can_up:
        result = subprocess.run('sudo ip link set can0 up type can bitrate 500000 restart-ms 100 sjw 3', shell=True)
        if result.returncode:
            exit(result.returncode)
        result = subprocess.run('sudo ip link set can0 txqueuelen 100', shell=True)
        if result.returncode:
            exit(result.returncode)


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('mep3_hardware')

    namespace = LaunchConfiguration('namespace', default='big')
    performed_namespace = namespace.perform(context)

    controller_params_file = os.path.join(get_package_share_directory(
        'mep3_bringup'), 'resource', f'ros2_control_{performed_namespace}.yaml')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', f'config_{performed_namespace}.urdf')).read_text()
    dynamixel_config = os.path.join(package_dir, 'resource', f'dynamixel_config_{performed_namespace}.yaml')

    enable_can_interface()

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ],
        remappings=[
            (f'/{performed_namespace}/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('/odom', 'odom'),
            ('/tf', 'tf')
        ],
        namespace=namespace,
        output='screen'
    )

    socketcan_bridge = Node(
        package='mep3_hardware',
        executable='socketcan_bridge',
        output='screen',
        namespace=namespace
    )

    cinch_driver = Node(
        package='mep3_hardware',
        executable='cinch_driver.py',
        output='screen'
    )

    pumps_driver = Node(
        package='mep3_hardware',
        executable='vacuum_pump_driver.py',
        output='screen',
        namespace=namespace
    )

    lidar_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_ros',
        parameters=[{
            'frame_id': 'laser',
            'serial_port': '/dev/rplidar'
        }],
        output='screen',
        namespace=namespace,
    )

    dynamixel_driver = Node(
        package='mep3_hardware',
        executable='dynamixel_driver',
        parameters=[dynamixel_config],
        output='screen',
        namespace=namespace
    )

    lcd_driver = Node(
        package='mep3_hardware',
        executable='lcd_driver.py',
        output='screen',
        condition=IfCondition(PythonExpression(["'", namespace, "' == 'small'"]))
    )

    return [
        controller_manager_node,
        socketcan_bridge,
        cinch_driver,
        lidar_rplidar,
        pumps_driver,
        dynamixel_driver,
        lcd_driver
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
