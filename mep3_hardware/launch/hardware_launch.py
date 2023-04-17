import os
import pathlib
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from mep3_bringup.launch_utils import get_controller_spawners


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

    usb_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_9e82eec869f4c84fb1901fc50d00c93c-if00-port0' if performed_namespace == 'big' else 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_0e10a6d7001d8f4fad5376bfd2f6f1ad-if00-port0'

    controller_params_file = os.path.join(package_dir, 'resource', f'{performed_namespace}_controllers.yaml')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', f'{performed_namespace}_description.urdf')).read_text()

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
            (f'/{performed_namespace}/diffdrive_controller/odom', 'odom'),
            ('/tf', 'tf')
        ],
        namespace=namespace,
        ros_arguments=['--log-level', 'warn'],
        output='screen'
    )

    socketcan_bridge = Node(
        package='mep3_hardware',
        executable='socketcan_bridge',
        output='screen',
        ros_arguments=['--log-level', 'warn'],
        namespace=namespace
    )

    cinch_driver = Node(
        package='mep3_hardware',
        executable='cinch_driver.py',
        ros_arguments=['--log-level', 'warn'],
        output='screen'
    )

    lidar_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_ros',
        parameters=[{
            'frame_id': 'laser',
            'serial_port': usb_port
        }],
        ros_arguments=['--log-level', 'warn'],
        output='screen',
        namespace=namespace,
    )

    lcd_driver = Node(
        package='mep3_hardware',
        executable='lcd_driver.py',
        output='screen',
        ros_arguments=['--log-level', 'warn'],
        condition=IfCondition(PythonExpression(["'", namespace, "' == 'big'"]))
    )

    return [
        controller_manager_node,
        socketcan_bridge,
        cinch_driver,
        lidar_rplidar,
        lcd_driver,
    ] + get_controller_spawners(controller_params_file)


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
