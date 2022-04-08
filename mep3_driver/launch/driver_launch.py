import os
import pathlib
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def generate_launch_description():
    package_dir = get_package_share_directory('mep3_driver')

    namespace = LaunchConfiguration('namespace', default='big')

    controller_params_file = LaunchConfiguration(
        'controller_params',
        default=os.path.join(get_package_share_directory('mep3_bringup'), 'resource', 'ros2_control_big.yaml')
    )

    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'config_big.urdf')).read_text()

    enable_can_interface()

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ],
        remappings=[
            ('/big/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('/odom', 'odom'),
            ('/tf', 'tf')
        ],
        namespace=namespace,
        output='screen'
    )

    socketcan_bridge = Node(
        package='mep3_driver',
        executable='socketcan_bridge',
        output='screen',
        namespace=namespace
    )

    cinch_driver = Node(
        package='mep3_driver',
        executable='cinch_driver.py',
        output='screen'
    )
        
    lidar = Node(
        package='hls_lfcd_lds_driver',
        executable='hlds_laser_publisher',
        name='hlds_laser_publisher',
        parameters=[{
            'port': '/dev/ttyAMA0',
            'frame_id': 'laser'
        }],
        output='screen',
        namespace=namespace
    )

    cinch_driver = Node(
        package='mep3_driver',
        executable='cinch_driver.py',
        output='screen'
    )

    return LaunchDescription([
        controller_manager_node,
        socketcan_bridge,
        cinch_driver,
        lidar
    ])
