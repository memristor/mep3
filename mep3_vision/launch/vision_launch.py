from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace', default='big')
    color = LaunchConfiguration('color', default='blue')
    debug = LaunchConfiguration('debug', default='false')

    base_color = color.perform(context).split('_')[0]

    vision_node = Node(
        package='mep3_vision',
        namespace=namespace,
        output='screen',
        executable='aruco_action_server',
        name='aruco_vision',
        parameters=[{
            'color': base_color,
            'debug': debug
        }]
    )

    object_detection_node = Node(
        package='mep3_vision',
        namespace=namespace,
        output='screen',
        executable='object_detection',
        name='camera_vision',
        parameters=[]
    )

    return [
        vision_node,
        object_detection_node
    
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
