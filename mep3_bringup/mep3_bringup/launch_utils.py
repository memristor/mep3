import yaml
from launch_ros.actions import Node


def get_controller_spawners(controller_params_file):
    with open(controller_params_file, 'r') as f:
        controller_params = yaml.safe_load(f)

    # Resolve namespace
    namespace = ''
    if 'controller_manager' not in controller_params.keys():
        namespace = list(controller_params.keys())[0]
        controller_params = controller_params[namespace]

    controller_names = list(controller_params['controller_manager']['ros__parameters'].keys())

    # Create controller spawners
    controller_spawners = []
    for controller_name in controller_names:
        if controller_name in ['update_rate', 'publish_rate']:
            continue

        controller_spawners.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            emulate_tty=True,
            arguments=[
                controller_name,
                '--controller-manager-timeout',
                '50',
                '--controller-manager',
                f'/{namespace}/controller_manager',
            ],
            namespace=namespace)
        )
    return controller_spawners
