from setuptools import setup
import os

package_name = 'mep3_simulation'

data = {
    'launch': [
        'robot_launch.py'
    ],
    'resource': [
        'ros2_control_configuration.yml',
        'webots_robot_description.urdf'
    ],
    'worlds': [
        'eurobot_2022.wbt',
        '.eurobot_2022.wbproj'
    ]
}


def files_in_directory(dir, extension=None):
    files = []
    for i in os.listdir(dir):
        if not os.path.isdir(f'{dir}/{i}') and (extension is None or i.endswith(extension)):
            files.append(f'{dir}/{i}')
    return files


data_files = []
data_files.extend([
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name,
     ['package.xml'])
])
data_files.append((
    'share/' + package_name + '/launch',
    [f'launch/{i}' for i in data['launch']]
))
data_files.append((
    'share/' + package_name + '/resource',
    [f'resource/{i}' for i in data['resource']]
))
data_files.append((
    'share/' + package_name + '/webots_data/worlds',
    [f'webots_data/worlds/{i}' for i in data['worlds']]
))
data_files.append((
    'share/' + package_name + '/webots_data/protos',
    files_in_directory('webots_data/protos', '.proto')
))
data_files.append((
    'share/' + package_name + '/webots_data/worlds/assets',
    files_in_directory('webots_data/worlds/assets')
))
data_files.append((
    'share/' + package_name + '/webots_data/worlds/assets/samples',
    files_in_directory('webots_data/worlds/assets/samples')
))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='memristor',
    maintainer_email='info@memristorrobotics.com',
    description='Memristor Eurobot Platform based on ROS 2 ',
    license='http://www.apache.org/licenses/LICENSE-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
