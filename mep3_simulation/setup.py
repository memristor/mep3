from glob import glob
import os

from setuptools import setup


package_name = 'mep3_simulation'

data = {
    'launch': [
        'simulation_launch.py'
    ],
    'resource': [
        'config_big.urdf',
        'ros2_control_big.yaml'
    ]
}


def files_in_directory(path, extension=None):
    files = []
    for i in os.listdir(path):
        if not os.path.isdir(f'{path}/{i}') and \
                (extension is None or i.endswith(extension)):
            files.append(f'{path}/{i}')
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
for path in glob('webots_data/**/*', recursive=True):
    if os.path.isdir(path):
        files_from_path = [f for f in glob(path + '/*') if os.path.isfile(f)]
        data_files.append(
            ('share/' + package_name + '/' + path, files_from_path)
        )

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='memristor',
    maintainer_email='info@memristorrobotics.com',
    description='Memristor Eurobot Platform based on ROS 2',
    license='http://www.apache.org/licenses/LICENSE-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
