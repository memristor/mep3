from setuptools import setup

package_name = 'mep3_simulation'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/ros2_control_configuration.yml',
    'resource/webots_robot_description.urdf',
]))
data_files.append(('share/' + package_name + '/webots_data/worlds', [
    'webots_data/worlds/eurobot_2022.wbt', 'webots_data/worlds/.eurobot_2022.wbproj',
]))
data_files.append(('share/' + package_name + '/webots_data/worlds/assets', [
    'webots_data/worlds/assets/table.png',
]))
data_files.append(('share/' + package_name + '/webots_data/worlds/assets/samples', [
    'webots_data/worlds/assets/samples/blue.png',
    'webots_data/worlds/assets/samples/bottom.png',
    'webots_data/worlds/assets/samples/empty.png',
    'webots_data/worlds/assets/samples/green.png',
    'webots_data/worlds/assets/samples/red.png',
]))
data_files.append(('share/' + package_name + '/webots_data/protos', [
    'webots_data/protos/Sample.proto',
    'webots_data/protos/DispenserVertical.proto'
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))

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
