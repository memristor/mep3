from setuptools import setup

package_name = 'mep3_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_launch.py',
            'launch/rviz_launch.py',
            'launch/simulation_launch.py',
            'launch/joystick_launch.py',
        ]),
        ('share/' + package_name + '/resource',
         [
            'resource/map.pgm',
            'resource/map.yml',
            'resource/default.rviz',
            'resource/ros2_control_big.yaml',
            'resource/ros2_control_small.yaml',
            'resource/joystick.yaml',
            'resource/domain_bridge.yaml',
         ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lukic',
    maintainer_email='lukicdarkoo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
