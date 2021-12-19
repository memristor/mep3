from setuptools import setup

package_name = 'mep3_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
        ('share/' + package_name + '/resource',
         ['resource/map.pgm', 'resource/map.yml', 'resource/default.rviz']),
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
