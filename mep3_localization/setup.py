from setuptools import setup

package_name = 'mep3_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/launch',
                 ['launch/localization_launch.py'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bici',
    maintainer_email='vladimirvincan@gmail.com',
    description='Localization using OpenCV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_detection = mep3_localization.robot_detection:main',
            'static_broadcast = mep3_localization.static_broadcaster:main'
        ],
    },
)
