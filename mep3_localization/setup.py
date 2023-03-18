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
                 ['launch/localization_launch.py']),
                ('share/' + package_name + '/config',
                 ['config/ekf.yaml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vladimir Vincan',
    maintainer_email='vladimirvincan@gmail.com',
    description='Memristor Localization Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = mep3_localization.aruco_detector:main',
        ],
    },
)
