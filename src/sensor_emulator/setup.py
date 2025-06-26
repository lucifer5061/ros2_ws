from setuptools import setup

package_name = 'sensor_emulator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/sensor_emulator_launch.py']),
        # Install config files
        ('share/' + package_name + '/config', ['config/motion_profile.yaml']),
        # Resource index for ament
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Syed Ahmed Zulfiqar',
    maintainer_email='syedahmed.zulfiqar@gmail.com',
    description='A ROS 2 node that emulates IMU, GPS, and target detection data.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_emulator_node = sensor_emulator.sensor_emulator_node:main',
        ],
    },
)

