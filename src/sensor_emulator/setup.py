from setuptools import setup

# This must match your Python package folder under src/
package_name = 'sensor_emulator'

setup(
    name=package_name,         # Distribution name
    version='0.0.1',
    packages=[package_name],    # Python package to include
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/sensor_emulator_launch.py',
          'launch/fusion.launch.py']),
        ('share/' + package_name + '/config',
         ['config/motion_profile.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Syed Ahmed Zulfiqar',
    maintainer_email='syedahmed.zulfiqar@gmail.com',
    description='A ROS 2 sensor emulator for IMU, GPS, and target cues.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_emulator_node = sensor_emulator.sensor_emulator_node:main',
            'test_input_pub        = sensor_emulator.test_input_publisher:main',
            'cue_fusion_node       = sensor_emulator.cue_fusion_node:main',
        ],
    },
)
