from setuptools import setup

package_name = 'sensor_emulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Syed',
    maintainer_email='syedahmed.zulfiqar@gmail.com',
    description='ROS 2 Sensor Emulator Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_emulator_node = sensor_emulator.sensor_emulator_node:main',
        ],
    },
)

