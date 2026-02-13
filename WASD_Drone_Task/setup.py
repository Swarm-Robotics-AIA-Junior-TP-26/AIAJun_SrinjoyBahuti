from setuptools import setup
import os
from glob import glob

package_name = 'px4_keyboard_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[

        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Keyboard teleoperation for PX4 drones using MAVROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'keyboard_input_node = px4_keyboard_teleop.keyboard_input:main',
            'velocity_controller_node = px4_keyboard_teleop.velocity_controller:main',
        ],
    },
)
