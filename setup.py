from glob import glob
import os

from setuptools import setup

package_name = 'bb_joy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JetSeaAI',
    maintainer_email='danielhuang2735@gmail.com',
    description='Xbox / PS5 joystick teleop for MAVROS USV.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joy_teleop_node = bb_joy.joy_teleop_node:main',
        ],
    },
)
