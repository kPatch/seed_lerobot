import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ledog_web_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LeRobot Developer',
    maintainer_email='dev@lerobot.local',
    description='WebSocket bridge for controlling Go2 robot and LeRobot arm via web interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_control_node = ledog_web_control.web_control_node:main',
        ],
    },
)
