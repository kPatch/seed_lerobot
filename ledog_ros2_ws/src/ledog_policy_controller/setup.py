from setuptools import find_packages, setup
import glob
import os

package_name = 'ledog_policy_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpatch',
    maintainer_email='irvsteve@gmail.com',
    description='ROS2 package for managing LeRobot policy execution',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy_controller_node = ledog_policy_controller.policy_controller_node:main'
        ],
    },
)
