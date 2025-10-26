from setuptools import find_packages, setup

package_name = 'lekiwi_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/*']),
        ('share/' + package_name + '/config', ['config/*']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LeKiwi Developer',
    maintainer_email='dev@lekiwi.local',
    description='Launch files and configuration for LeKiwi mobile base',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
