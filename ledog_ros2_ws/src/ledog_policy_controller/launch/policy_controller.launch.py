#!/usr/bin/env python3
"""
Launch file for LeRobot Policy Controller Node

This launch file starts the policy controller node with configurable parameters.

Usage:
    # Basic launch
    ros2 launch ledog_policy_controller policy_controller.launch.py
    
    # With custom conda environment
    ros2 launch ledog_policy_controller policy_controller.launch.py \
        conda_env_name:=my_lerobot_env \
        conda_base_path:=~/miniconda3

Parameters:
    conda_env_name (str): Name of the conda environment (default: 'lerobot')
    conda_base_path (str): Path to conda base installation (default: '~/anaconda3')
    shutdown_timeout (float): Seconds to wait for graceful shutdown (default: 5.0)
    robot_id (str): Robot ID for LeRobot (default: 'follower_1')
    robot_port (str): Serial port for robot (default: '/dev/ttyACM0')
    robot_type (str): Robot type (default: 'so100_follower')

Author: kpatch
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for policy controller node."""
    
    # Declare launch arguments
    conda_env_arg = DeclareLaunchArgument(
        'conda_env_name',
        default_value='lerobot',
        description='Name of the conda environment containing LeRobot'
    )
    
    conda_base_arg = DeclareLaunchArgument(
        'conda_base_path',
        default_value='~/anaconda3',
        description='Path to conda base installation (e.g., ~/anaconda3 or ~/miniconda3)'
    )
    
    shutdown_timeout_arg = DeclareLaunchArgument(
        'shutdown_timeout',
        default_value='5.0',
        description='Seconds to wait for graceful process shutdown before force kill'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='follower_1',
        description='Robot ID for LeRobot recording'
    )
    
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='/dev/ttyACM0',
        description='Serial port for robot communication'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='so100_follower',
        description='Type of robot for LeRobot'
    )
    
    # Create the policy controller node
    policy_controller_node = Node(
        package='ledog_policy_controller',
        executable='policy_controller_node',
        name='policy_controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'conda_env_name': LaunchConfiguration('conda_env_name'),
            'conda_base_path': LaunchConfiguration('conda_base_path'),
            'shutdown_timeout': LaunchConfiguration('shutdown_timeout'),
            'robot_id': LaunchConfiguration('robot_id'),
            'robot_port': LaunchConfiguration('robot_port'),
            'robot_type': LaunchConfiguration('robot_type'),
        }]
    )
    
    return LaunchDescription([
        conda_env_arg,
        conda_base_arg,
        shutdown_timeout_arg,
        robot_id_arg,
        robot_port_arg,
        robot_type_arg,
        policy_controller_node,
    ])

