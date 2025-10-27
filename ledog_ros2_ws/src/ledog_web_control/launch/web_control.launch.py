#!/usr/bin/env python3
"""
Launch file for LeRobot Web Control Node.

This launch file starts the WebSocket bridge node that connects to a signaling server
as a producer for controlling the Go2 mobile platform and LeRobot arm via web interface.

Usage:
    ros2 launch ledog_web_control web_control.launch.py
    
    # With custom parameters:
    ros2 launch ledog_web_control web_control.launch.py \
        signalling_server:=wss://your-server.herokuapp.com/ws \
        room_id:=my_robot_room \
        max_linear_speed:=0.3
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for web control node."""
    
    # Get package directory
    pkg_share = FindPackageShare('ledog_web_control')
    
    # Default parameters file path
    default_params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'web_control_params.yaml'
    ])
    
    # Declare launch arguments
    signalling_server_arg = DeclareLaunchArgument(
        'signalling_server',
        default_value='wss://localhost:8000/ws',
        description='WebSocket URL for the signalling server'
    )
    
    room_id_arg = DeclareLaunchArgument(
        'room_id',
        default_value='default',
        description='Room ID to join on the signalling server'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Robot name (defaults to hostname if empty)'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command topic for Go2'
    )
    
    arm_joint_topic_arg = DeclareLaunchArgument(
        'arm_joint_topic',
        default_value='/arm/joint_commands',
        description='Joint command topic for LeRobot arm'
    )
    
    feedback_rate_arg = DeclareLaunchArgument(
        'feedback_rate',
        default_value='10.0',
        description='Feedback data rate in Hz'
    )
    
    heartbeat_frequency_arg = DeclareLaunchArgument(
        'heartbeat_frequency',
        default_value='3.0',
        description='Heartbeat frequency in seconds'
    )
    
    command_timeout_arg = DeclareLaunchArgument(
        'command_timeout',
        default_value='0.5',
        description='Command timeout in seconds'
    )
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.5',
        description='Maximum linear speed in m/s'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.0',
        description='Maximum angular speed in rad/s'
    )
    
    ticks_per_second_arg = DeclareLaunchArgument(
        'ticks_per_second',
        default_value='30',
        description='Control loop frequency in Hz'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to parameters file'
    )
    
    # Create the node
    web_control_node = Node(
        package='ledog_web_control',
        executable='web_control_node',
        name='web_control_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'signalling_server': LaunchConfiguration('signalling_server'),
                'room_id': LaunchConfiguration('room_id'),
                'robot_name': LaunchConfiguration('robot_name'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'arm_joint_topic': LaunchConfiguration('arm_joint_topic'),
                'feedback_rate': LaunchConfiguration('feedback_rate'),
                'heartbeat_frequency': LaunchConfiguration('heartbeat_frequency'),
                'command_timeout': LaunchConfiguration('command_timeout'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'ticks_per_second': LaunchConfiguration('ticks_per_second'),
            }
        ],
        emulate_tty=True
    )
    
    # Log info
    log_info = LogInfo(
        msg=[
            '\n',
            '=============================================================\n',
            ' LeRobot Web Control Node (Signaling Server Mode)\n',
            '=============================================================\n',
            ' Signalling Server: ', LaunchConfiguration('signalling_server'), '\n',
            ' Room ID: ', LaunchConfiguration('room_id'), '\n',
            ' Robot Name: ', LaunchConfiguration('robot_name'), '\n',
            ' Velocity Topic: ', LaunchConfiguration('cmd_vel_topic'), '\n',
            ' Arm Joint Topic: ', LaunchConfiguration('arm_joint_topic'), '\n',
            ' Max Linear Speed: ', LaunchConfiguration('max_linear_speed'), ' m/s\n',
            ' Max Angular Speed: ', LaunchConfiguration('max_angular_speed'), ' rad/s\n',
            ' Feedback Rate: ', LaunchConfiguration('feedback_rate'), ' Hz\n',
            '=============================================================\n'
        ]
    )
    
    return LaunchDescription([
        # Declare arguments
        signalling_server_arg,
        room_id_arg,
        robot_name_arg,
        cmd_vel_topic_arg,
        arm_joint_topic_arg,
        feedback_rate_arg,
        heartbeat_frequency_arg,
        command_timeout_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        ticks_per_second_arg,
        params_file_arg,
        
        # Log info
        log_info,
        
        # Start node
        web_control_node
    ])
