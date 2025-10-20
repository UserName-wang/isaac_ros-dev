#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('isaac_ros_bringup')
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    declare_joy_dev_cmd = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device file'
    )
    
    declare_joy_config_cmd = DeclareLaunchArgument(
        'joy_config',
        default_value='xbox',
        description='Joystick configuration'
    )

    # Joy node for gamepad input
    start_joy_node_cmd = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.01,
            'autorepeat_rate': 20.0
        }],
        remappings=[
            ('/joy', '/joy')
        ]
    )

    # Teleop twist joy node for converting joystick inputs to robot commands
    start_teleop_twist_joy_node_cmd = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[os.path.join(pkg_share, 'config', 'joystick.yaml')],
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
        ]
    )

    # Add the actions to the launch description
    ld.add_action(declare_joy_dev_cmd)
    ld.add_action(declare_joy_config_cmd)
    ld.add_action(start_joy_node_cmd)
    ld.add_action(start_teleop_twist_joy_node_cmd)

    return ld