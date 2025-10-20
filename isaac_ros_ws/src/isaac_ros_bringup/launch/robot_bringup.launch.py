#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    yahboom_gazebo_launch_dir = os.path.join(
        get_package_share_directory('yahboom_rosmaster_gazebo'), 'launch')
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='cafe.world',
        description='World file name (e.g., empty.world, house.world)'
    )

    # Include the Gazebo simulation
    start_gazebo_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboom_gazebo_launch_dir, 'yahboom_rosmaster.gazebo.launch.py')
        ),
        launch_arguments={
            'world_file': LaunchConfiguration('world_file'),
            'enable_odom_tf': 'true',
            'headless': 'False',
            'use_rviz': 'true',
            'use_gazebo': 'true',
            'use_robot_state_pub': 'true',
            'load_controllers': 'true'
        }.items()
    )

    # Add the actions to the launch description
    ld.add_action(declare_world_cmd)
    ld.add_action(start_gazebo_simulation_cmd)

    return ld