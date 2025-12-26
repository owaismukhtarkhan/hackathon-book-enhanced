#!/usr/bin/env python3
# gazebo_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('my_robot_sim')

    # Define launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'robotics_lab.sdf'),
        description='SDF world file'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch GUI'
    )

    # Gazebo server node
    gazebo_server = Node(
        package='gazebo_ros',
        executable='gzserver',
        arguments=[LaunchConfiguration('world')],
        output='screen'
    )

    # Gazebo client node (GUI)
    gazebo_client = Node(
        package='gazebo_ros',
        executable='gzclient',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
        output='screen'
    )

    # Return launch description
    return LaunchDescription([
        world_arg,
        gui_arg,
        gazebo_server,
        gazebo_client
    ])