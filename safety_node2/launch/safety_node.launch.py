#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Path to simulator launch file
    gym_bridge_launch = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'launch',
        'gym_bridge_launch.py'
    )

    return LaunchDescription([

        # --- Start simulator ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gym_bridge_launch)
        ),

        # --- Start safety node ---
        Node(
            package='safety_node2',
            executable='safety_node',   # must match entry point in setup.py
            name='safety_node',
            output='screen',

            remappings=[
                ("/drive_out", "/drive"),
                ("/odom", "/ego_racecar/odom")
            ]
        )
    ])