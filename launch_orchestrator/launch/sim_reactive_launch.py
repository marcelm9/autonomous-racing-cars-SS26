import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # for crazy_map
    track_layout = {
        "start": [2.26, -3.9, 1.9, -2.2],
        "checkpoints": [
            [-5.5, -3.4, -5.76, -1.8],
            [-2.87, 1.48, -3.42, 3.44],
            [-7, -4.8, -8.4, -6],
        ],
    }

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("f1tenth_gym_ros"),
                        "launch",
                        "gym_bridge_launch.py",
                    )
                )
            ),
            Node(
                package="reactive",
                executable="reactive",
                name="reactive",
                output="screen",
                parameters=[
                    {
                        "scan_topic": "ego_racecar/scan",
                        "odom_topic": "ego_racecar/odom",
                        "drive_topic": "ego_racecar/drive",
                        "max_speed": 3.0,
                    }
                ],
            ),
            Node(
                package="lap_timer",
                executable="lap_timer",
                name="lap_timer_node",
                output="screen",
                parameters=[
                    {
                        "track_layout": json.dumps(track_layout),
                        "base_frame": "ego_racecar/odom",
                    }
                ],
            ),
        ]
    )
