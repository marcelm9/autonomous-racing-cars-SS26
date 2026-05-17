from launch import LaunchDescription
from launch_ros.actions import Node
import json


def generate_launch_description():
    # example
    track_layout = {
        "start": [0.0, 0.0, 0.0, 2.0],
        "checkpoints": [
            [1.0, 0.0, 1.0, 2.0],
            [2.0, 0.0, 2.0, 2.0],
            [3.0, 0.0, 3.0, 2.0],
        ],
    }

    return LaunchDescription(
        [
            Node(
                package="lap_timer_pkg",
                executable="lap_timer",
                name="lap_timer_node",
                output="screen",
                parameters=[{"track_layout": json.dumps(track_layout)}],
            )
        ]
    )
