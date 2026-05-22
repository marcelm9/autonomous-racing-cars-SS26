import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    raceline_path = LaunchConfiguration("raceline_path")
    base_frame = LaunchConfiguration("base_frame")

    aggressive = {
        "raceline_path": raceline_path,
        "base_frame": base_frame,
        "drive_topic": "/drive",
        "odom_topic": "/ego_racecar/odom",
        "min_lookahead": 0.25,
        "max_lookahead": 1.0,
        "lookahead_gain": 0.4,
        "wheelbase": 0.33,
        "max_steering_angle": 0.42,
        "target_speed_scaling": 0.8,
        "min_target_speed": 1.0,
        "max_target_speed": 20.0,
        "curvature_speed_gain": 0.2,
    }

    defensive = {
        "raceline_path": raceline_path,
        "base_frame": base_frame,
        "drive_topic": "/drive",
        "odom_topic": "/ego_racecar/odom",
        "min_lookahead": 0.25,
        "max_lookahead": 0.5,
        "lookahead_gain": 0.4,
        "wheelbase": 0.33,
        "max_steering_angle": 0.42,
        "target_speed_scaling": 0.8,
        "min_target_speed": 1.0,
        "max_target_speed": 3.0,
        "curvature_speed_gain": 0.2,
    }

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
            DeclareLaunchArgument(
                "raceline_path",
                default_value="opt_raceline/opt_raceline_algo/outputs/crazy_map/traj_race_cl.csv",
                description="Path to the crazy_map raceline CSV.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="ego_racecar/base_link",
                description="Vehicle base frame in the simulator TF tree.",
            ),
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
                package="opt_raceline",
                executable="raceline_csv_publisher",
                name="raceline_csv_publisher",
                output="screen",
                parameters=[
                    {
                        "csv_path": raceline_path,
                    }
                ],
            ),
            Node(
                package="pure_pursuit",
                executable="raceline_pure_pursuit_node",
                name="raceline_pure_pursuit_node",
                output="screen",
                parameters=[defensive],
                remappings=[("/drive", "/drive_pp")],
            ),
            Node(
                package="reactive",
                executable="reactive",
                name="reactive",
                output="screen",
                remappings=[("/drive", "/drive_re")],
            ),
            Node(
                package="drive_combiner",
                executable="drive_combiner",
                name="drive_combiner",
                output="screen",
                remappings=[
                    ("/odom", "/ego_racecar/odom"),  # ! somehow this is needed
                    # (
                    #     "/scan",
                    #     "/ego_racecar/scan", # ! somehow this breaks it
                    # ),
                ],
            ),
            Node(
                package="lap_timer",
                executable="lap_timer",
                name="lap_timer_node",
                output="screen",
                parameters=[{"track_layout": json.dumps(track_layout)}],
            ),
        ]
    )
