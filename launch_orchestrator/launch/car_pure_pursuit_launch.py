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

    base_pursuit_params = {
        "raceline_path": raceline_path,
        "base_frame": "particle_filter_base_frame",
        "drive_topic": "/drive",
        "odom_topic": "/odom",
        "min_lookahead": 0.25,
        "lookahead_gain": 0.3,
        "wheelbase": 0.33,
        "max_steering_angle": 0.42,
        "target_speed_scaling": 0.8,
        "min_target_speed": 1.0,
        "max_target_speed": 3.0,
        "curvature_speed_gain": 0.2,
    }

    aggressive = {**base_pursuit_params, "max_lookahead": 1.0}
    defensive = {**base_pursuit_params, "max_lookahead": 0.5}

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
                description="Path to the crazy_map raceline CSV.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("f1tenth_stack"),
                        "launch",
                        "bringup_launch.py",
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
                package="particle_filter",
                executable="particle_filter",
                name="particle_filter",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("particle_filter"),
                        "config",
                        "localize.yaml",
                    )
                ],
            ),
            Node(
                package="pure_pursuit",
                executable="pure_pursuit",
                name="pure_pursuit",
                output="screen",
                parameters=[aggressive],
            ),
            Node(
                package="lap_timer",
                executable="lap_timer",
                name="lap_timer_node",
                output="screen",
                parameters=[{"track_layout": json.dumps(track_layout)}],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    {
                        "yaml_filename": os.path.join(
                            get_package_share_directory("launch_orchestrator"),
                            "maps",
                            "crazy_map.yaml",
                        )
                    },
                    {"topic": "map"},
                    {"frame_id": "map"},
                    {"output": "screen"},
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": False},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            ),
        ]
    )
