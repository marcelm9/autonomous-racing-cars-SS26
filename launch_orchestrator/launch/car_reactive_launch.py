import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    If you want to run this file without a map, just put in any map name (eg. `map_name:=abc`).
    Then, of course, some things will not work:
        - displaying the map in rviz2
        - showing the position of the car in rviz2
        - timing of the laps
    These nodes will fail, but the reactive node can of course still run unaffected.
    """
    map_name = LaunchConfiguration("map_name")

    return LaunchDescription(
        [
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
                package="reactive",
                executable="reactive",
                name="reactive",
                output="screen",
                parameters=[
                    {
                        "scan_topic": "/scan",
                        "odom_topic": "/odom",
                        "drive_topic": "/drive",
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
                    ),
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    {
                        "yaml_filename": PathJoinSubstitution(
                            [
                                FindPackageShare("launch_orchestrator"),
                                "maps",
                                [map_name, ".yaml"],
                            ]
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
            Node(
                package="lap_timer",
                executable="lap_timer",
                name="lap_timer_node",
                output="screen",
                parameters=[
                    {
                        "map_name": map_name,
                        "base_frame": "particle_filter_base_frame",
                    }
                ],
            ),
        ]
    )
