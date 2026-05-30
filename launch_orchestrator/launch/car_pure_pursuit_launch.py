import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    raceline_index = LaunchConfiguration("raceline_index")

    raceline_file = PythonExpression([
        "'", map_name, "' + ('_' + '", raceline_index, "' if '", raceline_index, "' else '')"
    ])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_name",
                description="Map name. Must be passed when launching.",
            ),

            DeclareLaunchArgument(
                "raceline_index",
                default_value="",
                description="Optional raceline index.",
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
                        "map_name": raceline_file,
                    }
                ],
            ),
            Node(
                package="pure_pursuit",
                executable="pure_pursuit",
                name="pure_pursuit",
                output="screen",
                parameters=[
                    {
                        # parameters for craziest_map_bridge
                        "base_frame": "particle_filter_base_frame",
                        "drive_topic": "/drive",
                        "odom_topic": "/odom",
                        "min_lookahead": 0.5,
                        "max_lookahead": 2.0,
                        "lookahead_gain": 1.0,
                        "wheelbase": 0.33,
                        "max_steering_angle": 0.42,
                        "min_speed": 1.5,
                        "max_speed": 2.5,
                        "do_linear_speed_scaling": True,
                        "curvature_speed_gain": -0.2,
                        "do_error_based_adaptive_speed_control": False,
                        "adaptive_control_slowing_factor": 1.0,
                        "adaptive_control_speed_up_factor": 1.0,
                        "mps_speed_gain_for_nonlinear_speed_output": 1.0,
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
