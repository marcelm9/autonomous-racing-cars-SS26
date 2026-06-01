import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
                        "base_frame": "ego_racecar/base_link",
                        "drive_topic": "ego_racecar/drive_pp",
                        "odom_topic": "ego_racecar/odom",
                        "min_lookahead": 0.25,
                        "max_lookahead": 0.9,
                        "lookahead_gain": 0.75,
                        "wheelbase": 0.33,
                        "max_steering_angle": 0.42,
                        "target_speed_scaling": 0.8,
                        "min_target_speed": 1.0,
                        "max_target_speed": 15.0,
                        "curvature_speed_gain": 0.2,
                    }
                ],
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
                        "drive_topic": "ego_racecar/drive_re",
                        "max_speed": 10.0,
                    }
                ],
            ),
            Node(
                package="drive_combiner",
                executable="drive_combiner",
                name="drive_combiner",
                output="screen",
                parameters=[
                    {
                        "pp_input_topic": "ego_racecar/drive_pp",
                        "re_input_topic": "ego_racecar/drive_re",
                        "drive_output_topic": "ego_racecar/drive_combined",
                        "scan_topic": "ego_racecar/scan",
                        "odom_topic": "ego_racecar/odom",
                        "base_frame": "ego_racecar/base_link",
                        "laser_frame": "ego_racecar/laser",
                        "ttc_threshold": 0.5,
                        "ttc_cone_width_deg": 10.0,
                        "raceline_distance_threshold": 0.25,
                        "visual_path_length": 2.0,
                        "vehicle_width": 0.2,
                        "safety_margin": 0.0,
                    }
                ],
            ),
            Node(
                package="speed_limiter",
                executable="speed_limiter",
                name="speed_limiter",
                output="screen",
                parameters=[
                    {
                        "base_frame": "particle_filter_base_frame",
                        "laser_frame": "laser2",
                        "pp_input_topic": "ego_racecar/drive_combined",
                        "drive_output_topic": "drive",
                        "scan_topic": "ego_racecar/scan",
                        "odom_topic": "ego_racecar/odom",
                        "ttc_threshold": 1.0,
                        "ttc_cone_width_deg": 10.0,
                        "visual_path_length": 2.0,
                        "collision_distance_threshold": 0.0001,
                        "vehicle_width": 0.2,
                        "safety_margin": 0.0,
                    }
                ],
            ),
            Node(
                package="strategist",
                executable="strategist",
                name="strategist",
                output="screen",
                parameters=[
                    {
                        "map_name": map_name,
                        "base_frame": "ego_racecar/base_link",
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
                        "map_name": map_name,
                        "base_frame": "ego_racecar/base_link",
                    }
                ],
            ),
        ]
    )
