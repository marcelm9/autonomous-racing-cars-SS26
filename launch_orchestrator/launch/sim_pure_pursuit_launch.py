import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_name = LaunchConfiguration("map_name")

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
                package="opt_raceline",
                executable="raceline_csv_publisher",
                name="raceline_csv_publisher",
                output="screen",
                parameters=[
                    {
                        "map_name": map_name,
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
                        "drive_topic": "ego_racecar/drive",
                        "odom_topic": "ego_racecar/odom",
                        "min_lookahead": 0.3,
                        "max_lookahead": 3.0,
                        "lookahead_gain": 0.2,
                        "wheelbase": 0.33,
                        "max_steering_angle": 0.42,
                        "target_speed_scaling": 1.0,
                        "min_speed": 1.5,
                        "max_speed": 6.0,
                        "do_linear_speed_scaling": False,
                        "curvature_speed_gain": -0.2,
                        "do_error_based_adaptive_speed_control": False,
                        "adaptive_control_slowing_factor": 1.0,
                        "adaptive_control_speed_up_factor": 1.0,
                        "mps_speed_gain_for_nonlinear_speed_output": 1.0,
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
        ]
    )
