import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    raceline_index = LaunchConfiguration("raceline_index")
    raceline_name = LaunchConfiguration("raceline_name")
    
    

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

            LogInfo(msg=["raceline_file: ", raceline_file]),

            Node(
                package="opt_raceline",
                executable="raceline_csv_publisher",
                name="raceline_csv_publisher",
                output="screen",
                parameters=[
                    {
                        "map_name": raceline_name,
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
                        "odom_topic": "ego_racecar/ego_racecar/odom",
                        "min_lookahead": 0.3,
                        "max_lookahead": 3.0,
                        "lookahead_gain": 0.2,
                        "wheelbase": 0.33,
                        "max_steering_angle": 0.42,
                        "target_speed_scaling": 1.0,
                        "min_speed": 1.0,
                        "max_speed": 4.0,
                        "do_linear_speed_scaling": True,
                        "curvature_speed_gain": -0.2,
                        "do_error_based_adaptive_speed_control": False,
                        "adaptive_control_slowing_factor": 1.0,
                        "adaptive_control_speed_up_factor": 1.0,
                        "mps_speed_gain_for_nonlinear_speed_output": 1.0,
                        "delay_correction": 1,
                        # "localization_search_backward_m": 100.0,
                        # "localization_search_forward_m": 100.0,
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
