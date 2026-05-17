from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    raceline_path = LaunchConfiguration("raceline_path")
    base_frame = LaunchConfiguration("base_frame")
    drive_topic = LaunchConfiguration("drive_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "raceline_path",
                default_value="opt_raceline/opt_raceline_algo/outputs/crazy_map/traj_race_cl.csv",
                description="Path to the semicolon-separated raceline CSV.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="ego_racecar/base_link",
                description="Vehicle base frame used for Pure Pursuit steering.",
            ),
            DeclareLaunchArgument(
                "drive_topic",
                default_value="/drive",
                description="Ackermann drive command topic.",
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
                parameters=[
                    {
                        "raceline_path": raceline_path,
                        "base_frame": base_frame,
                        "drive_topic": drive_topic,
                        "min_lookahead": 0.7,
                        "max_lookahead": 3.0,
                        "lookahead_gain": 0.35,
                        "wheelbase": 0.33,
                        "max_steering_angle": 0.42,
                        "target_speed_scaling": 0.65,
                        "curvature_speed_gain": 0.5,
                    }
                ],
            ),
        ]
    )
