import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = []

    # SIM
    ld.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("f1tenth_gym_ros"),
                    "launch",
                    "gym_bridge_launch.py",
                )
            )
        )
    )

    # PURE PURSUIT
    # ld.append(
    #     Node(
    #         package="pure_pursuit",
    #         executable="controller",
    #         name="controller",
    #         output="screen",
    #         remappings=[],
    #     )
    # )

    # RACELINE
    ld.append(
        Node(
            package="opt_raceline",
            executable="raceline_csv_publisher",
            name="raceline_csv_publisher",
        )
    )

    # PARTICLE FILTER (not needed for sim)
    # ld.append(
    #     Node(
    #         package="particle_filter",
    #         executable="particle_filter",
    #         name="particle_filter",
    #         parameters=[
    #             os.path.join(
    #                 get_package_share_directory("particle_filter"),
    #                 "config",
    #                 "localize.yaml",
    #             )
    #         ],
    #         output="screen",
    #     )
    # )

    return LaunchDescription(ld)
