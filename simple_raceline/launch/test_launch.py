#!/usr/bin/env python3
# test_launch.py
"""Launch file for demonstrating and testing the simple_raceline.

Source:
https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-system.html
https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-Substitutions.html
"""

from launch import (
    LaunchDescription,
)
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
    TimerAction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import (
    Node,
)
from launch_ros.substitutions import (
    FindPackageShare,
)


def generate_launch_description() -> LaunchDescription:
    """Entry point function called by ros2 launch."""
    safety = LaunchConfiguration("safety")
    sparse = LaunchConfiguration("sparse")
    closed_loop = LaunchConfiguration("closed_loop")
    map_yaml = LaunchConfiguration("map_yaml")
    algorithm = LaunchConfiguration("algorithm")

    ld = []

    # Arguments
    ld.append(
        DeclareLaunchArgument(
            name="map_yaml",
            # Provide a fallback default or leave it empty to force user input
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("launch_orchestrator"),
                    "maps",
                    "crazy_map.yaml",
                ]
            ),
            description="Full path to the SLAM-generated map yaml file",
        )
    )

    ld.append(
        DeclareLaunchArgument(
            name="algorithm",
            default_value="simple",
            description=("algorithm to use [options: 'simple', 'centered']"),
        )
    )

    ld.append(DeclareLaunchArgument(name="safety", default_value="5"))

    ld.append(
        DeclareLaunchArgument(
            name="sparse",
            default_value="5",
            description=("[-] 1 / sampling ratio for the final raceline"),
        )
    )

    ld.append(
        DeclareLaunchArgument(
            name="closed_loop",
            default_value="True",
            description=("when True, the path is created as a closed loop"),
        )
    )

    # Nodes
    # Map server
    ld.append(
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[
                {
                    "yaml_filename": map_yaml,
                },
            ],
        )
    )

    # Lifecycle manager (to turn on the map_server)
    # Wrapped in a TimerAction to allow DDS discovery to settle
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        parameters=[
            {
                "node_names": [
                    "map_server",
                ],
                "autostart": True,
                "bond_timeout": 4.0,  # Increased timeout for stability
            },
        ],
    )

    ld.append(
        TimerAction(
            period=2.0,  # 2 second delay before starting the lifecycle manager
            actions=[lifecycle_manager_node],
        )
    )

    # Simple raceline node
    ld.append(
        Node(
            package="simple_raceline",
            executable="simple_raceline",
            name="simple_raceline",
            parameters=[
                {
                    "algorithm": algorithm,
                    "safety": safety,
                    "sparse": sparse,
                    "closed_loop": closed_loop,
                },
            ],
        )
    )

    # rViz2
    ld.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare("simple_raceline"),
                        "launch",
                        "test_launch.rviz",
                    ]
                ),
            ],
            on_exit=Shutdown(),
        )
    )

    return LaunchDescription(ld)
