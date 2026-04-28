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

    ld = []

    # Arguments
    ld.append(
        DeclareLaunchArgument(
            name = "safety",
            default_value = "5",
            description = (
                "[-] amount of pixels eroded from the driveable area"
            ),
        )
    )

    ld.append(
        DeclareLaunchArgument(
            name = "sparse",
            default_value = "5",
            description = (
                "[-] 1 / sampling ratio for the final raceline"
            ),
        )
    )

    ld.append(
        DeclareLaunchArgument(
            name = "closed_loop",
            default_value = "False",
            description = (
                "when True, the path is created as a closed loop"
            ),
        )
    )

    # Nodes
    # Map server
    ld.append(
        Node(
            package = "nav2_map_server",
            executable = "map_server",
            name = "map_server",
            parameters = [
                {
                    "yaml_filename": PathJoinSubstitution([
                        FindPackageShare("simple_raceline"),
                        "maps",
                        "bunch_of_sticks.yaml",
                    ])
                },
            ],
        )
    )

    # Lifecycle manager (to turn on the map_server)
    ld.append(
        Node(
            package = "nav2_lifecycle_manager",
            executable = "lifecycle_manager",
            name = "lifecycle_manager",
            parameters = [
                {
                    "node_names": [
                        "map_server",
                    ],
                    "autostart": True,
                },
            ],
        )
    )

    # Simple raceline node
    ld.append(
        Node(
            package = "simple_raceline",
            executable = "simple_raceline",
            name = "simple_raceline",
            parameters = [
                {
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
            package = "rviz2",
            executable = "rviz2",
            name = "rviz2",
            arguments = [
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("simple_raceline"),
                    "launch",
                    "test_launch.rviz",
                ]),
            ],
            on_exit = Shutdown(),
        )
    )


    return LaunchDescription(ld)
