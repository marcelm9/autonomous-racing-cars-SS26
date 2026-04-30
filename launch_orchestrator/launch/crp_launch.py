#!/usr/bin/env python3
# create_raceline_and_start_pure_persuit_launch.py
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

    # --- ARGUMENTS --- #
    ld.append(
        DeclareLaunchArgument(
            name = "safety",
            default_value = "12",
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
            default_value = "True",
            description = (
                "when True, the path is created as a closed loop"
            ),
        )
    )


    # --- NODES --- #
    
    # Map server
    ld.append(
        Node(
            package = "nav2_map_server",
            executable = "map_server",
            name = "map_server",
            parameters = [
                {
                    "yaml_filename": PathJoinSubstitution([
                        FindPackageShare("launch_orchestrator"),
                        "maps",
                        "crazy_map.yaml",
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
    
    # PurePersuite Node
    ld.append(
        Node(
            package='pure_pursuit',
            executable='controller',
            name='controller',
            output='screen',
            remappings=[]
        )
    )



    return LaunchDescription(ld)
