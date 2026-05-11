#!/usr/bin/env python3
# create_raceline_and_start_pure_persuit_launch.py


import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.conditions import IfCondition

def generate_launch_description() -> LaunchDescription:
    """Entry point function called by ros2 launch."""
    safety = LaunchConfiguration("safety")
    sparse = LaunchConfiguration("sparse")
    closed_loop = LaunchConfiguration("closed_loop")
    map_file = LaunchConfiguration("map")

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Selection of hardware target: "sim", "car", or "none"'
    )
    
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    drive_topic = LaunchConfiguration("drive_topic")
    
    ld = []
    
    ld.append(DeclareLaunchArgument("use_sim", default_value="false", description="Launch simulator?"))
    ld.append(DeclareLaunchArgument("car_launch", default_value="false", description="Launch car?"))
    ld.append(DeclareLaunchArgument("scan_topic", default_value="/scan"))
    ld.append(DeclareLaunchArgument("odom_topic", default_value="/ego_racecar/odom"))
    ld.append(DeclareLaunchArgument("drive_topic", default_value="/drive"))
    

    # --- ARGUMENTS --- #
    ld.append(
        DeclareLaunchArgument(
            "map",
            default_value="crazy_map.yaml",
            description="Name of the map yaml file in the launch_orchestrator/maps folder"
        )
    )
    ld.append(
        DeclareLaunchArgument(
            name = "safety",
            default_value = "7",
            description = (
                "[-] amount of pixels eroded from the driveable area"
            ),
        )
    )

    ld.append(
        DeclareLaunchArgument(
            name = "sparse",
            default_value = "1",
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
                        map_file,
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
    
    # SimpleRaceline Node
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
                    "algorithm": "centered",
                },
            ],
        )
    )
    
    # Particle Filter Node

    localize_config = os.path.join(
        get_package_share_directory('particle_filter'),
        'config',
        'localize.yaml'
    )

    ld.append(
        Node(
            package="particle_filter",
            executable="particle_filter",
            name="particle_filter",
            parameters=[localize_config],
            output="screen",
        )
    )
    
    # PurePersuite Node
    ld.append(
        Node(
            package='pure_pursuit',
            executable='controller',
            name='controller',
            output='screen',
            remappings=[('/scan', scan_topic),
                ('/odom', odom_topic),
                ('/drive', drive_topic),]
        )
    )


  
    

    ld.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge_launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'sim'"]))
    ))

    ld.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('f1tenth_stack'), 'launch', 'bringup_launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'car'"]))
    ))

    ld.append(mode_arg) 

    return LaunchDescription(ld)
