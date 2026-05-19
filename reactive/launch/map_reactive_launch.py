import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    ld = []
        
    ld.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Whether to launch the f1tenth simulator bridge'
        )
    )
    
    
    ld.append(
        scan_topic_arg = DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Topic name for laser scan'
        )
    )
    
    ld.append(
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/ego_racecar/odom',
            description='Topic name for odometry'
        )
    )
    
    ld.append(
        DeclareLaunchArgument(
            'drive_topic',
            default_value='/drive',
            description='Topic name for drive commands'
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
    
    pkg_f1tenth_gym_ros = get_package_share_directory('f1tenth_gym_ros')
    
    ld.append(
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_gym_ros, 'launch', 'gym_bridge_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )
    )

    ld.append(
        Node(
            package='reactive',
            executable='reactive',
            name='reactive',
            output='screen',
            parameters=[{
                'max_speed': LaunchConfiguration('max_speed')
            }],
            remappings=[
                ('/scan', LaunchConfiguration('scan_topic')), 
                ('/odom', LaunchConfiguration('odom_topic')),
                ('/drive', LaunchConfiguration('drive_topic')),
            ]
        )
    )

    return LaunchDescription(ld)
