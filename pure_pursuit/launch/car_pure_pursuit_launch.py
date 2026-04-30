import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('f1tenth_stack')
    launch_file = 'bringup_launch.py'
    basic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', launch_file)
        )
    )
    
    pure_pursuit = Node(
        package='pure_pursuit',
        executable='controller',
        name='controller',
        output='screen',
        remappings=[
            # Example: ('internal_name', 'simulator_name')
            # ('/scan', '/scan'), 
            # ('/odom', '/ego_racecar/odom'),
        ]
    )

    return LaunchDescription([
        basic_launch,
        pure_pursuit
    ])