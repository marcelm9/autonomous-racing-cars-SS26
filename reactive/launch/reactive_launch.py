import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to the simulator launch file
    pkg_f1tenth_gym_ros = get_package_share_directory('f1tenth_gym_ros')
    
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_gym_ros, 'launch', 'gym_bridge_launch.py')
        )
    )

    # 2. Your Reactive Node
    # Note: Topic names in the simulator are usually /scan and /ego_racecar/odom.
    # If your code uses different names, use the 'remappings' argument below.
    reactive = Node(
        package='reactive',
        executable='reactive',
        name='reactive',
        output='screen',
        remappings=[
            # Example: ('internal_name', 'simulator_name')
            # ('/scan', '/scan'), 
            # ('/odom', '/ego_racecar/odom'),
        ]
    )

    return LaunchDescription([
        sim_launch,
        reactive
    ])