import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the car launch file
    try:
        pkg_path = get_package_share_directory('f1tenth_stack')
        launch_file = 'bringup_launch.py'
    except:
        pkg_path = get_package_share_directory('f1tenth_gym_ros')
        launch_file = 'gym_bridge_launch.py'

    basic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', launch_file)
        )
    )

    # Safety Node
    safety_node = Node(
        package='safety_node1',
        executable='safety_node',
        name='safety_node',
        output='screen',
        remappings=[
            ('/drive_out', '/drive'),
            ('/ego_racecar/odom', '/odom'), 
        ]
    )

    
    # reactive node
    reactive = Node(
        package='reactive',
        executable='reactive',
        name='reactive',
        output='screen',
        remappings=[
            ('/drive', '/drive_in')
        ]
    )

    return LaunchDescription([
        basic_launch,
        safety_node,
        reactive        
    ])