import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    # 1. Path to the simulator launch file
    pkg_f1tenth_stack = get_package_share_directory('f1tenth_stack')
    
    car_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_stack, 'launch', 'bringup_launch.py')
        )
    )

    # 2. Your Safety Node
    # Note: Topic names in the simulator are usually /scan and /ego_racecar/odom.
    # If your code uses different names, use the 'remappings' argument below.
    safety_node = Node(
        package='safety_node1',
        executable='safety_node',
        name='safety_node',
        output='screen',
        remappings=[
            ('/scan', '/scan'),
            # The physical car odom is usually just /odom
            ('/ego_racecar/odom', '/odom'), 
            # Make sure your safety node's drive output goes to the mux
            ('/drive', '/ackermann_cmd'),
        ]
    )

    return LaunchDescription([
        car_bringup_launch,
        safety_node
    ])