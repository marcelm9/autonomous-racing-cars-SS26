import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    # Path to the car launch file
    pkg_f1tenth_stack = get_package_share_directory('f1tenth_stack')
    
    car_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_stack, 'launch', 'bringup_launch.py')
        )
    )

    #s Safety Node
    safety_node = Node(
        package='safety_node1',
        executable='safety_node',
        name='safety_node',
        output='screen',
        remappings=[
            ('/drive_out', '/ackermann_cmd'),
            # The physical car odom is usually just /odom
            ('/ego_racecar/odom', '/odom'), 
            # Make sure your safety node's drive output goes to the mux
            #('/drive', '/ackermann_cmd'),
        ]
    )
    
    
    # slow crash node
    slow_crash = Node(
        package='slow_crash',
        executable='slow_crash',
        name='slow_crash',
        output='screen',
        remappings=[
        ]
    )

    return LaunchDescription([
        car_bringup_launch,
        safety_node,
        slow_crash
    ])