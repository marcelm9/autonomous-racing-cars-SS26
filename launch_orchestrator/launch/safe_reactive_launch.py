import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    # Path to the car launch file
    try:
        pkg_f1tenth_stack = get_package_share_directory('f1tenth_stack')
        
        basic_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_f1tenth_stack, 'launch', 'bringup_launch.py')
            )
        )
    
    except:
        pkg_f1tenth_gym_ros = get_package_share_directory('f1tenth_gym_ros')
    
        basic_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_f1tenth_gym_ros, 'launch', 'gym_bridge_launch.py')
            )
        )

    # Safety Node
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

    
    # reactive node
    reactive = None(
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