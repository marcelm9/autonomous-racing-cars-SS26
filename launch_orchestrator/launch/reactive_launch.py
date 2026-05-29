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
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Whether to launch the f1tenth simulator bridge'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic name for laser scan'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/ego_racecar/odom',
        description='Topic name for odometry'
    )
    
    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic',
        default_value='/drive',
        description='Topic name for drive commands'
    )


    pkg_f1tenth_gym_ros = get_package_share_directory('f1tenth_gym_ros')
    
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_f1tenth_gym_ros, 'launch', 'gym_bridge_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    
    reactive = Node(
        package='reactive',
        executable='reactive',
        name='reactive',
        output='screen',
        parameters=[{}],
        remappings=[
            ('/scan', LaunchConfiguration('scan_topic')), 
            ('/odom', LaunchConfiguration('odom_topic')),
            ('/drive', LaunchConfiguration('drive_topic')),
        ]
    )

    return LaunchDescription([
        use_sim_arg,
        scan_topic_arg,
        odom_topic_arg,
        drive_topic_arg,

        sim_launch,
        reactive
    ])