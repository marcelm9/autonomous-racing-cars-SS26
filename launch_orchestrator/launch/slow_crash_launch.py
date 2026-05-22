import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_launch_orchestrator = get_package_share_directory("launch_orchestrator")

    try:
        pkg_path = get_package_share_directory("f1tenth_stack")
        launch_file = "bringup_launch.py"
    except:
        pkg_path = get_package_share_directory("f1tenth_gym_ros")
        launch_file = "gym_bridge_launch.py"

    car_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, "launch", launch_file))
    )

    safety_node = Node(
        package="safety_node1",
        executable="safety_node",
        name="safety_node",
        output="screen",
        remappings=[
            ("/drive_out", "/drive"),
            ("/ego_racecar/odom", "/odom"),
        ],
    )

    slow_crash = Node(
        package="slow_crash",
        executable="slow_crash",
        name="slow_crash",
        output="screen",
    )

    diagnostics_config = os.path.join(
        pkg_launch_orchestrator, "diagnostics", "config", "diagnostics_aggregator.yaml"
    )

    diagnostics_aggregator = Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        name="diagnostics_aggregator",
        output="screen",
        parameters=[diagnostics_config],
    )

    return LaunchDescription(
        [car_bringup_launch, safety_node, slow_crash, diagnostics_aggregator]
    )
