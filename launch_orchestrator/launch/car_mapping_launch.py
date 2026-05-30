import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    slam_params_file = LaunchConfiguration("slam_params_file")

    # This node must be instantiated as a variable first so we can
    # reference it in the lifecycle event handlers below.
    start_sync_slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {
                "use_lifecycle_manager": False,
                "use_sim_time": False,
            },
        ],
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        namespace="",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(
                    get_package_share_directory("launch_orchestrator"),
                    "config",
                    "my_mapper_params_online_sync.yaml",
                ),
                description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
            ),
            start_sync_slam_toolbox_node,
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                ),
            ),
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=start_sync_slam_toolbox_node,
                    start_state="configuring",
                    goal_state="inactive",
                    entities=[
                        LogInfo(
                            msg="[LifecycleLaunch] Slamtoolbox node is activating."
                        ),
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(
                                    start_sync_slam_toolbox_node
                                ),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        ),
                    ],
                ),
            ),
            # basic launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("f1tenth_stack"),
                        "launch",
                        "bringup_launch.py",
                    )
                )
            ),
            # reactive node launch
            Node(
                package="reactive",
                executable="reactive",
                name="reactive",
                output="screen",
                parameters=[
                    {
                        "scan_topic": "/scan",
                        "odom_topic": "/odom",
                        "drive_topic": "/drive",
                        "min_speed": 1.0,
                        "max_speed": 1.0,
                        "reac_n_extend": 300,
                    }
                ],
            ),
        ]
    )
