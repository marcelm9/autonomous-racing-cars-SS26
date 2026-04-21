# STARTUP
 
## Build only one package
`colcon build --packages-select reactive`

## Source Files
`source /opt/ros/jazzy/setup.bash`
`source install/setup.bash`

## Things you might want to launch

### better_keyboard_control
`ros2 run better_keyboard_control better_keyboard_control`

### gym_bridge_launch
`ros2 launch f1tenth_gym_ros gym_bridge_launch.py`

### safety_node1
`ros2 launch safety_node1 safety_node1_launch.py`

### reconfigure
`ros2 run rqt_reconfigure rqt_reconfigure`

### request ebrake disengagement
`ros2 service call /reset_safety_node std_srvs/srv/Trigger {}`


### Launch reactive node
`ros2 launch reactive reactive_launch.py`

### Launch reactive and safety node
`ros2 launch launch_orchestrator safe_reactive_launch.py`

# VESC config
- `speed_to_erpm_gain: 7500.0`
- `speed_to_erpm_offset: -32.0`
- `steering_angle_to_servo_gain: -1.8`
- `steering_angle_to_servo_offset: 0.56`
- `servo_min: 0.175`
- `servo_max: 0.99`

# SLAM and Localization

Make sure to not have the `bringup_launch` running separately!

The config file for the particle filter is located [here](~/racecar_ws/src/arc_riders/particle_filter/config/localize.yaml)

1) `ros2 launch reactive car_reactive_launch.py`
2) `ros2 launch launch_orchestrator slam_toolbox_launch.py`
3) `ros2 run nav2_map_server map_saver_cli -f ~/racecar_ws/src/arc_riders/particle_filter/maps/MAP_NAME_HERE`
4) kill slam process
5) edit the filename in `~/racecar_ws/src/arc_riders/particle_filter/config/localize.yaml` (line 36)
6) rebuild (`cd ~/racecar_ws && cb && src`)
7) `ros2 launch particle_filter localize_launch.py`

To have a visualization of the map just run rviz2 with the `/map` topic. To visualize the particle filter add the `/pf/viz/inferred_pose` and `/pf/viz/particles` topics.
