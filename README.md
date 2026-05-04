# STARTUP
 
## Build only one package
`colcon build --packages-select reactive`

## Source Files
`source /opt/ros/jazzy/setup.bash && source install/setup.bash`

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
3) drive around the map to build it (2 - 3 laps recommended), then contine with the next step
4) `ros2 run nav2_map_server map_saver_cli -f ~/racecar_ws/src/arc_riders/launch_orchestrator/maps/MAP_NAME_HERE`
5) kill slam process
6) edit the filename in `~/racecar_ws/src/arc_riders/particle_filter/config/localize.yaml` (line 36)
7) rebuild (`cd ~/racecar_ws && cb && src`)
8) `ros2 launch particle_filter localize_launch.py`

To have a visualization of the map just run rviz2 with the `/map` topic. To visualize the particle filter add the `/pf/viz/inferred_pose` and `/pf/viz/particles` topics.

# raceline
First install neccessary python package in your container
`sudo apt update && sudo apt install python3-skimage`
To launch run
`ros2 launch simple_raceline test_launch.py`


### Use mapserver to show map in rviz
1) `ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=MAP_FILENAME_PATH-yaml`
2) in different terminal:
 `ros2 run nav2_util lifecycle_bringup map_server`
3) In rviz2 include topic /maps and set topic
    History Policy: `Keep Last` 
    Reliability Policy: `Reliable`
    Durability Policy: `Transiet Local` 


### Launch crp_launch Arguments
ros2 launch launch_orchestrator crp_launch.py map:=MAP_Name 


