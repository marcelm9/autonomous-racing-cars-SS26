# STARTUP
 
## Build only one package
colcon build --packages-select reactive

## Source Files
source /opt/ros/jazzy/setup.bash
source install/setup.bash

## Things you might want to launch

### better_keyboard_control
ros2 run better_keyboard_control better_keyboard_control

### gym_bridge_launch
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

### safety_node1
ros2 launch safety_node1 safety_node1_launch.py

### reconfigure
ros2 run rqt_reconfigure rqt_reconfigure

### request ebrake disengagement
ros2 service call /reset_safety_node std_srvs/srv/Trigger {}


### Launch reactive node
ros2 launch reactive reactive_launch.py

### Launch reactive and safety node
ros2 launch launch_orchestrator safe_reactive_launch.py

# VESC config
- speed_to_erpm_gain: 7500.0
- speed_to_erpm_offset: -32.0
- steering_angle_to_servo_gain: -1.8
- steering_angle_to_servo_offset: 0.56
- servo_min: 0.175
- servo_max: 0.99