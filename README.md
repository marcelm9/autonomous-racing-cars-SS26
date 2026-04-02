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