# MANUAL

## General stuff

### Build only one package
`colcon build --packages-select reactive`

### Source Files
`source /opt/ros/jazzy/setup.bash && source install/setup.bash`

### VESC config
- `speed_to_erpm_gain: 7500.0`
- `speed_to_erpm_offset: -32.0`
- `steering_angle_to_servo_gain: -1.8`
- `steering_angle_to_servo_offset: 0.56`
- `servo_min: 0.175`
- `servo_max: 0.99`

### Launching stuff

Launch files should all be in `launch_orchestrator/launch`. Naming convention: prefix `car_` means the launch file is meant for the actual racecar, prefix `sim_` means the launch file is meant for the simulator. For most stuff there already exists a launch file you can copy stuff from (eg. `car_reactive_launch.py`, `sim_reactive_launch.py`, ...).

## Creating a map (SLAM)

1) `ros2 launch launch_orchestrator car_mapping_launch.py`
2) drive around the map to build it (2 - 3 laps recommended), then contine with the next step
3) `ros2 run nav2_map_server map_saver_cli -f ~/racecar_ws/src/arc_riders/launch_orchestrator/maps/MAP_NAME_HERE`

### Potential issues

- If not map is being shown, most likely the tf tree is broken. I honestly don't know how to fix it, but there is a workaround: Publish a static transform using the command `ros2 run tf2_ros static_transform_publisher 0.27 0 0.11 0 0 0 ego_racecar/base_link laser`. Note that this apparently has to be started AFTER the other `car_mapping_launch.py` command.

- For maps with double sided walls (walls that are seen from both sides during the race) the car often tries to find some connection to aggressively. In this case it is recommended to only do one lap for the mapping.

## Localization
To achieve localization, take a look at the launch file `car_pure_pursuit_launch.py`. The important part here is that parallel to the particle filter you also have to launch a map server and its lifecycle node. The argument for why it is implemented this way is that you can now use the parameter `map_name` instead of editing the file `localize.yaml`. This makes switching between maps and their corresponding racelines more convenient and also reduces the potential for errors (eg. forgetting to edit `localize.yaml`).

To visualize the particle filter add the `/pf/viz/inferred_pose` and `/pf/viz/particles` topics.

## raceline (DEPRECATED)
First install neccessary python package in your container
`sudo apt update && sudo apt install python3-skimage`
To launch run
`ros2 launch simple_raceline test_launch.py`

INFO: This is for the old "simple_raceline" node provided by the lecture.

## Use mapserver to show map in rviz
1) `ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=MAP_FILENAME_PATH-yaml`
2) in different terminal:
 `ros2 run nav2_util lifecycle_bringup map_server`
3) In rviz2 include topic /maps and set topic
    History Policy: `Keep Last` 
    Reliability Policy: `Reliable`
    Durability Policy: `Transiet Local` 


## Foxglove Setup
im Container installieren:

```sh
sudo apt update
sudo apt install ros-jazzy-foxglove-bridge
```

diese node muss im Hintergrund laufen:

```sh
ros2 run foxglove_bridge foxglove_bridge
```

Port 8765 nach außen legen:
in VS Code neben dem Terminal bei Ports 8765 hinzufügen

Bei Foxglove anmelden: `studio.foxglove.dev`
configs finden sich in `configs/foxglove`

## git naming convention

branches:
- feature/...
- bugfix/...

commits:
- added ...
- removed ...
- changed ...

## (Perfect) TUM Raceline

### Prerequisites
You need a configured conda environment with python 3.8

```sh
conda create -n raceline_clean python=3.8 -y
conda activate raceline_clean
conda install -c conda-forge numpy=1.19.5 scipy=1.6.3 matplotlib=3.3.4 scikit-learn=0.23.2 pyyaml quadprog -y
python -m pip install trajectory_planning_helpers==0.76 --no-deps
python -m pip install casadi==3.5.5 opencv-contrib-python --no-deps
```

### Create a new optimal raceline
(This does not work on macs with silicon chips)

1. make sure your map.pgm and your `map.yaml` is located in `launch_orchestrator/maps`
2. in `opt_raceline_algo/config/params.yaml` set the map name correct and choose your safety distance
3. activate the conda env with `conda activate <my_env_name>`
4. run `python lane_generator.py`
    you may need to edit the `pgm` map file with a photo editing software to ensure cleanly closed borders !
5. run `python main_globaltraj.py`
    you may need to adapt the parameters in `params/racecar.ini` !
    Especially 
    - `stepsize_prep`
    - `stepsize_reg`
    - `stepsize_interp_after_opt`
    - `k_reg`
    - `s_reg`
6. run `python raceline_scripts.py`
    The optimal raceline is now stored in `outputs/map_name/traj_race_cl.csv` .
7. Move the generated raceline (`outputs/map_name/traj_race_cl.csv`) into the maps folder (`launch_orchestrator/maps`). The name of the file should be `<MAP_NAME>.csv`, for example `crazy_map.csv`.

## Lap Timer
To use the lap timer, create the file `<MAP_NAME>.checkpoints` in the directory `launch_orchestrator/maps`. Take inspiration from `crazy_map.checkpoints`. The first line is the starting line, every following line stands for one checkpoint. The format is `x1, y1, x2, y2`. Then add the node to the launch configuration (check out `sim_pure_pursuit_launch.py`).

## Drive Combiner

If the cone / lane shown in rviz2 (`/drive_combiner/cone_marker`) does not move with the car, you have to publish a static transform: `ros2 run tf2_ros static_transform_publisher 0.27 0 0.11 0 0 0 particle_filter_base_frame laser2`.



## Opt raceline repo
https://github.com/zzjun725/f1tenth-racing-stack-ICRA22
