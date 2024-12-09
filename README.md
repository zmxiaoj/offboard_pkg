## Basic offboard ros package

Three are two parts for the ros package "offboard_pkg"
- Utils is for the basic tools for the pose&position estimator
- Offboard is for the offboard control

### Utils 
```shell
## offboard_pos_estimator.launch
## launch 1.mavros 2.vrpn 3.pos_estimator node
roslaunch offboard_pkg offboard_pos_estimator.launch

```




### Offboard

```shell
## offboard_node.launch
## takeoff_position_* is the position for the takeoff setpoint
roslaunch offboard_pkg offboard_node.launch takeoff_position_x:=2.0 takeoff_position_y:=2.0 takeoff_position_z:=2.0

### land command 
rostopic pub /land_command std_msgs/Bool "data: true"


## offboard_position.launch
## 
roslaunch offboard_pkg offboard_position.launch 

## offboard_rosbag.launch
## get setpoint from rosbag and publish to mavros
## defalult rosbag_path:=/home/amov/0713.bag
roslaunch offboard_pkg offboard_rosbag.launch rosbag_path:=/path/to/your/rosbag.bag
```

