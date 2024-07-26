## Basic offboard ros package

### waypoint tracker 

- P450
```
# defalult rosbag_path:=/home/amov/0713.bag
roslaunch offboard_pkg offboard_rosbag.launch rosbag_path:=/path/to/your/rosbag.bag
```

- A1_12

```
# offboard_node.launch
roslaunch offboard_pkg offboard_node.launch takeoff_position_x:=0.0 takeoff_position_y:=0.0 takeoff_position_z:=1.0

# offboard_position.launch
roslaunch offboard_pkg offboard_position.launch patrol_x:=2.3 patrol_y:=222 patrol_z:=1.2 takeoff_x:=1.2 takeoff_y:=3.2 takeoff_z:=4.2

# offboard_rosbag.launch
roslaunch offboard_pkg offboard_rosbag.launch rosbag_path:=/home/zmxj/code/Datasets/offboard/0720_tree_0781_2.bag current_position_x:=0.0 current_position_y:=0.0
```
