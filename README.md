# Offboard ROS Package

ROS package for UAV offboard control with various functionalities:
- Basic position control
- Trajectory following with motion capture
- Position estimation utilities

## Utils 
```shell
# offboard_mocap_flight
## Launch 1.mavros 2.vrpn 3.pos_estimator node
## input_source:= rigid_body_name:= mocap_frame_type:= 
### For multi-terminal
roslaunch offboard_pkg offboard_mocap_flight.launch
### For single-terminal
roslaunch offboard_pkg offboard_mocap_flight_nx.launch
```

### Offboard

```shell
# offboard_hover
## takeoff_position_* is the position for the takeoff setpoint
roslaunch offboard_pkg offboard_hover.launch takeoff_position_x:=2.0 takeoff_position_y:=2.0 takeoff_position_z:=2.0
## land command 
rostopic pub -r 1 /land_cmd std_msgs/Bool "data: true"


# offboard_traverse
## 
roslaunch offboard_pkg offboard_traverse.launch hover_time:=10.0
## land command 
rostopic pub -r 1 /land_cmd std_msgs/Bool "data: true"

# offboard_trajectory
## Terminal_0
roslaunch offboard_pkg offboard_trajectory.launch
## Terminal_1
### Input "hover/circle/rectangle/land/Ctrl+C"
python3 offboard_pkg/script/send_trajectory_cmd.py

```

