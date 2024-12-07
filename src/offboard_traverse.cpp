#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <sstream>
#include <iostream>


mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg);

 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traverse_node");
    ros::NodeHandle nh("~");
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    //ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
           // ("mavros/setpoint_velocity/cmd_vel", 10);


    double takeoff_x = 0.0, takeoff_y = 0.0, takeoff_z = 1.0,
           patrol_x = 1.0,  patrol_y = 1.0,  patrol_z = 1.0;
    
    nh.param("takeoff_x", takeoff_x, 0.0);
    nh.param("takeoff_y", takeoff_y, 0.0);
    nh.param("takeoff_z", takeoff_z, 1.0);
    nh.param("patrol_x",  patrol_x,  1.0);
    nh.param("patrol_y",  patrol_y,  1.0);
    nh.param("patrol_z",  patrol_z,  1.0);
    
    ROS_INFO("takeoff_x: %f, takeoff_y: %f, takeoff_z: %f", takeoff_x, takeoff_y, takeoff_z);
    ROS_INFO("patrol_x: %f, patrol_y: %f, patrol_z: %f", patrol_x, patrol_y, patrol_z);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
 
    geometry_msgs::PoseStamped takeoff_pose, patrol_pose;
    takeoff_pose.pose.position.x = takeoff_x;
    takeoff_pose.pose.position.y = takeoff_y;
    takeoff_pose.pose.position.z = takeoff_z;
    patrol_pose.pose.position.x  = patrol_x;
    patrol_pose.pose.position.y  = patrol_y;
    patrol_pose.pose.position.z  = patrol_z;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(takeoff_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    int state = 3;
    ros::Time last_request = ros::Time::now();
    while (ros::ok()) {
       
        if (!current_state.armed) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
        }
        if (current_state.mode != "OFFBOARD") {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
        } 
 
        if ((ros::Time::now() - last_request > ros::Duration(5.0))) 
            break;
 
        ros::spinOnce();
        rate.sleep();
    }
    while (state--) {
        last_request = ros::Time::now();
        while (ros::ok()) {
            if (ros::Time::now() - last_request > ros::Duration(5.0)) 
                break;

            local_pos_pub.publish(takeoff_pose);
            ROS_INFO("STATE0");
            ros::spinOnce();
            rate.sleep();
        }
        last_request = ros::Time::now();
        while (ros::ok()) {
            if (ros::Time::now() - last_request > ros::Duration(5.0)) 
                break;
        
            local_pos_pub.publish(patrol_pose);
            ROS_INFO("STATE1");
            ros::spinOnce();
            rate.sleep();
       }
       ROS_INFO_STREAM("Num od state=" << state);
    }

    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }
 
    return 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
