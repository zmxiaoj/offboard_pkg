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
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position");
    ros::NodeHandle nh;
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
           // ("mavros/setpoint_velocity/cmd_vel", 10);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
 
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    int state = 3;
    ros::Time last_request = ros::Time::now();
    while(ros::ok() ){
       
       if( !current_state.armed ){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
            }
        if( current_state.mode != "OFFBOARD"){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
        } 
 
        if( (ros::Time::now() - last_request > ros::Duration(5.0))) break;
 
        ros::spinOnce();
        rate.sleep();
    }
    while(state--) {
        last_request = ros::Time::now();
        while(ros::ok()) {
            if( (ros::Time::now() - last_request > ros::Duration(5.0))) break;
           
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            local_pos_pub.publish(pose);
            ROS_INFO("SUCCESS0");
            ros::spinOnce();
            rate.sleep();
        }
        last_request = ros::Time::now();
        while(ros::ok()) {
           if( (ros::Time::now() - last_request > ros::Duration(5.0))) break;
           
           pose.pose.position.x = 2;
           pose.pose.position.y = 2;
           local_pos_pub.publish(pose);
           ROS_INFO("SUCCESS1");
           ros::spinOnce();
           rate.sleep();
       }
       ROS_INFO_STREAM("state="<<state);
    }

    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }
 
    return 0;
}
