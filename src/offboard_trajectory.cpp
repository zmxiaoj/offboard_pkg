#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <string>
#include <sstream>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <eigen3/Eigen/Eigen>

mavros_msgs::State current_state;
bool land_command_received = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void land_command_cb(const std_msgs::Bool::ConstPtr& msg);
void pattern_command_cb(const std_msgs::Int8::ConstPtr& msg);

void publish_setpoint(const ros::Publisher& pub, const geometry_msgs::PoseStamped& pose, ros::Rate& rate, ros::Duration timeout);
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_trajectory_node");
    ros::NodeHandle nh("~");
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber land_command_sub = nh.subscribe<std_msgs::Bool>
            ("/land_command", 10, land_command_cb);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    //ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
           // ("/mavros/setpoint_velocity/cmd_vel", 10);


    float takeoff_x = 0.0, takeoff_y = 0.0, takeoff_z = 1.0,
           patrol_x = 1.0,  patrol_y = 1.0,  patrol_z = 1.0;
    
    nh.param<float>("takeoff_x", takeoff_x, 0.0);
    nh.param<float>("takeoff_y", takeoff_y, 0.0);
    nh.param<float>("takeoff_z", takeoff_z, 1.0);
    nh.param<float>("patrol_x",  patrol_x,  1.0);
    nh.param<float>("patrol_y",  patrol_y,  1.0);
    nh.param<float>("patrol_z",  patrol_z,  1.0);

    Eigen::Vector3f takeoff_position(takeoff_x, takeoff_y, takeoff_z),
                    patrol_position(patrol_x, patrol_y, patrol_z);
    
    // clamp flight height
    takeoff_position.z() = std::min(std::max(takeoff_position.z(), 0.0f), 5.0f);
    patrol_position.z() = std::min(std::max(patrol_position.z(), 0.0f), 5.0f);

    ROS_INFO_STREAM("Takeoff Position(x y z): " << std::endl << takeoff_position.transpose());
    ROS_INFO_STREAM("Patrol Position(x y z): " << std::endl << patrol_position.transpose());
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
 
    geometry_msgs::PoseStamped takeoff_setpoint, patrol_setpoint;
    takeoff_setpoint.pose.position.x = takeoff_position.x();
    takeoff_setpoint.pose.position.y = takeoff_position.y();
    takeoff_setpoint.pose.position.z = takeoff_position.z();
    patrol_setpoint.pose.position.x  = patrol_position.x();
    patrol_setpoint.pose.position.y  = patrol_position.y();
    patrol_setpoint.pose.position.z  = patrol_position.z();

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(takeoff_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD") {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                ROS_INFO("Offboard enabled");
            else 
                ROS_WARN("Failed to OFFBOARD mode");
        } 
        if (!current_state.armed) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                ROS_INFO("Vehicle armed");
            else 
                ROS_WARN("Failed to ARM");
        }

        if ((ros::Time::now() - last_request > ros::Duration(5.0))) 
            break;
 
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        if (land_command_received) {
            ROS_INFO("Land command received, exiting offboard mode and landing...");
            break;
        }
        
        ROS_INFO_STREAM("Takeoff Position: " << std::endl << takeoff_position.transpose());
        publish_setpoint(local_pos_pub, takeoff_setpoint, rate, ros::Duration(5.0));

        if (land_command_received) {
            ROS_INFO("Land command received, exiting offboard mode and landing...");
            break;
        }

        ROS_INFO_STREAM("Patrol Position: " << std::endl << patrol_position.transpose());
        publish_setpoint(local_pos_pub, patrol_setpoint, rate, ros::Duration(5.0));

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
void land_command_cb(const std_msgs::Bool::ConstPtr& msg) 
{
    land_command_received = msg->data;
}

void publish_setpoint(const ros::Publisher& setpoint_publisher, const geometry_msgs::PoseStamped& target_setpoint, ros::Rate& rate, ros::Duration timeout)
{
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        if (land_command_received) {
            break;
        }
        if (ros::Time::now() - start_time > timeout) {
            break;
        }
        setpoint_publisher.publish(target_setpoint);
        ros::spinOnce();
        rate.sleep();
    }
}
