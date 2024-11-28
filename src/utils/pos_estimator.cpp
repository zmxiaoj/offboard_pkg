/**
 * @file pos_estimator.cpp
 * @author ZMXiaoJ
 * @brief provide postion&pose information for UAV
 * @version 0.1
 * @date 2024-11-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

// ********** Paramters

// position&pose information source
// 0-mocap 1-slam 2-gazebo 3-others(todo)
int input_source;
// mocap frame type
// 0-Z_up 1-Y_up
int mocap_frame_type;
// rigid body name
std::string rigid_body_name;
// rate of node
float rate_hz;
// offset of position
Eigen::Vector3f pos_offset;
// offset of yaw
float yaw_offset;
// last timestamp
ros::Time last_timestamp;

// ********** Publisher&Messages&Paramters **********

// publisher for transfer information to fcu
ros::Publisher vision_pub;
// publisher for odometry visualization
ros::Publisher odom_pub;
// publisher for trajectory visualization
ros::Publisher trajectory_pub;
// 
bool _odom_valid = false;

// mocap position&pose  
Eigen::Vector3f mocap_position; 
Eigen::Quaternionf mocap_pose_quaternion;

// ********** Functions Declaration **********

// 
void send_to_fcu();
void pub_to_nodes();

// ********** Callback Functions **********

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);



int main(int argc, char** argv)
{
    ros::init(argc, argv, "pos_estimator");
    ros::NodeHandle nh("~");
    std::cout << "pos_estimator node started!" << std::endl;

    // load paramters from server
    nh.param<int>("input_source", input_source, 0);
    nh.param<int>("mocap_frame_type", mocap_frame_type, 0);
    nh.param<std::string>("rigid_body_name", rigid_body_name, "UAV");
    nh.param<float>("rate_hz", rate_hz, 20);
    nh.param<float>("offset_x", pos_offset[0], 0);
    nh.param<float>("offset_y", pos_offset[1], 0);
    nh.param<float>("offset_z", pos_offset[2], 0);
    nh.param<float>("offset_yaw", yaw_offset, 0);

    // subscribe topic from different position&pose sources
    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped> 
                                ("/vrpn_client_node/" + rigid_body_name + "/pose", 100, mocap_cb);

    ros::Subscriber slam_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                ("/slam/pose", 100, slam_cb);

    ros::Subscriber gazebo_sub = nh.subscribe<nav_msgs::Odometry>
                                ("/gazebo/ground_truth/UAV", 100, gazebo_cb);

    // publish position&pose information to FCU
    vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    // publish odometry for RVIZ visualization
    odom_pub = nh.advertise<nav_msgs::Odometry>("/visualization/odometry", 10);
    // publish trajectory for RVIZ visualization
    trajectory_pub = nh.advertise<nav_msgs::Path>("/visualization/trajectory", 10);

    // timer for print message show node is running correctly
    ros::Timer timer = nh.createTimer(ros::Duration(10.0), timer_cb);

    // class for communication with fcu through mavros [fcu->mavros->pos_estimator]
    // state_from_mavros _state_from_mavros;

    ros::Rate rate(rate_hz);

    // ********** main loop **********
    while (ros::ok()) {
        ros::spinOnce();

        // send position&pose information to fcu
        send_to_fcu();

        // publish state to other nodes
        pub_to_nodes();

        rate.sleep();
    }

    return 0;
}

void send_to_fcu()
{
    geometry_msgs::PoseStamped vision_pose;
    // mocap
    if (input_source == 0) {
        vision_pose.pose.position.x = mocap_position[0];
        vision_pose.pose.position.y = mocap_position[1];
        vision_pose.pose.position.z = mocap_position[2];

        vision_pose.pose.orientation.x = mocap_pose_quaternion.x();
        vision_pose.pose.orientation.y = mocap_pose_quaternion.y();
        vision_pose.pose.orientation.z = mocap_pose_quaternion.z();
        vision_pose.pose.orientation.w = mocap_pose_quaternion.w();
    }
    // 
    else if (input_source == 1) {

    }
    // 
    else if (input_source == 2) {

    }
    // 
    else if (input_source == 3) {

    }

    vision_pose.header.stamp = ros::Time::now();
    vision_pub.publish(vision_pose);
}

void pub_to_nodes()
{

}

// 
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // todo
    // 位置 -- optitrack系 到 ENU系
    // Frame convention 0: Z-up -- 1: Y-up (See the configuration in the motive software)
    if (mocap_frame_type == 0)
    {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        mocap_position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        mocap_position[0] = mocap_position[0] - pos_offset[0];
        mocap_position[1] = mocap_position[1] - pos_offset[1];
        mocap_position[2] = mocap_position[2] - pos_offset[2];
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        mocap_pose_quaternion = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    }
    else if (mocap_frame_type == 1)
    {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        mocap_position = Eigen::Vector3d(-msg->pose.position.x, msg->pose.position.z, msg->pose.position.y);
        mocap_position[0] = mocap_position[0] - pos_offset[0];
        mocap_position[1] = mocap_position[1] - pos_offset[1];
        mocap_position[2] = mocap_position[2] - pos_offset[2];
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        mocap_pose_quaternion = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y); //Y-up convention, switch the q2 & q3
    }

    // Transform the Quaternion to Euler Angles
    // Euler_mocap = quaternion_to_euler(q_mocap);

    _odom_valid= true;
    
    last_timestamp = msg->header.stamp;
}

