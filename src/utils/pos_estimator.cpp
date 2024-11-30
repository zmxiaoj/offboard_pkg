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
#include "mavros_msgs/State.h"
#include "mavros_msgs/ExtendedState.h"

// ********** Global Paramters **********
#define TIMEOUT_MAX 0.05
#define TRAJECTORY_WINDOW_MAX 1000
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
// 
std::string uav_name;
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
// history trajectory queue
std::vector<geometry_msgs::PoseStamped> history_trajectory_queue;

// mocap position&pose  
Eigen::Vector3f mocap_position; 
Eigen::Quaternionf mocap_pose_quaternion;
// slam position&pose
Eigen::Vector3f slam_position; 
Eigen::Quaternionf slam_pose_quaternion;
// gazebo position&pose
Eigen::Vector3f gazebo_position; 
Eigen::Quaternionf gazebo_pose_quaternion;

// drone state
bool drone_connected = false;
bool drone_armed = false;
std::string drone_mode;
bool drone_landed = false;
Eigen::Vector3f drone_position;
Eigen::Vector3f drone_velocity;
Eigen::Quaternionf drone_pose_quaternion;

// ********** Functions Declaration **********

void send_to_fcu();
void pub_state();
inline float get_time_diff(const ros::Time &begin_time);

// ********** Callback Functions **********

// position information
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void slam_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg);
void timer_cb(const ros::TimerEvent &e);
// state information
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void extened_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg);
void position_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);


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
    nh.param<std::string>("uav_name", uav_name, "uav0");

    // subscribe topic from different position&pose sources
    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped> 
                                ("/vrpn_client_node/" + rigid_body_name + "/pose", 100, mocap_cb);

    ros::Subscriber slam_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                ("/slam/pose", 100, slam_cb);

    ros::Subscriber gazebo_sub = nh.subscribe<nav_msgs::Odometry>
                                ("/gazebo/ground_truth/uav", 100, gazebo_cb);

    // subscribe drone state from mavros
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_cb);

    ros::Subscriber extened_state_sub = nh.subscribe<mavros_msgs::ExtendedState>
                                ("mavros/extended_state", 10, extened_state_cb);

    ros::Subscriber position_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                ("mavros/local_position/pose", 10, position_pose_cb);
    
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
                                ("mavros/local_position/velocity", 10, velocity_cb);

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

        // publish states
        pub_state();

        rate.sleep();
    }

    return 0;
}

// ********** Functions **********
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

        if (get_time_diff(last_timestamp) > TIMEOUT_MAX) {
            _odom_valid = false;
            ROS_ERROR("Mocap Timeout");
        }
        
    }
    // 1-slam
    else if (input_source == 1) {
        vision_pose.pose.position.x = slam_position[0];
        vision_pose.pose.position.y = slam_position[1];
        vision_pose.pose.position.z = slam_position[2];

        vision_pose.pose.orientation.x = slam_pose_quaternion.x();
        vision_pose.pose.orientation.y = slam_pose_quaternion.y();
        vision_pose.pose.orientation.z = slam_pose_quaternion.z();
        vision_pose.pose.orientation.w = slam_pose_quaternion.w();
    }
    // 2-gazebo
    else if (input_source == 2) {
        vision_pose.pose.position.x = gazebo_position[0];
        vision_pose.pose.position.y = gazebo_position[1];
        vision_pose.pose.position.z = gazebo_position[2];

        vision_pose.pose.orientation.x = gazebo_pose_quaternion.x();
        vision_pose.pose.orientation.y = gazebo_pose_quaternion.y();
        vision_pose.pose.orientation.z = gazebo_pose_quaternion.z();
        vision_pose.pose.orientation.w = gazebo_pose_quaternion.w();
    }
    // 3-others
    // else if (input_source == 3) {

    // }

    vision_pose.header.stamp = ros::Time::now();
    vision_pub.publish(vision_pose);
}

void pub_state()
{
    // out put drone state
    ROS_INFO("--------------------");
    ROS_INFO("Drone State: ");
    if (drone_connected) {
        ROS_INFO("Drone is connected.");
    }
    else {
        ROS_WARN("Drone is not connected.");
    }
    if (drone_armed) {
        ROS_INFO("Drone is armed.");
    }
    else {
        ROS_WARN("Drone is not armed.");
    }
    ROS_INFO("Mode: %s", drone_mode.c_str());
    if (drone_landed) {
        ROS_INFO("Drone is landed.");
    }
    else {
        ROS_INFO("Drone is not landed.");
    }
    ROS_INFO("--------------------");

    // publish drone odometry for RVIZ visualization
    nav_msgs::Odometry drone_odom;
    drone_odom.header.stamp = ros::Time::now();
    drone_odom.header.frame_id = "world";
    drone_odom.child_frame_id = "base_link";

    drone_odom.pose.pose.position.x = drone_position[0];
    drone_odom.pose.pose.position.y = drone_position[1];
    drone_odom.pose.pose.position.z = drone_position[2];

    // height should not be less than 0
    if (drone_odom.pose.pose.position.z <= 0) {
        drone_odom.pose.pose.position.z = 0.01;
    }

    drone_odom.pose.pose.orientation.x = drone_pose_quaternion.x();
    drone_odom.pose.pose.orientation.y = drone_pose_quaternion.y();
    drone_odom.pose.pose.orientation.z = drone_pose_quaternion.z();
    drone_odom.pose.pose.orientation.w = drone_pose_quaternion.w();

    drone_odom.twist.twist.linear.x = drone_velocity[0];
    drone_odom.twist.twist.linear.y = drone_velocity[1];
    drone_odom.twist.twist.linear.z = drone_velocity[2];

    odom_pub.publish(drone_odom);

    // publish drone trajeactory for RVIZ visualization
    geometry_msgs::PoseStamped drone_pose;
    drone_pose.header.stamp = ros::Time::now();
    drone_pose.header.frame_id = "world";
    drone_pose.pose.position.x = drone_position[0];
    drone_pose.pose.position.y = drone_position[1];
    drone_pose.pose.position.z = drone_position[2];

    drone_pose.pose.orientation.x = drone_pose_quaternion.x();
    drone_pose.pose.orientation.y = drone_pose_quaternion.y();
    drone_pose.pose.orientation.z = drone_pose_quaternion.z();
    drone_pose.pose.orientation.w = drone_pose_quaternion.w();

    history_trajectory_queue.push_back(drone_pose);
    if (history_trajectory_queue.size() > TRAJECTORY_WINDOW_MAX) {
        history_trajectory_queue.pop_back();
    }

    nav_msgs::Path drone_trajectory;
    drone_trajectory.header.stamp = ros::Time::now();
    drone_trajectory.header.frame_id = "world";
    drone_trajectory.poses = history_trajectory_queue;

    trajectory_pub.publish(drone_trajectory);

}

inline float get_time_diff(const ros::Time &begin_time)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin_time.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

// ********** Callback Functions ********** 
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // position&pose information from mocap to mavros(ENU)
    // Frame convention 0: Z-up -- 1: Y-up (See the configuration in the motive software)
    if (mocap_frame_type == 0) {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        mocap_position = Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        mocap_position[0] = mocap_position[0] - pos_offset[0];
        mocap_position[1] = mocap_position[1] - pos_offset[1];
        mocap_position[2] = mocap_position[2] - pos_offset[2];
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        mocap_pose_quaternion = Eigen::Quaternionf(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    }
    else if (mocap_frame_type == 1) {
        // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
        mocap_position = Eigen::Vector3f(-msg->pose.position.x, msg->pose.position.z, msg->pose.position.y);
        mocap_position[0] = mocap_position[0] - pos_offset[0];
        mocap_position[1] = mocap_position[1] - pos_offset[1];
        mocap_position[2] = mocap_position[2] - pos_offset[2];
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        mocap_pose_quaternion = Eigen::Quaternionf(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.z, msg->pose.orientation.y); //Y-up convention, switch the q2 & q3
    }

    // Transform the Quaternion to Euler Angles
    // Euler_mocap = quaternion_to_euler(q_mocap);

    _odom_valid= true;
    
    last_timestamp = msg->header.stamp;
}

void slam_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (msg->header.frame_id == "map_slam") {
        slam_position = Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        slam_pose_quaternion = Eigen::Quaternionf(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    }
    else {
        ROS_WARN("Wrong SLAM Frame ID");
    }

    _odom_valid = true;
}

void gazebo_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (msg->header.frame_id == "world") {
        gazebo_position = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        gazebo_pose_quaternion = Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    }
    else {
        ROS_WARN("Wrong Gazebo Frame ID");
    }
}

void timer_cb(const ros::TimerEvent &e)
{
    ROS_INFO("Program is running.");
    ROS_INFO("Current time: %f", e.current_real.toSec());
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    // get drone state
    drone_connected = msg->connected;
    drone_armed = msg->armed;
    drone_mode = msg->mode;
}

void extened_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    if (msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
        drone_landed = true;
    }
    else {
        drone_landed = false;
    }
}

void position_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    drone_position = Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    drone_pose_quaternion = Eigen::Quaternionf(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    drone_velocity = Eigen::Vector3f(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
}
