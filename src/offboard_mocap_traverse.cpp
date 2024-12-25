#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>

// Global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped initial_pose;
bool land_command_received = false;
bool initial_pose_received = false;

// Callback declarations
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void land_command_cb(const std_msgs::Bool::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

// Util function declarations
geometry_msgs::PoseStamped transformPose2Setpoint(  const geometry_msgs::Pose& relative_pose,
                                                    const geometry_msgs::PoseStamped& initial_pose); 
void publish_setpoint(const ros::Publisher& pub, const geometry_msgs::PoseStamped& pose, ros::Rate& rate, ros::Duration timeout);
void publishPoseAndSetpoint(const geometry_msgs::PoseStamped& current_pose, 
                          const geometry_msgs::PoseStamped& setpoint_pose,
                          const ros::Publisher& current_pub,
                          const ros::Publisher& setpoint_pub);
void printPoseInfo(const geometry_msgs::PoseStamped& current_pose, 
                  const geometry_msgs::PoseStamped& setpoint_pose);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_traverse_node");
    ros::NodeHandle nh("~");
 
    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber land_command_sub = nh.subscribe<std_msgs::Bool>
            ("/land_command", 10, land_command_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, pose_cb);
    
    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher current_pose_pub = nh.advertise<visualization_msgs::Marker>
            ("/visualization/current_pose", 10);
    ros::Publisher setpoint_pose_pub = nh.advertise<visualization_msgs::Marker>
            ("/visualization/setpoint_pose", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    float takeoff_x = 0.0, takeoff_y = 0.0, takeoff_z = 1.0,
           patrol_x = 1.0,  patrol_y = 1.0,  patrol_z = 1.0;
    float hover_time = 10.0;

    nh.param<float>("takeoff_x", takeoff_x, 0.0);
    nh.param<float>("takeoff_y", takeoff_y, 0.0);
    nh.param<float>("takeoff_z", takeoff_z, 1.0);
    nh.param<float>("patrol_x",  patrol_x,  1.0);
    nh.param<float>("patrol_y",  patrol_y,  1.0);
    nh.param<float>("patrol_z",  patrol_z,  1.0);
    nh.param<float>("hover_time", hover_time, 10.0);

    Eigen::Vector3f takeoff_position(takeoff_x, takeoff_y, takeoff_z),
                    patrol_position(patrol_x, patrol_y, patrol_z);
    
    // clamp flight height
    takeoff_position.z() = std::min(std::max(takeoff_position.z(), 0.0f), 3.0f);
    patrol_position.z() = std::min(std::max(patrol_position.z(), 0.0f), 3.0f);

    std::stringstream ss;
    ss << "\n============== Flight Mission ==============\n"
       << "Takeoff Position:\n"
       << "  x: " << std::fixed << std::setprecision(3) << std::setw(8) << takeoff_position.x()
       << "  y: " << std::setw(8) << takeoff_position.y()
       << "  z: " << std::setw(8) << takeoff_position.z() << "\n"
       << "\nPatrol Position:\n"
       << "  x: " << std::setw(8) << patrol_position.x()
       << "  y: " << std::setw(8) << patrol_position.y()
       << "  z: " << std::setw(8) << patrol_position.z() << "\n"
       << "\nHover Time: " << std::setw(8) << hover_time << " seconds\n"
       << "==========================================\n";
    
    ROS_INFO_STREAM(ss.str());
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
 
    // wait for FCU connection
    while (ros::ok() && (!current_state.connected || !initial_pose_received)) {
        ros::spinOnce();
        rate.sleep();
    }
 
    geometry_msgs::Pose takeoff_pose, patrol_pose;
    takeoff_pose.position.x = takeoff_position.x();
    takeoff_pose.position.y = takeoff_position.y();
    takeoff_pose.position.z = takeoff_position.z();
    takeoff_pose.orientation.w = 1.0;
    takeoff_pose.orientation.x = 0.0;
    takeoff_pose.orientation.y = 0.0;
    takeoff_pose.orientation.z = 0.0;
    patrol_pose.position.x  = patrol_position.x();
    patrol_pose.position.y  = patrol_position.y();
    patrol_pose.position.z  = patrol_position.z();
    patrol_pose.orientation.w = 1.0;
    patrol_pose.orientation.x = 0.0;
    patrol_pose.orientation.y = 0.0;
    patrol_pose.orientation.z = 0.0;

    geometry_msgs::PoseStamped takeoff_setpoint_pose = transformPose2Setpoint(takeoff_pose, initial_pose);
    geometry_msgs::PoseStamped patrol_setpoint_pose  = transformPose2Setpoint(patrol_pose, initial_pose);

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(takeoff_setpoint_pose);
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
        
        publish_setpoint(local_pos_pub, takeoff_setpoint_pose, rate, ros::Duration(hover_time));
        publishPoseAndSetpoint(current_pose, takeoff_setpoint_pose, current_pose_pub, setpoint_pose_pub);
        printPoseInfo(current_pose, takeoff_setpoint_pose);


        if (land_command_received) {
            ROS_INFO("Land command received, exiting offboard mode and landing...");
            break;
        }

        publish_setpoint(local_pos_pub, patrol_setpoint_pose, rate, ros::Duration(hover_time));
        publishPoseAndSetpoint(current_pose, patrol_setpoint_pose, current_pose_pub, setpoint_pose_pub);
        printPoseInfo(current_pose, patrol_setpoint_pose);

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

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
    if (!initial_pose_received) {
        initial_pose = current_pose;
        initial_pose_received = true;
        ROS_INFO("Initial pose recorded");
    }
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

geometry_msgs::PoseStamped transformPose2Setpoint(  const geometry_msgs::Pose& relative_pose,
                                                    const geometry_msgs::PoseStamped& initial_pose) 
{
    geometry_msgs::PoseStamped setpoint;
    
    // Get initial pose quaternion[w x y z]
    Eigen::Quaterniond q_initial(
        initial_pose.pose.orientation.w,
        initial_pose.pose.orientation.x,
        initial_pose.pose.orientation.y,
        initial_pose.pose.orientation.z
    );
    
    // Convert relative position to Eigen vector
    Eigen::Vector3d p_relative(
        relative_pose.position.x,
        relative_pose.position.y,
        relative_pose.position.z
    );
    
    // Rotate relative position by initial orientation
    Eigen::Vector3d p_global = q_initial * p_relative;
    
    // Add initial position
    // t_setpoint = R_initial * t_relative + t_initial
    setpoint.pose.position.x = p_global.x() + initial_pose.pose.position.x;
    setpoint.pose.position.y = p_global.y() + initial_pose.pose.position.y;
    setpoint.pose.position.z = p_global.z() + initial_pose.pose.position.z;
    
    // Combine rotations for orientation
    Eigen::Quaterniond q_relative(
        relative_pose.orientation.w,
        relative_pose.orientation.x,
        relative_pose.orientation.y,
        relative_pose.orientation.z
    );
    Eigen::Quaterniond q_global = q_initial * q_relative;
    
    // Set final orientation
    // R_setpoint = R_initial * R_relative
    setpoint.pose.orientation.w = q_global.w();
    setpoint.pose.orientation.x = q_global.x();
    setpoint.pose.orientation.y = q_global.y();
    setpoint.pose.orientation.z = q_global.z();
    
    return setpoint;
}

void publishPoseAndSetpoint(const geometry_msgs::PoseStamped& current_pose, 
                          const geometry_msgs::PoseStamped& setpoint_pose,
                          const ros::Publisher& current_pub,
                          const ros::Publisher& setpoint_pub) 
{
    // Current pose marker
    visualization_msgs::Marker current_marker;
    current_marker.header.frame_id = "world";
    current_marker.header.stamp = ros::Time::now();
    current_marker.ns = "current_pose";
    current_marker.id = 0;
    current_marker.type = visualization_msgs::Marker::ARROW;
    current_marker.action = visualization_msgs::Marker::ADD;
    
    current_marker.pose = current_pose.pose;
    current_marker.scale.x = 0.3;
    current_marker.scale.y = 0.1;
    current_marker.scale.z = 0.1;
    
    current_marker.color.r = 1.0;
    current_marker.color.g = 0.0;
    current_marker.color.b = 0.0;
    current_marker.color.a = 1.0;
    
    current_pub.publish(current_marker);

    // Setpoint marker
    visualization_msgs::Marker setpoint_marker;
    setpoint_marker.header.frame_id = "world";
    setpoint_marker.header.stamp = ros::Time::now();
    setpoint_marker.ns = "setpoint";
    setpoint_marker.id = 1;
    setpoint_marker.type = visualization_msgs::Marker::ARROW;
    setpoint_marker.action = visualization_msgs::Marker::ADD;
    
    setpoint_marker.pose = setpoint_pose.pose;
    setpoint_marker.scale.x = 0.3;
    setpoint_marker.scale.y = 0.1;
    setpoint_marker.scale.z = 0.1;
    
    setpoint_marker.color.r = 0.0;
    setpoint_marker.color.g = 1.0;
    setpoint_marker.color.b = 0.0;
    setpoint_marker.color.a = 1.0;
    
    setpoint_pub.publish(setpoint_marker);
}

void printPoseInfo(const geometry_msgs::PoseStamped& current_pose, 
                  const geometry_msgs::PoseStamped& setpoint_pose) 
{
    std::stringstream ss;
    ss << "\n==================== Pose Info ====================\n"
       << "Current Pose:\n"
       << "  Position  (x, y, z):    (" 
       << std::fixed << std::setprecision(3) 
       << current_pose.pose.position.x << ", "
       << current_pose.pose.position.y << ", "
       << current_pose.pose.position.z << ")\n"
       << "  Orientation (w,x,y,z):  ("
       << current_pose.pose.orientation.w << ", "
       << current_pose.pose.orientation.x << ", "
       << current_pose.pose.orientation.y << ", "
       << current_pose.pose.orientation.z << ")\n"
       << "\nSetpoint:\n"
       << "  Position  (x, y, z):    ("
       << setpoint_pose.pose.position.x << ", "
       << setpoint_pose.pose.position.y << ", "
       << setpoint_pose.pose.position.z << ")\n"
       << "  Orientation (w,x,y,z):  ("
       << setpoint_pose.pose.orientation.w << ", "
       << setpoint_pose.pose.orientation.x << ", "
       << setpoint_pose.pose.orientation.y << ", "
       << setpoint_pose.pose.orientation.z << ")\n"
       << "================================================\n";
    
    ROS_INFO_STREAM(ss.str());
}
