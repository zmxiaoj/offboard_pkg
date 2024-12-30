#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>	//发布的消息体对应的头文件，该消息体的类型为geometry_msgs::PoseStamped 本地位置
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/CommandBool.h>  //CommandBool服务的头文件，该服务的类型为mavros_msgs::CommandBool
#include <mavros_msgs/SetMode.h>  //SetMode服务的头文件，该服务的类型为mavros_msgs::SetMode
#include <mavros_msgs/State.h>  //订阅的消息体的头文件，该消息体的类型为mavros_msgs::State
#include <mavros_msgs/ExtendedState.h>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>

// Global Parameters
mavros_msgs::State current_state;
mavros_msgs::ExtendedState current_extended_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped initial_pose;
bool land_command_received = false;
bool initial_pose_received = false;

// Callback Function
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);
void land_cmd_cb(const std_msgs::Bool::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

// Util Function
geometry_msgs::PoseStamped transformPose2Setpoint(  const geometry_msgs::Pose& relative_pose,
                                                    const geometry_msgs::PoseStamped& initial_pose); 
void publishPoseAndSetpoint(const geometry_msgs::PoseStamped& current_pose, 
                            const geometry_msgs::PoseStamped& setpoint_pose,
                            const ros::Publisher& current_pub,
                            const ros::Publisher& setpoint_pub);
void printPoseInfo( const geometry_msgs::PoseStamped& current_pose, 
                    const geometry_msgs::PoseStamped& setpoint_pose);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_hover_node");
    ros::NodeHandle nh("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber land_command_sub = nh.subscribe<std_msgs::Bool>
            ("/land_cmd", 10, land_cmd_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher current_pose_marker_pub = nh.advertise<visualization_msgs::Marker>
            ("/visualization/current_pose", 10);
    ros::Publisher setpoint_pose_marker_pub = nh.advertise<visualization_msgs::Marker>
            ("/visualization/setpoint_pose", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    
    float takeoff_position_x = 0.0, takeoff_position_y = 0.0, takeoff_position_z = 1.0;

    nh.param<float>("takeoff_position_x", takeoff_position_x, 0.0);
    nh.param<float>("takeoff_position_y", takeoff_position_y, 0.0);
    nh.param<float>("takeoff_position_z", takeoff_position_z, 1.0);

    Eigen::Vector3f takeoff_position(takeoff_position_x, takeoff_position_y, takeoff_position_z);

    // Clamp the flight height to be between 0.0 and 3.0 meters
    takeoff_position.z() = std::min(std::max(takeoff_position.z(), 0.0f), 3.0f);

    ROS_INFO_STREAM("Takeoff Position(x y z): " << std::endl << takeoff_position.transpose());

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && (!current_state.connected || !initial_pose_received)) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::Pose takeoff_pose;

    takeoff_pose.position.x = takeoff_position.x();
    takeoff_pose.position.y = takeoff_position.y();
    takeoff_pose.position.z = takeoff_position.z();
    takeoff_pose.orientation.w = 1.0;
    takeoff_pose.orientation.x = 0.0;
    takeoff_pose.orientation.y = 0.0;
    takeoff_pose.orientation.z = 0.0;

    geometry_msgs::PoseStamped setpoint_pose = transformPose2Setpoint(takeoff_pose, initial_pose);
    publishPoseAndSetpoint(current_pose, setpoint_pose, current_pose_marker_pub, setpoint_pose_marker_pub);
    printPoseInfo(current_pose, setpoint_pose);

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(setpoint_pose);
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
            ROS_INFO("Land command received, performing controlled descent...");          
            break;
        }

        local_pos_pub.publish(setpoint_pose);
        publishPoseAndSetpoint(current_pose, setpoint_pose, current_pose_marker_pub, setpoint_pose_marker_pub);
        printPoseInfo(current_pose, setpoint_pose);

        ros::spinOnce();
        rate.sleep();
    }

    // Create landing setpoint at current X-Y position
    geometry_msgs::PoseStamped landing_setpoint_pose = current_pose;
    float landed_height = initial_pose.pose.position.z;
    float landing_speed = 0.3;
    float landing_threshold = 0.1;
    float landing_timeout = 2.0 * ((current_pose.pose.position.z - landed_height) / landing_speed);
    last_request = ros::Time::now();
    ROS_INFO("\033[33m[Landing] Starting descent from height: %.2f m with speed: %.2f m/s\033[0m", 
            current_pose.pose.position.z, landing_speed);

    // Descend slowly (about 0.5 m/s = (0.025 * 20)) until near ground
    while (ros::ok() && current_pose.pose.position.z > landed_height - landing_threshold) {
        if ((ros::Time::now() - last_request).toSec() > landing_timeout) {
            ROS_WARN("\033[31m[Landing] Timeout reached. Landing finished.\033[0m");
            break;
        }
        landing_setpoint_pose.pose.position.x = landing_setpoint_pose.pose.position.x;
        landing_setpoint_pose.pose.position.y = landing_setpoint_pose.pose.position.y;
        landing_setpoint_pose.pose.position.z = landing_setpoint_pose.pose.position.z - landing_speed / 20.0;
        landing_setpoint_pose.pose.orientation = landing_setpoint_pose.pose.orientation;
        landing_setpoint_pose.header.stamp = ros::Time::now();
        landing_setpoint_pose.header.frame_id = "world";

        local_pos_pub.publish(landing_setpoint_pose);
        publishPoseAndSetpoint(current_pose, landing_setpoint_pose, current_pose_marker_pub, setpoint_pose_marker_pub);
        printPoseInfo(current_pose, landing_setpoint_pose);
        
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("\033[32m[Landing] Reached target height. Landing complete.\033[0m");

    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }

    // Disarm
    arm_cmd.request.value = false;
    last_request = ros::Time::now();
    
    while (ros::ok() && current_state.armed) {
        if (ros::Time::now() - last_request > ros::Duration(1.0)) {
            if (arming_client.call(arm_cmd)) {
                if (arm_cmd.response.success) {
                    ROS_INFO("Vehicle disarmed");
                    break;
                } else {
                    ROS_WARN("Disarming failed, retrying...");
                }
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
    current_extended_state = *msg;
}

void land_cmd_cb(const std_msgs::Bool::ConstPtr& msg) 
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

/**
 * @brief Transforms a relative pose to a setpoint pose based on the initial pose.
 * 
 * @param relative_pose The relative pose to be transformed.
 * @param initial_pose The initial pose used as the reference for the transformation.
 * @return geometry_msgs::PoseStamped The transformed setpoint pose.
 */
geometry_msgs::PoseStamped transformPose2Setpoint(const geometry_msgs::Pose& relative_pose,
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
    
    setpoint.header.frame_id = "world";
    setpoint.header.stamp = ros::Time::now();

    return setpoint;
}

/**
 * @brief Publish the current pose and setpoint pose as visualization markers.
 * 
 * @param current_pose 
 * @param setpoint_pose 
 * @param current_pub 
 * @param setpoint_pub 
 */
void publishPoseAndSetpoint(const geometry_msgs::PoseStamped& current_pose, 
                            const geometry_msgs::PoseStamped& setpoint_pose,
                            const ros::Publisher& current_marker_pub,
                            const ros::Publisher& setpoint_marker_pub) 
{
    // Publish current pose
    visualization_msgs::Marker current_marker;
    current_marker.header.frame_id = "world";
    current_marker.header.stamp = ros::Time::now();
    current_marker.ns = "current_pose";
    current_marker.id = 0;
    current_marker.type = visualization_msgs::Marker::ARROW;
    current_marker.action = visualization_msgs::Marker::ADD;
    
    current_marker.pose = current_pose.pose;
    current_marker.scale.x = 0.3;  // 箭头长度
    current_marker.scale.y = 0.1;  // 箭头宽度
    current_marker.scale.z = 0.1;  // 箭头高度
    
    // Set the current pose marker to red
    current_marker.color.r = 1.0;
    current_marker.color.g = 0.0;
    current_marker.color.b = 0.0;
    current_marker.color.a = 1.0;
    
    current_marker_pub.publish(current_marker);

    // Publish setpoint pose
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
    
    // Set the setpoint marker to green
    setpoint_marker.color.r = 0.0;
    setpoint_marker.color.g = 1.0;
    setpoint_marker.color.b = 0.0;
    setpoint_marker.color.a = 1.0;
    
    setpoint_marker_pub.publish(setpoint_marker);
}

/**
 * @brief Prints the current pose and setpoint pose information.
 * 
 * @param current_pose The current pose of the vehicle.
 * @param setpoint_pose The target setpoint pose for the vehicle.
 */
void printPoseInfo( const geometry_msgs::PoseStamped& current_pose, 
                    const geometry_msgs::PoseStamped& setpoint_pose) {
    // Format output using stringstream
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