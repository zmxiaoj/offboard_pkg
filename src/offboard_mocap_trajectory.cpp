/**
 * @file offboard_mocap_trajectory.cpp
 * @brief This node controls the UAV to follow different trajectories using offboard mode with motion capture system.
 * 
 * Flight Process:
 * 1. Takeoff: The UAV first takes off to the specified height
 * 2. Trajectory Following: After takeoff, UAV follows the selected trajectory while maintaining height
 * 3. Landing: UAV lands when receiving land command
 * 
 * Height Control Strategy:
 * - During takeoff: Uses height parameter from configuration
 * - During trajectory: Maintains the height at trajectory switching point
 * - Trajectory switching: Preserves current height, only changes x-y motion
 * 
 * Parameters:
 * - /trajectory/height: Initial takeoff height [0.5-2.0m]
 * - /trajectory/takeoff_position_x: Takeoff position x-coordinate
 * - /trajectory/takeoff_position_y: Takeoff position y-coordinate
 * - /trajectory/radius: Radius for circular and eight-shaped trajectories [0.0-3.0m]
 * - /trajectory/rect_width: Width of the rectangular trajectory [max: 5.0m]
 * - /trajectory/rect_height: Height of the rectangular trajectory [max: 5.0m]
 * - /trajectory/speed: Flight speed [0.1-1.0m/s]
 * 
 * Subscribers:
 * - /mavros/state: Current state of the UAV
 * - /mavros/local_position/pose: Current pose from motion capture
 * - /trajectory_cmd: Command to switch trajectory type (hover/circle/rectangle/eight)
 * - /land_cmd: Command to trigger landing
 * 
 * Publishers:
 * - /mavros/setpoint_position/local: Setpoint position for the UAV
 * - /visualization/current_pose: Visualization marker for current pose
 * - /visualization/setpoint_pose: Visualization marker for target pose
 * 
 * Service Clients:
 * - /mavros/cmd/arming: Service to arm the UAV
 * - /mavros/set_mode: Service to set the mode of the UAV
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>

// Global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped initial_pose;
geometry_msgs::PoseStamped hover_setpoint_pose;
bool land_command_received = false;
bool initial_pose_received = false;
bool takeoff_completed = false;

enum class TrajectoryType {
    HOVER = 0,
    CIRCLE = 1,
    RECTANGLE = 2,
    EIGHT = 3
};
TrajectoryType trajectory_type = TrajectoryType::HOVER;

// Add these variables after the TrajectoryType definition
geometry_msgs::PoseStamped trajectory_start_pose;
ros::Time trajectory_switch_time;

// Trajectory Parameters
struct TrajectoryParams {
    float height = 1.0;          // flight height(m)
    float takeoff_position_x = 0.0;
    float takeoff_position_y = 0.0;
    float radius = 1.0;          // circle/eight radius(m)
    float rect_width = 4.0;      // rectangle width(m)
    float rect_height = 3.0;     // rectangle height(m)
    float speed = 0.5;           // flight speed(m/s)
} params;

// Callback
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void trajectory_cmd_cb(const std_msgs::String::ConstPtr& msg);
void land_cmd_cb(const std_msgs::Bool::ConstPtr& msg);

// Util function
geometry_msgs::PoseStamped calculateTrajectorySetpoint(const ros::Time& start_time);
geometry_msgs::PoseStamped transformPose2Setpoint(const geometry_msgs::Pose& relative_pose,
                                                  const geometry_msgs::PoseStamped& initial_pose);
void publishPoseAndSetpoint(const geometry_msgs::PoseStamped& current_pose, 
                            const geometry_msgs::PoseStamped& setpoint_pose,
                            const ros::Publisher& current_pub,
                            const ros::Publisher& setpoint_pub);
void printPoseInfo(const geometry_msgs::PoseStamped& current_pose, 
                   const geometry_msgs::PoseStamped& setpoint_pose);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_trajectory_node");
    ros::NodeHandle nh("~");
    
    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber land_cmd_sub = nh.subscribe<std_msgs::Bool>
            ("/land_cmd", 10, land_cmd_cb);
    ros::Subscriber trajectory_cmd_sub = nh.subscribe<std_msgs::String>
            ("/trajectory_cmd", 10, trajectory_cmd_cb);
    
    
    // Publishers
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher current_pose_marker_pub = nh.advertise<visualization_msgs::Marker>
            ("/visualization/current_pose", 10);
    ros::Publisher setpoint_pose_marker_pub = nh.advertise<visualization_msgs::Marker>
            ("/visualization/setpoint_pose", 10);
    
    // Service clients
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    // Load parameters
    nh.param<float>("height", params.height, 1.0);
    nh.param<float>("takeoff_position_x", params.takeoff_position_x, 0.0);
    nh.param<float>("takeoff_position_y", params.takeoff_position_y, 0.0);
    nh.param<float>("radius", params.radius, 1.0);
    nh.param<float>("rect_width", params.rect_width, 2.0);
    nh.param<float>("rect_height", params.rect_height, 2.0);
    nh.param<float>("speed", params.speed, 0.5);

    // Clamp
    params.height = std::min(std::max(params.height, 0.5f), 2.0f);
    params.speed = std::min(std::max(params.speed, 0.1f), 1.0f);
    params.radius = std::min(params.radius, 3.0f);
    params.rect_width = std::min(params.rect_width, 5.0f);
    params.rect_height = std::min(params.rect_height, 5.0f);

    std::stringstream ss;
    ss  << "\n================ Flight Parameters ================\n"
        << "Height:     " << std::fixed << std::setprecision(2) 
        << params.height << " m   [0.5 - 2.0]\n"
        << "Speed:      " << params.speed << " m/s [0.1 - 1.0]\n"
        << "Radius:     " << params.radius << " m   [0.0 - 3.0]\n"
        << "Rectangle:  " << params.rect_width << " x " 
        << params.rect_height << " m [max: 5.0]\n"
        << "================================================\n";
    ROS_INFO_STREAM(ss.str());

    ros::Rate rate(20.0);
    ros::Time trajectory_start_time = ros::Time::now();

    // Wait for FCU connection
    while (ros::ok() && (!current_state.connected || !initial_pose_received)) {
        ros::spinOnce();
        rate.sleep();
    }

    // Takeoff 
    geometry_msgs::Pose hover_pose;
    hover_pose.position.x = params.takeoff_position_x;
    hover_pose.position.y = params.takeoff_position_y;
    hover_pose.position.z = params.height;
    hover_pose.orientation.w = 1.0;
    hover_pose.orientation.x = 0.0;
    hover_pose.orientation.y = 0.0;
    hover_pose.orientation.z = 0.0;

    hover_setpoint_pose = transformPose2Setpoint(hover_pose, initial_pose);
    publishPoseAndSetpoint(current_pose, hover_setpoint_pose, current_pose_marker_pub, setpoint_pose_marker_pub);
    printPoseInfo(current_pose, hover_setpoint_pose);

    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(hover_setpoint_pose);
        ros::spinOnce();
        rate.sleep();
    }
    // Use the takeoff position as the initial trajectory position
    takeoff_completed = true;  
    trajectory_start_pose = hover_setpoint_pose; 

    // Switch to OFFBOARD mode and arm
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

    // Main loop
    while (ros::ok()) {
        if (land_command_received) {
            ROS_INFO("Land command received, exiting offboard mode and landing...");
            break;
        }

        // Calculate and publish position setpoint
        geometry_msgs::PoseStamped setpoint_pose = calculateTrajectorySetpoint(trajectory_start_time);
        local_pos_pub.publish(setpoint_pose);
        // Visualize trajectory
        publishPoseAndSetpoint(current_pose, setpoint_pose, current_pose_marker_pub, setpoint_pose_marker_pub);
        printPoseInfo(current_pose, setpoint_pose);

        ros::spinOnce();
        rate.sleep();
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

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    current_pose = *msg;
    if (!initial_pose_received) {
        initial_pose = current_pose;
        initial_pose_received = true;
        ROS_INFO("Initial pose recorded");
    }
}

void trajectory_cmd_cb(const std_msgs::String::ConstPtr& msg) 
{
    std::string cmd = msg->data;
    TrajectoryType previous_type = trajectory_type;
    
    if (cmd == "hover") {
        trajectory_type = TrajectoryType::HOVER;
    } 
    else if (cmd == "circle") {
        trajectory_type = TrajectoryType::CIRCLE;
    } 
    else if (cmd == "rectangle") {
        trajectory_type = TrajectoryType::RECTANGLE;
    } 
    else if (cmd == "eight") {
        trajectory_type = TrajectoryType::EIGHT;
    }

    if (previous_type != trajectory_type) {
        trajectory_start_pose = current_pose;  // Use current pose as new reference
        trajectory_switch_time = ros::Time::now();
        ROS_INFO("\033[33mSwitching to trajectory: %s\033[0m", cmd.c_str());
    }
}

void land_cmd_cb(const std_msgs::Bool::ConstPtr& msg) 
{
    land_command_received = msg->data;
}

geometry_msgs::PoseStamped calculateTrajectorySetpoint(const ros::Time& start_time) 
{
    geometry_msgs::PoseStamped setpoint_pose;
    geometry_msgs::Pose relative_pose;
    double t = (ros::Time::now() - trajectory_switch_time).toSec();
    
    // Determine the altitude to use based on the flight phase
    float target_height;
    if (!takeoff_completed) 
        // Use preset altitude during takeoff
        target_height = params.height;
    else 
        // Maintain the current altitude during the trajectory flight phase
        target_height = trajectory_start_pose.pose.position.z;
    
    if (trajectory_type == TrajectoryType::HOVER) {
        relative_pose.position.x = 0;
        relative_pose.position.y = 0;
        relative_pose.position.z = 0;
        relative_pose.orientation.w = 1;
        relative_pose.orientation.x = 0;
        relative_pose.orientation.y = 0;
        relative_pose.orientation.z = 0;
    }
    else if (trajectory_type == TrajectoryType::CIRCLE) {
        double omega = params.speed / params.radius;
        relative_pose.position.x = params.radius * cos(omega * t);
        relative_pose.position.y = params.radius * sin(omega * t);
        relative_pose.position.z = 0; 
        
        double yaw = omega * t + M_PI/2;
        relative_pose.orientation.w = cos(yaw/2);
        relative_pose.orientation.x = 0;
        relative_pose.orientation.y = 0;
        relative_pose.orientation.z = sin(yaw/2);
    }
    else if (trajectory_type == TrajectoryType::RECTANGLE) {
        double perimeter = 2 * (params.rect_width + params.rect_height);
        double s = fmod(params.speed * t, perimeter);
        
        // Position calculation
        if (s < params.rect_width) {
            relative_pose.position.x = s;
            relative_pose.position.y = 0;
            // Moving along +X axis
            double yaw = 0;
            relative_pose.orientation.w = cos(yaw/2);
            relative_pose.orientation.x = 0;
            relative_pose.orientation.y = 0;
            relative_pose.orientation.z = sin(yaw/2);
        }
        else if (s < params.rect_width + params.rect_height) {
            relative_pose.position.x = params.rect_width;
            relative_pose.position.y = s - params.rect_width;
            // Moving along +Y axis
            double yaw = M_PI/2;
            relative_pose.orientation.w = cos(yaw/2);
            relative_pose.orientation.x = 0;
            relative_pose.orientation.y = 0;
            relative_pose.orientation.z = sin(yaw/2);
        }
        else if (s < 2 * params.rect_width + params.rect_height) {
            relative_pose.position.x = params.rect_width - (s - params.rect_width - params.rect_height);
            relative_pose.position.y = params.rect_height;
            // Moving along -X axis
            double yaw = M_PI;
            relative_pose.orientation.w = cos(yaw/2);
            relative_pose.orientation.x = 0;
            relative_pose.orientation.y = 0;
            relative_pose.orientation.z = sin(yaw/2);
        }
        else {
            relative_pose.position.x = 0;
            relative_pose.position.y = params.rect_height - (s - 2 * params.rect_width - params.rect_height);
            // Moving along -Y axis
            double yaw = -M_PI/2;
            relative_pose.orientation.w = cos(yaw/2);
            relative_pose.orientation.x = 0;
            relative_pose.orientation.y = 0;
            relative_pose.orientation.z = sin(yaw/2);
        }
        relative_pose.position.z = 0;
    }
    else if (trajectory_type == TrajectoryType::EIGHT) {
        double omega = params.speed / params.radius;
        relative_pose.position.x = params.radius * sin(omega * t);
        relative_pose.position.y = params.radius * sin(omega * t * 2);
        relative_pose.position.z = 0;
        
        // Calculate tangent direction
        double dx = params.radius * omega * cos(omega * t);
        double dy = 2 * params.radius * omega * cos(omega * t * 2);
        double yaw = atan2(dy, dx);
        relative_pose.orientation.w = cos(yaw/2);
        relative_pose.orientation.x = 0;
        relative_pose.orientation.y = 0;
        relative_pose.orientation.z = sin(yaw/2);
    }
        
    setpoint_pose = transformPose2Setpoint(relative_pose, trajectory_start_pose);
    
    // Set target altitude according to flight phase
    if (!takeoff_completed) {
        setpoint_pose.pose.position.z = target_height;
    }
    
    return setpoint_pose;
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
                            const ros::Publisher& current_pub,
                            const ros::Publisher& setpoint_pub) 
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
    
    current_pub.publish(current_marker);

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
    
    setpoint_pub.publish(setpoint_marker);
}

/**
 * @brief Prints the current pose and setpoint pose information.
 * 
 * @param current_pose The current pose of the vehicle.
 * @param setpoint_pose The target setpoint pose for the vehicle.
 */
void printPoseInfo(const geometry_msgs::PoseStamped& current_pose, 
                   const geometry_msgs::PoseStamped& setpoint_pose) 
{
    std::string traj_type;
    switch(trajectory_type) {
        case TrajectoryType::HOVER: traj_type = "HOVER"; break;
        case TrajectoryType::CIRCLE: traj_type = "CIRCLE"; break;
        case TrajectoryType::RECTANGLE: traj_type = "RECTANGLE"; break;
        case TrajectoryType::EIGHT: traj_type = "EIGHT"; break;
        default: traj_type = "UNKNOWN"; break;
    }
    
    // Format output using stringstream
    std::stringstream ss;
    ss  << "\n================== Flight Status ==================\n"
        << "Time: " << std::fixed << std::setprecision(1) 
        << ros::Time::now().toSec() << " sec\n"
        << "Mode: " << "\033[34m" << (current_state.mode) << "\033[0m"
        << " | Armed: " << (current_state.armed ? "\033[32mYES\033[0m" : "\033[31mNO\033[0m")  << "\n"
        << "Current Mission: " << "\033[33m" << traj_type << "\033[0m" << "\n"
        << "--------------------------------------------------\n"
        << "Current Position:\n"
        << std::setprecision(3) 
        << "x: " << current_pose.pose.position.x 
        << " | y: " << current_pose.pose.position.y 
        << " | z: " << current_pose.pose.position.z << "\n"
        << "Current Orientation:\n" 
        << "w: " << current_pose.pose.orientation.w 
        << " | x: " << current_pose.pose.orientation.x 
        << " | y: " << current_pose.pose.orientation.y 
        << " | z: " << current_pose.pose.orientation.z << "\n"
        << "--------------------------------------------------\n"
        << "Target Position:\n"
        << "x: " << setpoint_pose.pose.position.x 
        << " | y: " << setpoint_pose.pose.position.y 
        << " | z: " << setpoint_pose.pose.position.z << "\n"
        << "Target Orientation:\n"
        << "w: " << setpoint_pose.pose.orientation.w 
        << " | x: " << setpoint_pose.pose.orientation.x 
        << " | y: " << setpoint_pose.pose.orientation.y 
        << " | z: " << setpoint_pose.pose.orientation.z << "\n"
        << "--------------------------------------------------\n"
        << "Position Error(current2setpoint):\n"
        << "x: " << std::setprecision(3) 
        << setpoint_pose.pose.position.x - current_pose.pose.position.x
        << " | y: " << setpoint_pose.pose.position.y - current_pose.pose.position.y
        << " | z: " << setpoint_pose.pose.position.z - current_pose.pose.position.z
        << "\n================================================\n";
    ROS_INFO_STREAM(ss.str());
}
