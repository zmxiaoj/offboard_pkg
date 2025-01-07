/**
 * @file offboard_mocap_trajectory.cpp
 * @brief This node controls the UAV to follow different trajectories using offboard mode with motion capture system.
 * 
 * Flight Process:
 * 1. Takeoff: The UAV first takes off to the specified height
 * 2. Trajectory Following: After takeoff, UAV follows the selected trajectory while maintaining height
 * 3. Landing: UAV lands when receiving land command
 * 
 * Trajectory Modes:
 * 1. Basic Trajectories (Without heading control):
 *    - hover: Maintain position
 *    - circle: Circle trajectory
 *    - rectangle: Rectangle trajectory
 *    - eight: Figure-8 trajectory
 * 
 * 2. Heading-controlled Trajectories:
 *    - circle_head: Circle with heading aligned to motion direction
 *    - rectangle_head: Rectangle with heading aligned to motion direction
 *    - eight_head: Figure-8 with heading aligned to motion direction
 * 
 * Height Control Strategy:
 * - During takeoff: Uses height parameter from configuration
 * - During trajectory: Maintains the height at trajectory switching point
 * - Trajectory switching: Preserves current height, only changes x-y motion
 * 
 * Parameters:
 * - /trajectory/flight_height: Initial takeoff height [0.5-2.0m]
 * - /trajectory/takeoff_position_x: Takeoff position x-coordinate
 * - /trajectory/takeoff_position_y: Takeoff position y-coordinate
 * - /trajectory/circle_radius: Radius for circular and eight-shaped trajectories [0.0-3.0m]
 * - /trajectory/rect_width: Width of the rectangular trajectory [max: 5.0m]
 * - /trajectory/rect_length: Height of the rectangular trajectory [max: 5.0m]
 * - /trajectory/eight_width: Width of figure-8 trajectory [max: 4.0m]
 * - /trajectory/eight_length: Length of figure-8 trajectory [max: 4.0m]
 * - /trajectory/flight_speed: Flight speed [0.1-1.0m/s]
 * - /trajectory/landing_speed: Landing descent speed [m/s]
 * 
 * Subscribers:
 * - /mavros/state: Current state of the UAV
 * - /mavros/local_position/pose: Current pose from motion capture
 * - /trajectory_cmd: Command to switch trajectory type
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
#include <mavros_msgs/ExtendedState.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <std_msgs/Bool.h>
#include <eigen3/Eigen/Eigen>

// Global variables
mavros_msgs::State current_state;
mavros_msgs::ExtendedState current_extended_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped initial_pose;
geometry_msgs::PoseStamped hover_setpoint_pose;
bool land_command_received = false;
bool initial_pose_received = false;
bool takeoff_completed = false;

enum class TrajectoryType {
    LANDING = -1,
    HOVER = 0,
    CIRCLE_HEAD = 1,
    CIRCLE = 2,
    RECTANGLE_HEAD = 3,
    RECTANGLE = 4,
    EIGHT_HEAD = 5,
    EIGHT = 6,
    CUSTOM = 7,
};
TrajectoryType trajectory_type = TrajectoryType::HOVER;

// Add these variables after the TrajectoryType definition
geometry_msgs::PoseStamped trajectory_start_pose;
ros::Time trajectory_switch_time;

// Trajectory Parameters
struct TrajectoryParams {
    float flight_height = 1.0;   // flight height(m)
    float takeoff_position_x = 0.0;
    float takeoff_position_y = 0.0;
    float circle_radius = 2.0;   // circle radius(m)
    float rect_width = 4.0;      // rectangle width(m)
    float rect_length = 3.0;     // rectangle length(m)
    float eight_width = 2.0;     // eight width(m)
    float eight_length = 2.0;    // eight length(m)
    float flight_speed = 0.5;    // flight speed(m/s)
    float landing_speed = 0.5;   // landing speed(m/s)
} params;

// Callback
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void trajectory_cmd_cb(const std_msgs::String::ConstPtr& msg);
void land_cmd_cb(const std_msgs::Bool::ConstPtr& msg);

// Util function
geometry_msgs::PoseStamped calculateTrajectorySetpoint();
geometry_msgs::PoseStamped transformPose2Setpoint(const geometry_msgs::Pose& relative_pose,
                                                  const geometry_msgs::PoseStamped& original_pose);
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
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>
            ("/mavros/extended_state", 10, extended_state_cb);
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

    float node_rate = 50.0;
    nh.param<float>("node_rate", node_rate, 50.0);
    // Load trajectory parameters
    nh.param<float>("flight_height", params.flight_height, 1.0);
    nh.param<float>("takeoff_position_x", params.takeoff_position_x, 0.0);
    nh.param<float>("takeoff_position_y", params.takeoff_position_y, 0.0);
    nh.param<float>("circle_radius", params.circle_radius, 2.0);
    nh.param<float>("rect_width", params.rect_width, 3.0);
    nh.param<float>("rect_length", params.rect_length, 4.0);
    nh.param<float>("eight_width", params.eight_width, 2.0);
    nh.param<float>("eight_length", params.eight_length, 2.0);
    nh.param<float>("flight_speed", params.flight_speed, 0.5);
    nh.param<float>("landing_speed", params.landing_speed, 0.5);

    // Clamp
    params.flight_height = std::min(std::max(params.flight_height, 0.5f), 2.0f);
    params.flight_speed = std::min(std::max(params.flight_speed, 0.1f), 1.0f);
    params.circle_radius = std::min(params.circle_radius, 3.0f);
    params.rect_width = std::min(params.rect_width, 5.0f);
    params.rect_length = std::min(params.rect_length, 5.0f);
    params.eight_width = std::min(params.eight_width, 4.0f);
    params.eight_length = std::min(params.eight_length, 4.0f);

    std::stringstream ss;
    ss  << "\n================ Flight Parameters ================\n"
        << "\033[36mBasic Parameters:\033[0m\n"
        << "  Height:         " << std::fixed << std::setprecision(2) 
        << params.flight_height << " m   [0.5 - 2.0]\n"
        << "  Speed:          " << params.flight_speed 
        << " m/s [0.1 - 1.0]\n"
        << "\033[36mTrajectory Parameters:\033[0m\n"
        << "  Circle Radius:  " << params.circle_radius 
        << " m   [max: 3.0]\n"
        << "  Rectangle:      " << params.rect_width << " x " 
        << params.rect_length << " m [max: 5.0]\n"
        << "  Eight-Shape:    " << params.eight_width << " x "
        << params.eight_length << " m [max: 4.0]\n"
        << "================================================\n";
    ROS_INFO_STREAM(ss.str());

    ros::Rate rate(node_rate);
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
    hover_pose.position.z = params.flight_height;
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
        if(trajectory_type == TrajectoryType::CUSTOM){
            // 这里为一个阻塞式的循环，启动自定义逻辑，直至运行完毕，运行完毕后，直接设定 trajectory_type 为 land
            ros::NodeHandle pnh;
            ros::param::set("/lih_custom_traj","begin");
            std::string custom_traj_state;
            while(ros::ok()){
                ros::param::get("/lih_custom_traj",custom_traj_state);
                if(custom_traj_state == "finish"){
                    break;
                }
                ROS_INFO_THROTTLE(1.0, "Waiting for custom traj to finish");
                rate.sleep();
            }
            ROS_INFO("Finish custom traj and to land");
        }else{
            geometry_msgs::PoseStamped next_setpoint_pose = calculateTrajectorySetpoint();
            local_pos_pub.publish(next_setpoint_pose);
            // Visualize trajectory
            publishPoseAndSetpoint(current_pose, next_setpoint_pose, current_pose_marker_pub, setpoint_pose_marker_pub);
            printPoseInfo(current_pose, next_setpoint_pose);
            rate.sleep();
        }
        ros::spinOnce();
    }

    geometry_msgs::PoseStamped landing_setpoint_pose = current_pose;
    float landed_height = initial_pose.pose.position.z;
    float landing_threshold = 0.1;
    float landing_timeout = 4.0 * ((current_pose.pose.position.z - landed_height) / params.landing_speed);
    last_request = ros::Time::now();
    ROS_INFO("\033[33m[Landing] Starting descent from height: %.2f m to %.2f m with speed: %.2f m/s\033[0m", 
            current_pose.pose.position.z, landed_height, params.landing_speed);

    // Descend slowly (about 0.5 m/s = (0.01 * 50)) until near ground
    while (ros::ok() && current_pose.pose.position.z > landed_height - landing_threshold) {
        if ((ros::Time::now() - last_request).toSec() > landing_timeout) {
            ROS_WARN("\033[31m[Landing] Timeout reached. Landing finished.\033[0m");
            break;
        }

        geometry_msgs::Pose landing_pose;
        landing_pose.position.x = 0.0;
        landing_pose.position.y = 0.0;
        landing_pose.position.z = - params.landing_speed / node_rate;
        landing_pose.orientation.w = 1.0;
        landing_pose.orientation.x = 0.0;
        landing_pose.orientation.y = 0.0;
        landing_pose.orientation.z = 0.0;

        landing_setpoint_pose = transformPose2Setpoint(landing_pose, landing_setpoint_pose);

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
                } 
                else {
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
    else if (cmd == "circle_head") {
        trajectory_type = TrajectoryType::CIRCLE_HEAD;
    }
    else if (cmd == "rectangle_head") {
        trajectory_type = TrajectoryType::RECTANGLE_HEAD;
    }
    else if (cmd == "eight_head") {
        trajectory_type = TrajectoryType::EIGHT_HEAD;
    }
    else if (cmd == "custom"){
        trajectory_type = TrajectoryType::CUSTOM;
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
    if (land_command_received) {
        ROS_INFO("\033[31mLand command received\033[0m");
        trajectory_type = TrajectoryType::LANDING;
    }
}

geometry_msgs::PoseStamped calculateTrajectorySetpoint() 
{
    geometry_msgs::PoseStamped setpoint_pose;
    geometry_msgs::Pose relative_pose;
    double t = (ros::Time::now() - trajectory_switch_time).toSec();
    
    // Determine the altitude to use based on the flight phase
    float target_height;
    if (!takeoff_completed) 
        // Use preset altitude during takeoff
        target_height = params.flight_height;
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
    else if (trajectory_type == TrajectoryType::CIRCLE_HEAD) {
        // heading to +X&-Y axis
        double omega = params.flight_speed / params.circle_radius;
        relative_pose.position.x = params.circle_radius * sin(omega * t);
        relative_pose.position.y = params.circle_radius * (cos(omega * t) - 1);
        relative_pose.position.z = 0; 
        
        double yaw = - omega * t;
        relative_pose.orientation.w = cos(yaw/2);
        relative_pose.orientation.x = 0;
        relative_pose.orientation.y = 0;
        relative_pose.orientation.z = sin(yaw/2);
    }
    else if (trajectory_type == TrajectoryType::CIRCLE) {
        double omega = params.flight_speed / params.circle_radius;
        relative_pose.position.x = params.circle_radius * sin(omega * t);
        relative_pose.position.y = params.circle_radius * (cos(omega * t) - 1);
        relative_pose.position.z = 0; 
        relative_pose.orientation.w = 1;
        relative_pose.orientation.x = 0;
        relative_pose.orientation.y = 0;
        relative_pose.orientation.z = 0;
    }
    else if (trajectory_type == TrajectoryType::RECTANGLE_HEAD) {
        double perimeter = 2 * (params.rect_width + params.rect_length);
        double s = fmod(params.flight_speed * t, perimeter);
        
        // Position calculation
        if (s < params.rect_width) {
            relative_pose.position.x = s;
            relative_pose.position.y = 0;
            relative_pose.position.z = 0; 
            // Moving along +X axisZ
            double yaw = 0;
            relative_pose.orientation.w = cos(yaw/2);
            relative_pose.orientation.x = 0;
            relative_pose.orientation.y = 0;
            relative_pose.orientation.z = sin(yaw/2);
        }
        else if (s < params.rect_width + params.rect_length) {
            relative_pose.position.x = params.rect_width;
            relative_pose.position.y = s - params.rect_width;
            relative_pose.position.z = 0; 
            // Moving along +Y axis
            double yaw = M_PI/2;
            relative_pose.orientation.w = cos(yaw/2);
            relative_pose.orientation.x = 0;
            relative_pose.orientation.y = 0;
            relative_pose.orientation.z = sin(yaw/2);
        }
        else if (s < 2 * params.rect_width + params.rect_length) {
            relative_pose.position.x = params.rect_width - (s - params.rect_width - params.rect_length);
            relative_pose.position.y = params.rect_length;
            relative_pose.position.z = 0; 
            // Moving along -X axis
            double yaw = M_PI;
            relative_pose.orientation.w = cos(yaw/2);
            relative_pose.orientation.x = 0;
            relative_pose.orientation.y = 0;
            relative_pose.orientation.z = sin(yaw/2);
        }
        else {
            relative_pose.position.x = 0;
            relative_pose.position.y = params.rect_length - (s - 2 * params.rect_width - params.rect_length);
            relative_pose.position.z = 0; 
            // Moving along -Y axis
            double yaw = -M_PI/2;
            relative_pose.orientation.w = cos(yaw/2);
            relative_pose.orientation.x = 0;
            relative_pose.orientation.y = 0;
            relative_pose.orientation.z = sin(yaw/2);
        }
        relative_pose.position.z = 0;
    }
    else if (trajectory_type == TrajectoryType::RECTANGLE) {
        double perimeter = 2 * (params.rect_width + params.rect_length);
        double s = fmod(params.flight_speed * t, perimeter);
        
        // Position calculation
        if (s < params.rect_width) {
            relative_pose.position.x = s;
            relative_pose.position.y = 0;
            relative_pose.position.z = 0; 
        }
        else if (s < params.rect_width + params.rect_length) {
            relative_pose.position.x = params.rect_width;
            relative_pose.position.y = s - params.rect_width;
            relative_pose.position.z = 0; 
        }
        else if (s < 2 * params.rect_width + params.rect_length) {
            relative_pose.position.x = params.rect_width - (s - params.rect_width - params.rect_length);
            relative_pose.position.y = params.rect_length;
            relative_pose.position.z = 0; 
        }
        else {
            relative_pose.position.x = 0;
            relative_pose.position.y = params.rect_length - (s - 2 * params.rect_width - params.rect_length);
            relative_pose.position.z = 0; 
        }
        relative_pose.orientation.w = 1;
        relative_pose.orientation.x = 0;
        relative_pose.orientation.y = 0;
        relative_pose.orientation.z = 0;
    }
    else if (trajectory_type == TrajectoryType::EIGHT_HEAD) {
        double omega = params.flight_speed / params.circle_radius;
        relative_pose.position.x = params.eight_width * sin(omega * t);
        relative_pose.position.y = params.eight_length * sin(omega * t * 2);
        relative_pose.position.z = 0;
        
        // Calculate tangent direction
        double dx = params.eight_width * omega * cos(omega * t);
        double dy = 2 * params.eight_length * omega * cos(omega * t * 2);
        double yaw = atan2(dy, dx); 
        // double yaw = std::fmod(atan2(dy, dx) - atan2(2 * params.eight_length, params.eight_width), 2 * M_PI);
        relative_pose.orientation.w = cos(yaw/2);
        relative_pose.orientation.x = 0;
        relative_pose.orientation.y = 0;
        relative_pose.orientation.z = sin(yaw/2);
    }
    else if (trajectory_type == TrajectoryType::EIGHT) {
        double omega = params.flight_speed / params.circle_radius;
        relative_pose.position.x = params.eight_width * sin(omega * t);
        relative_pose.position.y = params.eight_length * sin(omega * t * 2);
        relative_pose.position.z = 0;
        // double yaw = std::fmod(atan2(dy, dx) - atan2(2 * params.eight_length, params.eight_width), 2 * M_PI);
        relative_pose.orientation.w = 1;
        relative_pose.orientation.x = 0;
        relative_pose.orientation.y = 0;
        relative_pose.orientation.z = 0;
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
 * @param original_pose The initial pose used as the reference for the transformation.
 * @return geometry_msgs::PoseStamped The transformed setpoint pose.
 */
geometry_msgs::PoseStamped transformPose2Setpoint(const geometry_msgs::Pose& relative_pose,
                                                  const geometry_msgs::PoseStamped& original_pose) 
{
    geometry_msgs::PoseStamped setpoint;
    
    
    // Get initial pose quaternion[w x y z]
    Eigen::Quaterniond q_initial(
        original_pose.pose.orientation.w,
        original_pose.pose.orientation.x,
        original_pose.pose.orientation.y,
        original_pose.pose.orientation.z
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
    setpoint.pose.position.x = p_global.x() + original_pose.pose.position.x;
    setpoint.pose.position.y = p_global.y() + original_pose.pose.position.y;
    setpoint.pose.position.z = p_global.z() + original_pose.pose.position.z;
    
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
        case TrajectoryType::LANDING: 
            traj_type = "\033[31mLANDING\033[0m"; 
            break;
        case TrajectoryType::HOVER: 
            traj_type = "\033[32mHOVER\033[0m"; 
            break;
        case TrajectoryType::CIRCLE_HEAD: 
            traj_type = "\033[33mPREPARE CIRCLE\033[0m"; 
            break;
        case TrajectoryType::CIRCLE: 
            traj_type = "\033[36mCIRCLE\033[0m"; 
            break;
        case TrajectoryType::RECTANGLE_HEAD: 
            traj_type = "\033[33mPREPARE RECTANGLE\033[0m"; 
            break;
        case TrajectoryType::RECTANGLE: 
            traj_type = "\033[36mRECTANGLE\033[0m"; 
            break;
        case TrajectoryType::EIGHT_HEAD: 
            traj_type = "\033[33mPREPARE EIGHT\033[0m"; 
            break;
        case TrajectoryType::EIGHT: 
            traj_type = "\033[36mEIGHT\033[0m"; 
            break;
        default: 
            traj_type = "\033[31mUNKNOWN\033[0m"; 
            break;
    }
    
    // Format output using stringstream
    std::stringstream ss;
    ss  << "\n================== Flight Status ==================\n"
        << "Time: " << std::fixed << std::setprecision(1) 
        << ros::Time::now().toSec() << " sec\n"
        << "Mode: " << "\033[34m" << (current_state.mode) << "\033[0m"
        << " | Armed: " << (current_state.armed ? "\033[32mYES\033[0m" : "\033[31mNO\033[0m") 
        << " | Landed: " << (current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND ? 
                             "\033[32mYES\033[0m" : "\033[31mNO\033[0m") << "\n"
        << "Current Mission: " << traj_type << "\n"
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
