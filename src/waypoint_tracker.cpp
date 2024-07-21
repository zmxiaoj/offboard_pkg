#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>	



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

std::string flight_state = "follow_waypoint";

void flight_command_cb(const std_msgs::String::ConstPtr& msg) 
{
    std::string flight_command = msg->data.c_str();
    ROS_INFO("Received command: [%s]", flight_command);

    if (flight_command == "circle") {
        flight_state = "forward_to_circle";
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position");
    ros::NodeHandle nh("~");
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber flight_command_sub = nh.subscribe<std_msgs::String>
            ("/flight_command", 10, flight_command_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    std::string rosbag_path;
    nh.param<std::string>("rosbag_path", rosbag_path, "/home/comb/0720.bag");
    ROS_INFO("rosbag path: %s", rosbag_path.c_str());
    // 从参数服务器读入current_position_x和current_position_y
    double current_position_x = 0.0, current_position_y = 0.0;
    nh.param("current_position_x", current_position_x, 0.23);
    nh.param("current_position_y", current_position_y, 8.10);

    std::vector<geometry_msgs::PoseStamped> waypoints, waypoints_circle;
    size_t current_waypoint = 0, current_waypoint_circle = 0;

    //ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
           // ("mavros/setpoint_velocity/cmd_vel", 10);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0), rate_waypoint(120.0);

    /**
     * @brief 从rosbag中的topic /vrpn_client_node/A1_12/pose读取waypoints保存到Vetor
     */
    rosbag::Bag bag;
    bag.open(rosbag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back("/vrpn_client_node/A1_12/pose");
    // topics.push_back("/mavros/vision_pose/pose");

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance& m : view) {
        geometry_msgs::PoseStamped::ConstPtr p = m.instantiate<geometry_msgs::PoseStamped>();
        if (p != NULL && p->pose.position.z > 0.4 && p->pose.position.x < 14.2)
            waypoints.push_back(*p);
        else if (p != NULL && p->pose.position.z > 0.4 && p->pose.position.x >= 15.2)
            waypoints_circle.push_back(*p);
    }
    bag.close();
    ROS_INFO("Read waypoints from ROSbag");
    ROS_INFO("Number of waypoints: %lu", waypoints.size());
    ROS_INFO("Number of waypoints_circle: %lu", waypoints_circle.size());
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU Connected");

    // 根据waypoints中第一个点的高度设置起飞点并发送100个
    geometry_msgs::PoseStamped takeoff_pose;
    // takeoff_pose.pose.position.z = waypoints[current_waypoint].pose.position.z;
    // offset
    double offset_x = current_position_x - waypoints[current_waypoint].pose.position.x;
    double offset_y = current_position_y - waypoints[current_waypoint].pose.position.y;

    ROS_INFO("offset_x: %.3f, offset_y: %.3f", offset_x, offset_y);

    takeoff_pose.pose.position.x = waypoints[current_waypoint].pose.position.x + offset_x;
    takeoff_pose.pose.position.y = waypoints[current_waypoint].pose.position.y + offset_y;
    takeoff_pose.pose.position.z = waypoints[current_waypoint].pose.position.z;

    // orientation
    takeoff_pose.pose.orientation.w = waypoints[current_waypoint].pose.orientation.w;
    takeoff_pose.pose.orientation.x = waypoints[current_waypoint].pose.orientation.x;
    takeoff_pose.pose.orientation.y = waypoints[current_waypoint].pose.orientation.y;
    takeoff_pose.pose.orientation.z = waypoints[current_waypoint].pose.orientation.z;
    
    current_waypoint += 1;

    // send a few setpoints before starting
    ROS_INFO("Start send setpoints");
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(takeoff_pose);
        ros::spinOnce();
        rate.sleep();
    }
    // 输出takeoff_pose
    ROS_INFO("Takeoff pose: [%.3f, %.3f, %.3f]", takeoff_pose.pose.position.x, takeoff_pose.pose.position.y, takeoff_pose.pose.position.z);
    

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    // convert to offboard&arm
    ROS_INFO("Convert to OFFBOARD");
    while(ros::ok() ){
        if(!current_state.armed){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
        }
        if(current_state.mode != "OFFBOARD"){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
        } 
        local_pos_pub.publish(takeoff_pose);
        if( (ros::Time::now() - last_request > ros::Duration(10.0))) break;
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Takeoff");

    // follow waypoints
    ROS_INFO("Start follow waypoints");
    // pub waypoint per 10 Hz
    int info_frequency = 10;
    int wiat_frequency_cnt = 0; 
    geometry_msgs::PoseStamped waypoint_pose, last_waypoint_pose;
    while (ros::ok() && flight_state == "follow_waypoint") {
        
        if (current_waypoint < waypoints.size()) {
            waypoint_pose.pose.position.x = waypoints[current_waypoint].pose.position.x + offset_x;
            waypoint_pose.pose.position.y = waypoints[current_waypoint].pose.position.y + offset_y;
            waypoint_pose.pose.position.z = waypoints[current_waypoint].pose.position.z;

            // orientation
            waypoint_pose.pose.orientation.w = waypoints[current_waypoint].pose.orientation.w;
            waypoint_pose.pose.orientation.x = waypoints[current_waypoint].pose.orientation.x;
            waypoint_pose.pose.orientation.y = waypoints[current_waypoint].pose.orientation.y;
            waypoint_pose.pose.orientation.z = waypoints[current_waypoint].pose.orientation.z;
            
            last_waypoint_pose = waypoint_pose;
            local_pos_pub.publish(waypoint_pose);
            if (current_waypoint % info_frequency == 0)
                ROS_INFO("Pub waypoint: [%.3f, %.3f, %.3f]", waypoint_pose.pose.position.x, waypoint_pose.pose.position.y, waypoint_pose.pose.position.z);
            current_waypoint++;
        }
        else {
            local_pos_pub.publish(last_waypoint_pose);
            if (wiat_frequency_cnt % info_frequency == 0)
                ROS_INFO("Reach the last waypoint: [%.3f, %.3f, %.3f]", last_waypoint_pose.pose.position.x, last_waypoint_pose.pose.position.y, last_waypoint_pose.pose.position.z);
            wiat_frequency_cnt += 1;
        }
        ros::spinOnce();
        rate_waypoint.sleep();
    }
    ROS_INFO("Finished follow waypoints");

    geometry_msgs::PoseStamped circle_waypoint_pose;
    while (ros::ok() && flight_state == "forward_to_circle") {
        
        if (current_waypoint_circle < waypoints_circle.size()) {
            circle_waypoint_pose.pose.position.x = waypoints_circle[current_waypoint_circle].pose.position.x + offset_x;
            circle_waypoint_pose.pose.position.y = waypoints_circle[current_waypoint_circle].pose.position.y + offset_y;
            circle_waypoint_pose.pose.position.z = waypoints_circle[current_waypoint_circle].pose.position.z;

            // orientation
            circle_waypoint_pose.pose.orientation.w = waypoints_circle[current_waypoint_circle].pose.orientation.w;
            circle_waypoint_pose.pose.orientation.x = waypoints_circle[current_waypoint_circle].pose.orientation.x;
            circle_waypoint_pose.pose.orientation.y = waypoints_circle[current_waypoint_circle].pose.orientation.y;
            circle_waypoint_pose.pose.orientation.z = waypoints_circle[current_waypoint_circle].pose.orientation.z;

            local_pos_pub.publish(circle_waypoint_pose);
            if (current_waypoint_circle % info_frequency == 0)
                ROS_INFO("Pub waypoint: [%.3f, %.3f, %.3f]", circle_waypoint_pose.pose.position.x, circle_waypoint_pose.pose.position.y, circle_waypoint_pose.pose.position.z);
            current_waypoint_circle++;
        }
        else {
            ROS_INFO("Finish forward to circle");
            break;
        }
    
        ros::spinOnce();
        rate_waypoint.sleep();
    }
    // land
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }
    ROS_INFO("Finish Mission");

    return 0;
}
