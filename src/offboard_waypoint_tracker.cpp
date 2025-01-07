#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>	


mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_waypoint_tracker_node");
    ros::NodeHandle nh("~");
 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    std::string rosbag_path;
    nh.param<std::string>("rosbag_path", rosbag_path, "/home/zmxj/code/Datasets/0713.bag");
    ROS_INFO("rosbag path: %s", rosbag_path.c_str());

    std::vector<geometry_msgs::PoseStamped> waypoints;
    size_t current_waypoint = 0;

    //ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>
           // ("mavros/setpoint_velocity/cmd_vel", 10);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0), rate_waypoint(100.0);

    /**
     * @brief 从rosbag中的topic /vrpn_client_node/P450/pose读取waypoints保存到Vetor
     */
    rosbag::Bag bag;
    bag.open(rosbag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back("/vrpn_client_node/P450/pose");
    // topics.push_back("/mavros/vision_pose/pose");

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance& m : view) {
        geometry_msgs::PoseStamped::ConstPtr p = m.instantiate<geometry_msgs::PoseStamped>();
        if (p != NULL && p->pose.position.z > 0.5)
        // if (p != NULL)
            waypoints.push_back(*p);
    }
    bag.close();
    ROS_INFO("Read waypoints from ROSbag, number of waypoints: %lu", waypoints.size());

    // wait for FCU connection
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU Connected");

    // 根据waypoints中第一个点的高度设置起飞点并发送100个
    geometry_msgs::PoseStamped takeoff_pose;
    // takeoff_pose.pose.position.z = waypoints[current_waypoint].pose.position.z;
    takeoff_pose.pose = waypoints[current_waypoint].pose;
    current_waypoint += 1;

    // send a few setpoints before starting
    ROS_INFO("Start send setpoints");
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(takeoff_pose);
        ros::spinOnce();
        rate.sleep();
    }
    // 输出takeoff_pose
    ROS_INFO("Takeoff pose: [%.2f, %.2f, %.2f]", takeoff_pose.pose.position.x, takeoff_pose.pose.position.y, takeoff_pose.pose.position.z);
    

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    // convert to offboard&arm
    ROS_INFO("Convert to OFFBOARD");
    while (ros::ok()) {
        if (!current_state.armed) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
        }
        if(current_state.mode != "OFFBOARD"){
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
        } 
        local_pos_pub.publish(takeoff_pose);
        if (ros::Time::now() - last_request > ros::Duration(10.0)) 
            break;
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Takeoff");

    // follow waypoints
    ROS_INFO("Start follow waypoints");
    // pub waypoint per 10 Hz
    int info_frequency = 10;
    while (ros::ok() && current_waypoint < waypoints.size()) {
        if (current_waypoint < waypoints.size()) {
            local_pos_pub.publish(waypoints[current_waypoint]);
            if (current_waypoint % info_frequency == 0)
                ROS_INFO("Pub waypoint: [%.2f, %.2f, %.2f]", waypoints[current_waypoint].pose.position.x, waypoints[current_waypoint].pose.position.y, waypoints[current_waypoint].pose.position.z);
            current_waypoint++;
        }
        ros::spinOnce();
        rate_waypoint.sleep();
    }
    ROS_INFO("Finished follow waypoints");

    // land
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("AUTO.LAND enabled");
        last_request = ros::Time::now();
    }
    ROS_INFO("Finish Mission");

    return 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
