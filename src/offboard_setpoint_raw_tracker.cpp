#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>	
#include <std_msgs/Bool.h>


mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_setpoint_raw_tracker_node");
    ros::NodeHandle nh("~");
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Publisher local_setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local",10);
    ros::Publisher land_cmd_pub = nh.advertise<std_msgs::Bool>("/land_cmd",10);
    std::string rosbag_path;
    std::string rosbag_topic;
    int topic_start_seq,topic_end_seq;
    nh.param<std::string>("rosbag_path", rosbag_path, "null");
    nh.param<std::string>("rosbag_topic",rosbag_topic,"null");
    nh.param<int>("topic_start_seq",topic_start_seq,0);
    nh.param<int>("topic_end_seq",topic_end_seq,0);
    ROS_INFO("rosbag path: %s", rosbag_path.c_str());
    ROS_INFO("rosbag topic: %s", rosbag_topic.c_str());
    ROS_INFO("topic start seq : %d", topic_start_seq);

    std::vector<mavros_msgs::PositionTarget> setpoint_raw_vector;
    size_t current_waypoint = 0;

    ros::Rate rate(20.0), rate_waypoint(100.0);
    // Ros bag and topic read
    rosbag::Bag bag;
    try{
        bag.open(rosbag_path, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e){
        ROS_ERROR("Failed to open bag file : %s",e.what());
        return 1;
    }
    std::vector<std::string> topics;
    topics.push_back(rosbag_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance& m : view) {
        mavros_msgs::PositionTargetConstPtr msg = m.instantiate<mavros_msgs::PositionTarget>();
        if(msg != nullptr){
            if(msg->header.seq < topic_start_seq){
                continue;
            }
            if(msg->header.seq > topic_end_seq){
                break;
            }
            setpoint_raw_vector.push_back(*msg);
        }
    }
    bag.close();
    ROS_INFO("Read waypoints from ROSbag, number of waypoints: %lu", setpoint_raw_vector.size());

    // wait for param "/lih_custom_traj"
    ros::NodeHandle pnh;
    std::string custom_traj_state;
    while(custom_traj_state != "begin"){
        rate.sleep();
        ros::param::get("/lih_custom_traj",custom_traj_state);
        ROS_INFO_THROTTLE(2,"Wait for param /lih_custom_traj");
    }

    // follow waypoints
    ROS_INFO("Start follow waypoints");
    ros::Time prev_msg_time,curr_msg_time;
    for(const mavros_msgs::PositionTarget& msg : setpoint_raw_vector){
        curr_msg_time = msg.header.stamp;
        // 按照消息间隔进行发送
        if(!prev_msg_time.isZero()){
            ros::Duration delay = curr_msg_time - prev_msg_time;
            delay.sleep();
        }
        // msg.header.frame_id = ""
        local_setpoint_pub.publish(msg);
        prev_msg_time = curr_msg_time;
        ROS_INFO_THROTTLE(2.0, "Publishing mavros/setpoint_raw/local");
    }
    ROS_INFO("Finished follow waypoints and set land_cmd");
    ros::param::set("/lih_custom_traj","finish");
    for(int i = 0 ; i < 100; ++i){
        std_msgs::Bool msg;
        msg.data = true;
        land_cmd_pub.publish(msg);
        rate.sleep();
    }
    ROS_INFO("Finish Mission");

    return 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
