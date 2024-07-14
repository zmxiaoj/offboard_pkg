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
#include <sensor_msgs/NavSatFix.h>

class WaypointTracker
{
public:
    WaypointTracker(const std::string& rosbag_path) : nh("~"), current_waypoint(0)
    {
        // Subscribers
        state_sub = nh.subscribe("/mavros/state", 10, &WaypointTracker::state_cb, this);
        
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

        arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
        // Publishers
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        
        // Read waypoints from ROSbag
        read_waypoints_from_bag(rosbag_path);
    }

    void spin()
    {
        ros::Rate rate(20.0);
        
        // wait for FCU
        while (ros::ok() && !current_state.connected)
        {
            ros::spinOnce();
            rate.sleep();
        }

        // send a few setpoints before starting
        for (size_t i = 0; i < 100; ++i)
        {
            publish_current_waypoint();
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        
        ros::Time last_request = ros::Time::now();

        while (ros::ok())
        {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            publish_current_waypoint();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;
    ros::Publisher local_pos_pub;

    mavros_msgs::State current_state;
    std::vector<geometry_msgs::PoseStamped> waypoints;
    size_t current_waypoint;

    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;
    }

    /**
     * @brief 从rosbag中的topic /vrpn_client_node/P450/pose读取waypoints保存到Vetor
     * 
     * @param bag_file 
     */
    void read_waypoints_from_bag(const std::string& bag_file)
    {
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back("/vrpn_client_node/P450/pose");

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for (const rosbag::MessageInstance& m : view)
        {
            geometry_msgs::PoseStamped::ConstPtr p = m.instantiate<geometry_msgs::PoseStamped>();
            if (p != NULL)
                waypoints.push_back(*p);
        }
        bag.close();
    }

    void publish_current_waypoint()
    {
        if (current_waypoint < waypoints.size())
        {
            local_pos_pub.publish(waypoints[current_waypoint]);
            current_waypoint++;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_tracker");
    ros::NodeHandle nh;
    std::string rosbag_path;
    nh.param<std::string>("rosbag_path", rosbag_path, "/default/path/to/rosbag.bag");
    WaypointTracker tracker(rosbag_path);
    tracker.spin();
    return 0;
}
