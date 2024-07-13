#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/NavSatFix.h>

class WaypointTracker
{
public:
    WaypointTracker(const std::string& rosbag_path) : nh("~"), state_connected(false), current_waypoint(0)
    {
        // Subscribers
        state_sub = nh.subscribe("/mavros/state", 10, &WaypointTracker::state_cb, this);

        // Publishers
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        
        // Read waypoints from ROSbag
        read_waypoints_from_bag(rosbag_path);
    }

    void spin()
    {
        ros::Rate rate(20.0);
        
        while (ros::ok() && !state_connected)
        {
            ros::spinOnce();
            rate.sleep();
        }

        for (size_t i = 0; i < 100; ++i)
        {
            publish_current_waypoint();
            ros::spinOnce();
            rate.sleep();
        }

        while (ros::ok())
        {
            publish_current_waypoint();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;

    bool state_connected;
    std::vector<geometry_msgs::PoseStamped> waypoints;
    size_t current_waypoint;

    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        state_connected = msg->connected;
    }

    void read_waypoints_from_bag(const std::string& bag_file)
    {
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back("/mavros/setpoint_position/local");

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
    std::string rosbag_path;
    nh.param<std::string>("rosbag_path", rosbag_path, "/default/path/to/rosbag.bag");
    WaypointTracker tracker(bag_path);
    tracker.spin();
    return 0;
}
