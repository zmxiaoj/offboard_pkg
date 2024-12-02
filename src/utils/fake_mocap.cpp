#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_mocap_node");
    ros::NodeHandle nh("~");

    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node/uav/pose", 1000);

    ros::Rate loop_rate(100.0); // 10 Hz

    Eigen::Vector3f position(1.0, 2.0, 3.0);
    Eigen::Quaternionf quaternion(1.0, 0.0, 0.0, 0.0);

    while (ros::ok()) {
        geometry_msgs::PoseStamped fake_mocap;
        fake_mocap.header.stamp = ros::Time::now();
        fake_mocap.header.frame_id = "world";

        // set position
        fake_mocap.pose.position.x = position.x();
        fake_mocap.pose.position.y = position.y();
        fake_mocap.pose.position.z = position.z();

        // set pose (quaternion)
        fake_mocap.pose.orientation.x = quaternion.x();
        fake_mocap.pose.orientation.y = quaternion.y();
        fake_mocap.pose.orientation.z = quaternion.z();
        fake_mocap.pose.orientation.w = quaternion.w();

        ROS_INFO_STREAM("---------- Fake Mocap ----------");
        ROS_INFO_STREAM("Position: " << position.transpose());
        ROS_INFO_STREAM("Quaternion: " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w());

        mocap_pub.publish(fake_mocap);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}