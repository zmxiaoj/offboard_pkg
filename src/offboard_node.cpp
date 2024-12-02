
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>	//发布的消息体对应的头文件，该消息体的类型为geometry_msgs::PoseStamped 本地位置
#include <mavros_msgs/CommandBool.h>  //CommandBool服务的头文件，该服务的类型为mavros_msgs::CommandBool
#include <mavros_msgs/SetMode.h>  //SetMode服务的头文件，该服务的类型为mavros_msgs::SetMode
#include <mavros_msgs/State.h>  //订阅的消息体的头文件，该消息体的类型为mavros_msgs::State

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    

    double takeoff_position_x = 0.0, takeoff_position_y = 0.0, takeoff_position_z = 1.0;

    nh.param("takeoff_position_x", takeoff_position_x, 0.0);
    nh.param("takeoff_position_y", takeoff_position_y, 0.0);
    nh.param("takeoff_position_z", takeoff_position_z, 1.0);

    ROS_INFO("takeoff_position_x: %f, takeoff_position_y: %f, takeoff_position_z: %f", takeoff_position_x, takeoff_position_y, takeoff_position_z);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped takeoff_positon;
    // 仅设置起飞高度
    takeoff_positon.pose.position.x = takeoff_position_x;
    takeoff_positon.pose.position.y = takeoff_position_y;
    takeoff_positon.pose.position.z = takeoff_position_z;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(takeoff_positon);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) { 
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(takeoff_positon);

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
