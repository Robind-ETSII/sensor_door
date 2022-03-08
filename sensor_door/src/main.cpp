#include <ros/ros.h>

#include "sensor_door/server.hpp"
#include "sensor_door/client.hpp"
#include "sensor_door/cluster.hpp"

#include <thread>
#include <unordered_map>

#include "sensor_door_msgs/MultipleDoorStatus.h"
#include <std_srvs/SetBool.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sensor_door_server");
    ros::NodeHandle nh("~");

    int server_port {3333};
    if(!nh.getParam("server_port", server_port))
    {
        ROS_WARN("No listen port defined(~server_port), default port is %d", server_port);
    }
    if(!nh.getParam("client/refresh_rate", uvone_robot::Client::refresh_rate))
    {
        ROS_WARN("No client refresh rate defined(~client/refresh_rate), default refresh rate is %.2f hz", uvone_robot::Client::refresh_rate);
    }
    if(!nh.getParam("client/timeout_ms", uvone_robot::Client::timeout_ms))
    {
        ROS_WARN("No client timeout defined(~client/timeout_ms), default timeout is %.2f ms", uvone_robot::Client::timeout_ms);
    }

    uvone_robot::Cluster cluster(server_port, 5);

    ros::Publisher pub = nh.advertise<sensor_door_msgs::MultipleDoorStatus>("/door_status", 1000);
    ros::ServiceServer srv = nh.advertiseService("clean_disconnected_devices", &uvone_robot::Cluster::callback_clean_devices, &cluster);

    while(ros::ok())
    {
        auto msg {cluster.get_ros_msg()};
        pub.publish(msg);

        ros::spinOnce();
        ros::Rate(10).sleep();
    }
}