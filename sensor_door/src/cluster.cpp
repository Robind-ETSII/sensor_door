#include "sensor_door/cluster.hpp"
#include <iostream>

#include <ros/ros.h>

namespace uvone_robot
{

Cluster::Cluster(uint16_t port, uint8_t queue)
    : _server{port, queue}
    , _clients{}
    , _keep_theads{true}
    , _accept_client_thread{&Cluster::_accept_client_task, this}
{
    ROS_INFO("Cluster is ready to accept door clients");
}

Cluster::~Cluster()
{
    _keep_theads = false;
    _accept_client_thread.join();
}

void Cluster::_accept_client_task()
{
    while (_keep_theads)
    {
        bool repeat {true};
        while (repeat)
        {
            try
            {
                uvone_robot::Client new_client{_server.accept_client()};
                std::string new_client_id{new_client.get_id()};
                _clients[new_client_id]=std::move(new_client);
                
                repeat = false;
            }
            catch(const std::exception& e)
            {
                if (!_keep_theads)
                    repeat = false;
                //std::cerr << e.what() << '\n';
            }
        }
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

sensor_door_msgs::MultipleDoorStatus Cluster::get_ros_msg() const
{
    sensor_door_msgs::MultipleDoorStatus msg;
    msg.header.stamp = ros::Time::now();
        
    for (const auto& c : _clients)
    {
        msg.doors.push_back(c.second.get_ros_msg());
    }

    return msg;
}

bool Cluster::callback_clean_devices(sensor_door_msgs::CleanDisconnectedDevices::Request &req, sensor_door_msgs::CleanDisconnectedDevices::Response &res)
{
    ROS_INFO("Cleaning disconnected devices");

    res.num_disconnected_devices = 0;
    res.num_remaining_devices = 0;

    for (auto it {_clients.cbegin()} ; it != _clients.cend() ; )
    {
        
        if ((it->second).is_connected())
        {
            res.num_remaining_devices++;
            ++it;
        }
        else
        {
            res.num_disconnected_devices++;
            it = _clients.erase(it);
        }
    }


    res.success = true;

    return true;
}


    
} // namespace uvone_robot
