#pragma once
#include <unordered_map>
#include <thread>
#include <atomic>

#include "sensor_door/client.hpp"
#include "sensor_door/server.hpp"

#include "sensor_door_msgs/MultipleDoorStatus.h"
#include "sensor_door_msgs/CleanDisconnectedDevices.h"

namespace uvone_robot
{

class Cluster
{
private:
    Server _server;
    std::unordered_map<std::string, uvone_robot::Client> _clients;

    std::atomic_bool _keep_theads;
    std::thread _accept_client_thread;

    void _accept_client_task();
    
public:
    Cluster(uint16_t port, uint8_t queue);
    const std::unordered_map<std::string, uvone_robot::Client>& get_clients() const {return _clients;}
    ~Cluster();

    sensor_door_msgs::MultipleDoorStatus get_ros_msg() const;
    bool callback_clean_devices(sensor_door_msgs::CleanDisconnectedDevices::Request &req, sensor_door_msgs::CleanDisconnectedDevices::Response &res);
};


} // namespace uvone_robot
