#include <ros/ros.h>

#include <sensor_door_msgs/MultipleDoorStatus.h>
#include <sensor_door_msgs/DoorSecurity.h>
#include <std_srvs/Empty.h>
#include <kobuki_msgs/Sound.h>

struct CheckerDoors
{
    ros::Subscriber door_status;
    ros::ServiceServer rearm_srv;
    ros::Publisher system_pub;

    bool is_secure {false};

    CheckerDoors(ros::NodeHandle& nh)
      : door_status {nh.subscribe("door_status", 10, &CheckerDoors::callback_door_status, this)}
      , rearm_srv {nh.advertiseService("rearm", &CheckerDoors::callback_rearm, this) }
      , system_pub { nh.advertise<sensor_door_msgs::DoorSecurity>("sys_status", 10, false) }
    {}

    void callback_door_status(const sensor_door_msgs::MultipleDoorStatusConstPtr msg)
    {
        const std::size_t n_door = msg->doors.size();
        sensor_door_msgs::DoorSecurity s;
        

        for (std::size_t i {0} ; i<n_door ; ++i)
        {
            const sensor_door_msgs::DoorStatus& cur_door = msg->doors[i];
            if (cur_door.connected && cur_door.last_response_successfull && cur_door.door_status)
                continue;
            
            ROS_INFO_COND(is_secure, "System has detected intrusion. Disarmed!");
            is_secure = false;
            
            if (!cur_door.connected)
            {
                ROS_WARN_DELAYED_THROTTLE(10, "Disconnected devices is added.");
            }
        }
        s.header = msg->header;
        s.is_secure = is_secure;
        system_pub.publish(s);
    }

    bool callback_rearm(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
    {
        ROS_INFO_COND(!is_secure, "Service to rearm called. Reaming!");
        is_secure = true;
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "light_cmd_node");
    ros::NodeHandle nh("~");

    CheckerDoors checker{nh};
    checker.is_secure = true;

    ros::spin();

    return 0;
}

