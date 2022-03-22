#include <ros/ros.h>

#include <sensor_door_msgs/MultipleDoorStatus.h>
#include <std_srvs/Empty.h>
#include <kobuki_msgs/Sound.h>

struct CheckerDoors
{
    ros::Subscriber door_status;
    ros::ServiceServer rearm_srv;
    ros::Publisher sound;
    ros::Timer remember_sound;

    bool is_secure {false};

    CheckerDoors(ros::NodeHandle& nh)
      : door_status {nh.subscribe("door_status", 10, &CheckerDoors::callback_door_status, this)}
      , rearm_srv {nh.advertiseService("rearm", &CheckerDoors::callback_rearm, this) }
      , sound { nh.advertise<kobuki_msgs::Sound>("sound", 10, false) }
      , remember_sound { nh.createTimer(ros::Duration(7), &CheckerDoors::callback_remember_sound, this, false, false) }
    {}

    void callback_door_status(const sensor_door_msgs::MultipleDoorStatusConstPtr msg)
    {
        const std::size_t n_door = msg->doors.size();
        for (std::size_t i {0} ; i<n_door ; ++i)
        {
            const sensor_door_msgs::DoorStatus& cur_door = msg->doors[i];
            if (cur_door.connected && cur_door.last_response_successfull && cur_door.door_status)
                continue;
            is_secure = false;
            if (cur_door.connected && cur_door.last_response_successfull && !cur_door.door_status)
            {
                kobuki_msgs::Sound s;
                s.value = kobuki_msgs::Sound::CLEANINGSTART;
                sound.publish(s);
            }
        }
        ROS_INFO("xD %d", static_cast<int>(is_secure));
    }

    bool callback_rearm(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
    {
        ROS_INFO("Rearming sensor door");
        is_secure = true;
        remember_sound.stop();
        return true;
    }

    void callback_remember_sound(const ros::TimerEvent& t)
    {
        kobuki_msgs::Sound s;
        s.value = kobuki_msgs::Sound::ERROR;
        sound.publish(s);
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

