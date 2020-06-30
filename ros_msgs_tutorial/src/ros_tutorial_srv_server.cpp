#include "ros/ros.h"
#include "ros_msgs_tutorial/srvTutorial.h"
#include <cstdlib>

bool calculation(ros_msgs_tutorial::srvTutorial::Request &req, ros_msgs_tutorial::srvTutorial::Response &res)
{
    res.result = req.a + req.b;

    // system("rosrun map_server map_saver");
    system("rosrun ros_msgs_tutorial ros_tutorial_msg_publisher");
    ROS_INFO("Map Saved!");

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_tutorial_srv_server");
    ros::NodeHandle nh;

    ros::ServiceServer ros_tutorial_service_server = nh.advertiseService("ros_tutorial_srv", calculation);

    ROS_INFO("ready srv server!");

    ros::spin();

    return 0;
}