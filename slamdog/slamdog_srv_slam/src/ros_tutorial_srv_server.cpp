#include "ros/ros.h"
#include "slamdog_srv_slam/srvTutorial.h"
#include <cstdlib>

bool calculation(slamdog_srv_slam::srvTutorial::Request &req, slamdog_srv_slam::srvTutorial::Response &res)
{
    res.result = req.a + req.b;

    system("rosrun map_server map_saver -f ~/catkin_ws/src/slamdog/maps/map");
    system("rosrun slamdog_srv_slam ros_tutorial_msg_publisher");
    ROS_INFO("Map Saved!");

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_slam_srv_server");
    ros::NodeHandle nh;

    ros::ServiceServer ros_tutorial_service_server = nh.advertiseService("ros_tutorial_srv", calculation);

    ROS_INFO("ready srv server!");

    ros::spin();

    return 0;
}
