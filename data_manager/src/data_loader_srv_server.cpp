#include "ros/ros.h" 
#include "data_manager/dataLoader.h"
#include <cstdlib>

bool calculation(data_manager::dataLoader::Request &req, data_manager::dataLoader::Response &res) {
    res.result = req.a;
    ROS_INFO("sending back response: [%ld]", (long int)res.result);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_loader_server");
    ros::NodeHandle nh;

    ros::ServiceServer ros_tutorial_service_server = nh.advertiseService("data_loader_srv", calculation);
    ROS_INFO("ready srv server!");
    ros::spin();
    return 0;
}