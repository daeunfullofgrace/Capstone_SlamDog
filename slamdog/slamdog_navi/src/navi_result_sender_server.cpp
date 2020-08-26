#include "ros/ros.h"
#include "slamdog_navi/arrivalType.h"
#include <cstdlib>

bool getter(slamdog_navi::arrivalType::Request &req, slamdog_navi::arrivalType::Response &res)
{
    res.result = req.type;

    // system("");
    ROS_INFO("Received Type : %1d", (long int)res.result);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slamdog_navi_result_server");
    ros::NodeHandle nh;

    ros::ServiceServer slamdog_navi_result_server = nh.advertiseService("slamdog_navi_result", getter);

    ROS_INFO("Ready Service Server!");

    ros::spin();

    return 0;
}