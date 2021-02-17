#include "ros/ros.h"
#include "slamdog_navi/arrivalType.h"
#include <cstdlib>

int main(int argc, char **argv) {
    ros::init(argc, argv, "slamdog_navi_result_client");

    if (argc != 2) {
       ROS_INFO("cmd : rosrun slamdog_navi navi_result_sender_client arg0");
       ROS_INFO("0 : Succeed | 1: Filed to Init Orientation | 2 : Failed to Go Destination");

       return 0;
    }

    ros::NodeHandle nh;

    ros::ServiceClient slamdog_navi_result_client = nh.serviceClient<slamdog_navi::arrivalType>("slamdog_navi_result");
    slamdog_navi::arrivalType srv;

    srv.request.type = atoll(argv[1]);

    if (slamdog_navi_result_client.call(srv)) {
       ROS_INFO("srv.Request.type  : %ld", (long int)srv.request.type);
       ROS_INFO("Result Type : %ld", (long int)srv.response.result);
    } else {
       ROS_ERROR("Failed to Call Service");
       return 1;
    }
    return 0;
}