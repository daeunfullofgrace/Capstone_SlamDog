#include "ros/ros.h"
#include "slamdog_srv_slam/srvTutorial.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slamdog_slam_srv_client");

    if (argc != 3)
    {
       ROS_INFO("cmd : rosrun slamdog_srv_slam ros_tutorial_service_client arg0 arg1");
       ROS_INFO("arg0 : double number, arg1 : double number");

       return 1;
    }

    ros::NodeHandle nh;

    ros::ServiceClient ros_tutorial_service_client = nh.serviceClient<slamdog_srv_slam::srvTutorial>("ros_tutorial_srv");
    slamdog_srv_slam::srvTutorial srv;

    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);

    if (ros_tutorial_service_client.call(srv))
    {
       ROS_INFO("send srv, srv.Request.a and b : %1d, %1d", (long int)srv.request.a, (long int)srv.request.b);
       ROS_INFO("recieve srv, srv.Response.result : %1d", (long int)srv.response.result);
    }  
    else
    {
       ROS_ERROR("Failed to call service");
       return 1;
    }
    return 0;
}