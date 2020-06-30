#include "ros/ros.h"
#include "data_manager/dataLoader.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_loader_client");

    if (argc != 2)
    {
       ROS_INFO("arg : double number");

       return 1;
    }

    ros::NodeHandle nh;

    ros::ServiceClient service_client = nh.serviceClient<data_manager::srvTutorial>("data_loader_srv");
    data_manager::dataLoader srv;

    srv.request.a = atoll(argv[1]);

    if (service_client.call(srv))
    {
       ROS_INFO("send srv, srv.Request.a: %1d", (long int)srv.request.a);
       ROS_INFO("recieve srv, srv.Response.result : %1d", (long int)srv.response.result);
    }  
    else
    {
       ROS_ERROR("Failed to call service");
       return 1;
    }
    return 0;
}