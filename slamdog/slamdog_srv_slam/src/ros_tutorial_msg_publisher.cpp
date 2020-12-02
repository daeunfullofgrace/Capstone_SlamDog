#include "ros/ros.h"
#include "slamdog_srv_slam/msgTutorial.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "slamdog_msg_publisher");
	ros::NodeHandle nh;

	ros::Publisher ros_tutorial_pub = nh.advertise<slamdog_srv_slam::msgTutorial>("ros_tutorial_msg", 100);

	ros::Rate loop_rate(10);

	int count = 0;

	while (count++ < 10)	
	{
		slamdog_srv_slam::msgTutorial msg;

		msg.data = 0;

		ROS_INFO("send msg = %d", 0);

		ros_tutorial_pub.publish(msg);

		loop_rate.sleep();
		
	}

	return 0;
}