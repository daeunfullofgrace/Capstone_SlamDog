#include "ros/ros.h"
#include "ros_msgs_tutorial/msgTutorial.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_tutorial_msg_publisher");
	ros::NodeHandle nh;

	ros::Publisher ros_tutorial_pub = nh.advertise<ros_msgs_tutorial::msgTutorial>("ros_tutorial_msg", 100);

	ros::Rate loop_rate(10);

	int count = 0;

	while (count++ < 10)	
	{
		ros_msgs_tutorial::msgTutorial msg;

		msg.data = 0;

		ROS_INFO("send msg = %d", 0);

		ros_tutorial_pub.publish(msg);

		loop_rate.sleep();
		
	}

	return 0;
}