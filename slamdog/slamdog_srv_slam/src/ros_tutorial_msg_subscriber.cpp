#include "ros/ros.h"
#include "slamdog_srv_slam/msgTutorial.h"

void msgCallback(const slamdog_srv_slam::msgTutorial::ConstPtr& msg)
{
	ROS_INFO("recieve msg: %d", msg->data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "slamdog_msg_subscriber");
	ros::NodeHandle nh;

	ros::Subscriber ros_tutorial_sub = nh.subscribe("ros_tutorial_msg", 10, msgCallback);

	ros::spin();
	
	return 0;
}
