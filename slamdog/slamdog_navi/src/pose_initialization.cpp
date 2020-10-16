#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_msgs/TFMessage.h"

#define MSE_IS_POSE_INITIALIZING_COMPLETED 10000.0

class PoseInitialization{
public:
    PoseInitialization(){
        is_initialized = true;
        pub_initial_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
        pub_twist = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        
        ROS_INFO("pose initialization");
        sub_gather_particle = nh.subscribe("particlecloud", 1, &PoseInitialization::cbGatherParticle, this);
        // is_initialized = setInitialPose();
    }

    void cbGatherParticle(const geometry_msgs::PoseArray& poseArray) {
        if (!is_initialized)
            return;

        int size = poseArray.poses.size();

        float score = 0.0;

        for (int i = 0; i < size; i++) {
            for (int j = i + 1; j < size; j++)
                score += sqrt((poseArray.poses[i].position.x - poseArray.poses[j].position.x) * (poseArray.poses[i].position.x - poseArray.poses[j].position.x) +
                        (poseArray.poses[i].position.y - poseArray.poses[j].position.y) * (poseArray.poses[i].position.y - poseArray.poses[j].position.y));
        }

        if (score > MSE_IS_POSE_INITIALIZING_COMPLETED && score < 2*MSE_IS_POSE_INITIALIZING_COMPLETED) {
            geometry_msgs::Twist twist;
            twist.angular.z = 0.8;

            ROS_INFO("on pose initializing | MSE : %lf > %lf", score, MSE_IS_POSE_INITIALIZING_COMPLETED);

            pub_twist.publish(twist);
        } else if (score > 2*MSE_IS_POSE_INITIALIZING_COMPLETED){
            geometry_msgs::Twist twist;
            twist.angular.z = -0.8;

            ROS_INFO("on pose initializing | MSE : %lf > %lf", score, MSE_IS_POSE_INITIALIZING_COMPLETED);

            pub_twist.publish(twist);
        } else {
            geometry_msgs::Twist twist;
            twist.angular.z = 0.0;

            ROS_INFO("pose initializing completed");

            pub_twist.publish(twist);
            // system("rosrun slamdog_srv_navi send_loc_data_server");

            exit(0);
        }
    }

    bool setInitialPose() {
        geometry_msgs::PoseWithCovarianceStamped pubPoseWithCovarianceStamped;

        pubPoseWithCovarianceStamped.header.stamp = ros::Time::now();

        pubPoseWithCovarianceStamped.pose.pose.position.x = 0;
        pubPoseWithCovarianceStamped.pose.pose.position.y = 0;
        pubPoseWithCovarianceStamped.pose.pose.position.z = 0;

        pubPoseWithCovarianceStamped.pose.pose.orientation.x = 0;
        pubPoseWithCovarianceStamped.pose.pose.orientation.y = 0;
        pubPoseWithCovarianceStamped.pose.pose.orientation.z = 0;
        pubPoseWithCovarianceStamped.pose.pose.orientation.w = 1;

        // pubPoseWithCovarianceStamped.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 
        //     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

        pubPoseWithCovarianceStamped.pose.covariance[6*0+0] = 0.5 * 0.5;
        pubPoseWithCovarianceStamped.pose.covariance[6*1+1] = 0.5 * 0.5;
        pubPoseWithCovarianceStamped.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

        ros::Rate poll_rate(100);
        while(pub_initial_pose.getNumSubscribers() == 0)
            poll_rate.sleep();

        pub_initial_pose.publish(pubPoseWithCovarianceStamped);

        return true;
  }

private:
    geometry_msgs::PoseWithCovarianceStamped initPose;
    ros::NodeHandle nh;

    ros::Publisher pub_initial_pose;
    ros::Publisher pub_twist;

    ros::Subscriber sub_gather_particle;

    bool is_initialized;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "sd_pose_initialization");

    PoseInitialization poseInitialization;

    ros::spin();

    return 0;
} 