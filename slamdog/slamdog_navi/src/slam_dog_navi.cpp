#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <cmath>
#include <math.h>
#include <time.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>

#include "slamdog_navi/arrivalType.h"
#include <cstdlib>

#define MSE_IS_POSE_INITIALIZING_COMPLETED 12000.0
#define MAX_ARR_SIZE 6
#define FILE_PATH "/home/daeun/catkin_ws/src/slamdog/slamdog_qrcode/data/qr_code_data.txt"
#define MAX_TIME 10

using namespace std;

class SlamdogNaviResult {
public:
    SlamdogNaviResult(long result) {
        slamdog_navi_result_client = nh.serviceClient<slamdog_navi::arrivalType>("slamdog_navi_result");
        
        srv.request.type = result;
        if (slamdog_navi_result_client.call(srv)) {
            ROS_INFO("Result Type : %1d", (long int)srv.response.result);
        } else {
            ROS_ERROR("Failed to Call Service");
        }
    }
private:
    ros::NodeHandle nh;
    ros::ServiceClient slamdog_navi_result_client;
    slamdog_navi::arrivalType srv;
};

class SlamDogNavi {
public:
    SlamDogNavi() {
        is_initialized = false;
        is_executed = false;
        idx = 0;

        ros::NodeHandle n("~");
        n.getParam("id", idx);

        odom_list = getOdom();
        destination_list = getDestination();

        cout << "Destination : " << destination_list[idx] << endl;
        for(int i=0; i<MAX_ARR_SIZE; i++) {
            printf("Target Odometry %d : %1f\n", i+1, odom_list[idx][i]);
        }

        ROS_INFO("--------------------------------");
        ROS_INFO("Position Initializing");

        pub_initial_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
        pub_twist = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        sub_gather_particle = nh.subscribe("particlecloud", 1, &SlamDogNavi::cbGatherParticle, this);
        sub_odom = nh.subscribe("odom", 1, &SlamDogNavi::odomCallBack, this);
        
        if(abs(init_pose_position[0] - origin_pose[0]) >= 0.5 && abs(init_pose_position[1] - origin_pose[1]) >= 0.5)
            is_initialized = setInitialPose(init_pose_position);
        else 
            is_initialized = setInitialPose(origin_pose);
    }

    void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
        origin_pose[0] = msg -> pose.pose.position.x;
        origin_pose[1] = msg -> pose.pose.position.y;
        origin_pose[2] = msg -> pose.pose.orientation.x;
        origin_pose[3] = msg -> pose.pose.orientation.x;
        origin_pose[4] = msg -> pose.pose.orientation.z;
        origin_pose[5] = msg -> pose.pose.orientation.w;
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

        if (score > MSE_IS_POSE_INITIALIZING_COMPLETED) {
            geometry_msgs::Twist twist;
            twist.angular.z = 0.8;

            ROS_INFO("MSE : %lf > %lf", score, MSE_IS_POSE_INITIALIZING_COMPLETED);

            pub_twist.publish(twist);
        } else {
            ROS_INFO("--------------------------------");
            ROS_INFO("Pose Initializing Completed");
            ROS_INFO("Origin Position x : %lf, y: %lf", origin_pose[0], origin_pose[1]);

            geometry_msgs::Twist twist;
            twist.angular.z = 0.0;  

            pub_twist.publish(twist);
    
            sub_odom.shutdown();

            int result = goToDestination(odom_list[idx]);
            SlamdogNaviResult navi_result(result);
            
            if(result == 0)
                goBacktoInitialPose(odom_list[idx]);

            exit(0);
        }
    }

    bool setInitialPose(float* m_pose) {
        geometry_msgs::PoseWithCovarianceStamped pubPoseWithCovarianceStamped;

        pubPoseWithCovarianceStamped.header.stamp = ros::Time::now();

        pubPoseWithCovarianceStamped.pose.pose.position.x = m_pose[0];
        pubPoseWithCovarianceStamped.pose.pose.position.y = m_pose[1];
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

        // ros::Rate poll_rate(100);
        // while(pub_initial_pose.getNumSubscribers() == 0)
        //     poll_rate.sleep();

        pub_initial_pose.publish(pubPoseWithCovarianceStamped);

        return true;
  }

    int getNumOfDestination() {
        std::ifstream inFile;
        string inLine;
        int size = -1;

        inFile.open(FILE_PATH);

        if(inFile.is_open()){
            while(getline(inFile, inLine)){
                if(inLine.at(0) == '%'){
                    size++;
                }
            }
        } else {
            cout<<"Unable to Open File"<<endl;
        }

        inFile.close();
        return size+1;
    }

    int max(int a){
        if(a >= MAX_ARR_SIZE)
            return MAX_ARR_SIZE;
        else 
            return a;
    }

    string* getDestination() {
        std::ifstream inFile;
        string inLine;

        string* list = new string[getNumOfDestination()];
        inFile.open(FILE_PATH);

        if(inFile.is_open()){
            int row = 0;

            while(getline(inFile, inLine)){
                if(inLine.at(0) == '%'){
                    list[row++] = inLine.substr(1);
                }
            }
        } else {
            cout<<"Unable to Open File"<<endl;
        }

        inFile.close();
        return list;
    }

    double** getOdom(){
        std::ifstream inFile;
        string inLine;

        double **list = 0;
        list = new double*[getNumOfDestination()];

        inFile.open(FILE_PATH);

        if(inFile.is_open()){
            int col = 0;
            int row = -1;

            while(getline(inFile, inLine)) {
                if(inLine.at(0) == '%') {
                    list[++row] = new double[MAX_ARR_SIZE];
                    col = 0;
                } else {
                    list[row][col++] = boost::lexical_cast<double>(inLine);
                }
            }
        } else {
            cout<<"Unable to Open File"<<endl;
        }

        inFile.close();
        return list;
    }

    geometry_msgs::Quaternion getRotMatrix(double* origin, double* target){
        tf2::Quaternion q_rot;
        geometry_msgs::Quaternion q_msg;
        
        tf2::Quaternion q_origin_inv(origin[0], origin[1], origin[2], -origin[3]);
        tf2::Quaternion q_target(target[0], target[1], target[2], target[3]);

        q_rot = q_target * q_origin_inv;
        q_msg = tf2::toMsg(q_rot);

        return q_msg;
    }

    int goToDestination(double* target_odom){
        move_base_msgs::MoveBaseGoal goal;
        MoveBaseClient ac("move_base", true);

        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }    

        //-----------------------------------------
        // Orientation Initiation
        //-----------------------------------------
        double* q_origin = new double[4]{origin_pose[2], origin_pose[3], origin_pose[4], origin_pose[5]};
        double* q_target = new double[4]{0.0, 0.0, 0.0, 1.0};   //x,y,z -> 0 dgree
    
        ROS_INFO("--------------------------------");
        ROS_INFO("Orientation Initiation!");    

        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.position.z = 0.0;

        goal.target_pose.pose.orientation = getRotMatrix(q_origin, q_target);

        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Initiation Successed!");
        } else {
            ROS_INFO("Failed to Initiation");
            return 1;
        }

        //-----------------------------------------
        // Navigation
        //-----------------------------------------
        ROS_INFO("--------------------------------");
        ROS_INFO("Goal x : %lf, y : %lf", target_odom[0], target_odom[1]);
        ROS_INFO("Navigation Start!");

        q_origin = new double[4]{0.0, 0.0, 0.0, 1.0};
        q_target = new double[4]{target_odom[2], target_odom[3], target_odom[4], target_odom[5]};

        goal.target_pose.pose.position.x = target_odom[0]-origin_pose[0];
        goal.target_pose.pose.position.y = target_odom[1]-origin_pose[1];

        goal.target_pose.pose.orientation = getRotMatrix(q_origin, q_target);

        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Destination Arrival!");
            return 0;
        } else {
            ROS_INFO("Failed to Arrive Destination");
            return 2;
        }

        //-----------------------------------------
        // Go Back to Initial Point
        //-----------------------------------------
        // ROS_INFO("--------------------------------");
        // ROS_INFO("Go Back to Initial Point");

        // goal.target_pose.pose.position.x = 0.0;
        // goal.target_pose.pose.position.y = 0.0;
        // goal.target_pose.pose.position.z = 0.0;

        // goal.target_pose.pose.orientation = getRotMatrix(q_target, q_origin);

        // ac.sendGoal(goal);
        // ac.waitForResult();

        // goal.target_pose.pose.position.x = origin_pose[0]-target_odom[0];
        // goal.target_pose.pose.position.y = origin_pose[1]-target_odom[1];

        // goal.target_pose.pose.orientation.x = 0.0;
        // goal.target_pose.pose.orientation.y = 0.0;
        // goal.target_pose.pose.orientation.z = 0.0;
        // goal.target_pose.pose.orientation.w = 1.0;

        // ac.sendGoal(goal);
        // ac.waitForResult();

        // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        //     ROS_INFO("Initial Point Arrival!");
        //     return 0;
        // } else {
        //     ROS_INFO("Failed to Arrive Initial Point");
        //     return 3;
        // }
    }

    bool goBacktoInitialPose(double* target_odom){
        move_base_msgs::MoveBaseGoal goal;
        MoveBaseClient ac("move_base", true);
        
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }    

        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        //-----------------------------------------
        // Go Back to Initial Point
        //-----------------------------------------
        ROS_INFO("--------------------------------");
        ROS_INFO("Go Back to Initial Point");

        double* q_origin = new double[4]{target_odom[2], target_odom[3], target_odom[4], target_odom[5]};
        double* q_target = new double[4]{0.0, 0.0, 0.0, 1.0};

        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.position.z = 0.0;

        goal.target_pose.pose.orientation = getRotMatrix(q_origin, q_target);

        ac.sendGoal(goal);
        ac.waitForResult();

        goal.target_pose.pose.position.x = origin_pose[0]-target_odom[0];
        goal.target_pose.pose.position.y = origin_pose[1]-target_odom[1];

        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;

        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            float tmp[2] = {init_pose_position[0]+0.1, init_pose_position[1]+0.1};
            
            ROS_INFO("Initial Point Arrival!");
            setInitialPose(tmp);
            return true;
        } else {
            ROS_INFO("Failed to Arrive Initial Point");
            return false;
        }

    }

private:
    geometry_msgs::PoseWithCovarianceStamped initPose;
    ros::NodeHandle nh;

    ros::Publisher pub_initial_pose;
    ros::Publisher pub_twist;

    ros::Subscriber sub_gather_particle;
    ros::Subscriber sub_odom;

    float init_pose_position[3] = {0.0, 0.0, 0.0};
    float origin_pose[6];
    
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    bool is_initialized;
    bool is_executed;
    int idx;

    double** odom_list;
    string* destination_list;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "sd_pose_initialization");

    SlamDogNavi slamDogNavi;

    ros::spin();

    return 0;
} 