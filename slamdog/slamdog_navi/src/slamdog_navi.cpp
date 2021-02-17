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
#include "slamdog_navi/dataLoader.h"
#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>

#define MAX_ARR_SIZE 6
#define FILE_PATH "/home/daeun/catkin_ws/src/slamdog/slamdog_qrcode/data/test.txt"
#define MAX_TIME 10

using namespace std;

class SlamdogNaviResult {
public:
    SlamdogNaviResult(long result) {
        slamdog_navi_result_client = nh.serviceClient<slamdog_navi::arrivalType>("slamdog_navi_result");
        
        srv.request.type = result;
        if (slamdog_navi_result_client.call(srv)) {
            ROS_INFO("Result Type : %ld", (long int)srv.response.result);
        } else {
            ROS_ERROR("Failed to Call Service");
        }
    }
private:
    ros::NodeHandle nh;
    ros::ServiceClient slamdog_navi_result_client;
    slamdog_navi::arrivalType srv;
};

// class NaviServiceServer {
//     public:
//         NaviServiceServer() {
//             server = nh.advertiseService("send_loc_data", execute);
//             ROS_INFO("ready srv server!");

//             ros::spin();
//         }

//         bool execute(slamdog_navi::dataLoader::Request &req, slamdog_navi::dataLoader::Response &res) {
//             res.result = "ok";
//             SlamDogNavi slamDogNavi;

//             return true;
//         }

//     private:
//         ros::NodeHandle nh;
//         ros::ServiceServer server;
// };

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

        cout << "Destination : " << destination_list[idx].substr(1) << endl;
        for(int i=0; i<MAX_ARR_SIZE; i++) {
            printf("Target Odometry %d : %1f\n", i+1, odom_list[idx][i]);
        }

        pub_goal = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        sub_odom = nh.subscribe("/odom", 1, &SlamDogNavi::odomCallBack, this);

        goToDestination(odom_list[idx]);

    }

    void statusCallBack(const actionlib_msgs::GoalStatusArray& msg) {
        int status = msg.status_list[0].status;

        if(cnt < 5) {
            ROS_INFO("status : %d, cnt : %d", status, cnt);
            
            if(status == 3){
                cnt++;
            } else {
                cnt = 0;
            }

            if(status > 3) {
                SlamdogNaviResult navi_result(2);
                exit(0);
            }
        } else {
            SlamdogNaviResult navi_result(0);
            exit(0);
        }
    }

    void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
        origin_pose[0] = msg -> pose.pose.position.x;
        origin_pose[1] = msg -> pose.pose.position.y;
        origin_pose[2] = msg -> pose.pose.orientation.x;
        origin_pose[3] = msg -> pose.pose.orientation.y;
        origin_pose[4] = msg -> pose.pose.orientation.z;
        origin_pose[5] = msg -> pose.pose.orientation.w;
    }

    bool setInitialPose(double* m_pose) {
        geometry_msgs::PoseWithCovarianceStamped pubPoseWithCovarianceStamped;

        pubPoseWithCovarianceStamped.header.stamp = ros::Time::now();

        pubPoseWithCovarianceStamped.pose.pose.position.x = m_pose[0];
        pubPoseWithCovarianceStamped.pose.pose.position.y = m_pose[1];
        pubPoseWithCovarianceStamped.pose.pose.position.z = 0.0;

        pubPoseWithCovarianceStamped.pose.pose.orientation.x = 0.0;
        pubPoseWithCovarianceStamped.pose.pose.orientation.y = 0.0;
        pubPoseWithCovarianceStamped.pose.pose.orientation.z = 0.0;
        pubPoseWithCovarianceStamped.pose.pose.orientation.w = 1.0;

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
                    ++size;
                }
            }
        } else {
            cout<<"Unable to Open File"<<endl;
        }

        inFile.close();
        return size;
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
            int row = -1;

            while(getline(inFile, inLine)){
                if(inLine.at(0) == '%'){
                    list[++row] = inLine;
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

    void goToDestination(double* target_odom){
        
        MoveBaseClient ac("move_base", true);

        while(!ac.waitForServer(ros::Duration(3.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }    
        
        move_base_msgs::MoveBaseGoal goal;

        //-----------------------------------------
        // Navigation
        //-----------------------------------------
        ROS_INFO("--------------------------------");
        ROS_INFO("Goal x : %lf, y : %lf", target_odom[0], target_odom[1]);
        ROS_INFO("Origin x : %lf, y : %lf", origin_pose[0], origin_pose[1]);
        ROS_INFO("Navigation Start!");

        pubPoseStamped.header.frame_id = "map";
        pubPoseStamped.header.stamp = ros::Time::now();

        pubPoseStamped.pose.position.x = target_odom[0];
        pubPoseStamped.pose.position.y = target_odom[1];
        pubPoseStamped.pose.position.z = 0.0;

        pubPoseStamped.pose.orientation.x = 0.0;
        pubPoseStamped.pose.orientation.y = 0.0;
        pubPoseStamped.pose.orientation.z = target_odom[4];
        pubPoseStamped.pose.orientation.w = target_odom[5];
        
        pub_goal.publish(pubPoseStamped);

        ros::Duration(5).sleep();
        ROS_INFO("--------------------------------");
        ROS_INFO("Waiting for Arrival");
        goal_reached = nh.subscribe("/move_base/status", 1, &SlamDogNavi::statusCallBack, this);
    }

private:
    geometry_msgs::PoseWithCovarianceStamped initPose;
    geometry_msgs::PoseStamped pubPoseStamped;
    ros::NodeHandle nh;

    ros::Publisher pub_initial_pose;
    ros::Publisher pub_twist;
    ros::Publisher pub_goal;

    ros::Subscriber sub_odom;
    ros::Subscriber goal_reached;

    double init_pose_position[3] = {0.0, 0.0, 0.0};
    double origin_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    bool is_initialized;
    bool is_executed;
    int idx;
    int cnt = 0;

    double** odom_list;
    string* destination_list;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "slamdog_navi_final");

    // NaviServiceServer server;

    ros::spin();

    return 0;
} 