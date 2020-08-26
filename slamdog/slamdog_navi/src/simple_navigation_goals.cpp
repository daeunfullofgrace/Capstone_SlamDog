#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <cmath>
#include <math.h>
#include <time.h>

#define MAX_SIZE 10
#define MAX_ARR_SIZE 6
#define POSE_SIZE 7
#define MAX_TIME 10
#define FILE_PATH "/home/daeun/catkin_ws/src/qrcode/data/qr_code_data.txt"

using namespace std;

// pose initiation && setting goal
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseWithCovarianceStamped initPose;
float mPose[POSE_SIZE];
float mAmclPose[POSE_SIZE];
bool isDone = false;
bool isSame = false;

void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg) {
  mPose[0] = msg -> pose.pose.position.x;
  mPose[1] = msg -> pose.pose.position.y;
  mPose[2] = msg -> pose.pose.position.z;
  mPose[3] = msg -> pose.pose.orientation.x;
  mPose[4] = msg -> pose.pose.orientation.y;
  mPose[5] = msg -> pose.pose.orientation.z;
  mPose[6] = msg -> pose.pose.orientation.w;

  // ROS_INFO("Current Pose x : %f", mPose[0]);
}

void amclCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgAMCL){
  mAmclPose[0] = msgAMCL -> pose.pose.position.x;
  mAmclPose[1] = msgAMCL -> pose.pose.position.y;
  // mAmclPose[2] = msgAMCL -> pose.pose.position.z;
  // mAmclPose[3] = msgAMCL -> pose.pose.orientation.x;
  // mAmclPose[4] = msgAMCL -> pose.pose.orientation.y;
  // mAmclPose[5] = msgAMCL -> pose.pose.orientation.z;
  // mAmclPose[6] = msgAMCL -> pose.pose.orientation.w;

  ROS_INFO("Current AMCL Pose x : %f", mAmclPose[0]);
}

void calInitPose(){
  string fixed_frame = "map";

  initPose.header.frame_id = fixed_frame;
  initPose.header.stamp = ros::Time::now();

  initPose.pose.pose.position.x = mPose[0];
  initPose.pose.pose.position.y = mPose[1];
  initPose.pose.pose.position.z = mPose[2];

  initPose.pose.pose.orientation.x = mPose[3];
  initPose.pose.pose.orientation.y = mPose[4];
  initPose.pose.pose.orientation.z = mPose[5];
  initPose.pose.pose.orientation.w = mPose[6];

  initPose.pose.covariance[6*0+0] = 0.5 * 0.5;
  initPose.pose.covariance[6*1+1] = 0.5 * 0.5;
  initPose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

  isDone = true;
}

int getTargetSize(){
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

string* getDestination(){
  std::ifstream inFile;
  string inLine;

  string* list = new string[getTargetSize()];
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
  list = new double*[getTargetSize()];

  inFile.open(FILE_PATH);

  if(inFile.is_open()){
    int col = 0;
    int row = -1;

    while(getline(inFile, inLine)){
      if(inLine.at(0) == '%'){
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

bool goToDestination(double* odomList){
  ROS_INFO("Navigation Start!");
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = odomList[0]-mPose[0];
  goal.target_pose.pose.position.y = odomList[1]-mPose[1];
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  
  if(odomList[5] < 0)
    goal.target_pose.pose.orientation.w = -1.0;
  else 
    goal.target_pose.pose.orientation.w = 1.0;

  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    return true;
  else
    return false;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "slamdog_map_navi");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(3);

  // Get Input Integer
  int idx = 0;
  ros::NodeHandle n("~");
  n.getParam("id", idx);
  cout << idx << endl;

  // Get Target Odom
  double** odomList = getOdom();
  string* destinationList = getDestination();

  cout << "destination name : " << destinationList[idx] << endl;
  for(int i=0; i<MAX_ARR_SIZE; i++){
    printf("odom %d : %1f\n", i+1, odomList[idx][i]);
  }

  spinner.start();

  // Subscribe On
  ros::Subscriber sub = nh.subscribe("odom", 10, odomCallBack);
  // ros::Subscriber subAmcl = nh.subscribe("amcl_pose", 100, amclCallBack);
  ros::Publisher pub 
    = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);

  ROS_INFO("waiting for initial position setting");

  // Wait for Initial Odometry Setting
  clock_t start, end;
  start = clock();

  while(true){
    calInitPose();
    end = clock();
    pub.publish(initPose);
    double det = (double)((end-start)/CLOCKS_PER_SEC);

    if(det >= MAX_TIME) {
      pub.shutdown();
      sub.shutdown();
      break;
    }
  }

  if(goToDestination(odomList[idx])){
    ROS_INFO("Destination Arrival!");
  } else {
    ROS_INFO("Failed to Arrive Destination");
  }

  return 0;
}