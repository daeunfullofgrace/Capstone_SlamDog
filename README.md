# SLAMDOG
* Execution video: [link](https://youtu.be/t8WpG0OXGug?feature=shared)

* This project is a capstone of Chungnam National University in 2022.
* It's based on ROS to provide an indoor navigation robot for the blind.
 
## Overall Functions
1. With SLAM, we can control the robot and store the QR code location and its information on the server.
* The QR code stores the name of the target location.
2. When navigating, the robot goes to the location of the stored QR code by controlling the smartphone.
* The application supports voice guidance and the selection of a target location.
 
## SLAM: Process to generate a map for navigation
By using ROS, the application and server are connected.
* Application: to control the robot; * Server: to store the target locations
  
### 1) ROS Node
* Store data on the server PC
* If the QR code is detected, the current odometry is stored in a stack, and then this data is stored in a text file.
   
#### * Sequence of execution
 1. gazebo or turtlebot3_bringup
 2. rosrun slamdog_srv_slam ros_tutorial_srv_server
 3. rosrun slamdog_qrcode qrcode_scanner
 4. roslaunch turtlebot3_slam turtlebot3_slam
 5. Run your application.
  
### 2) Application (make_a_map)
The default structure is make_a_map, which is open source from ROS and RIODA.
The difference is a joystick, which is for controlling the robot, and the data storage function on the server.
  
## Navigation
Navigate to the target location selected by the user.
The overall UI is designed for the blind.
When completing the navigation, the user can get an alert from the application.
 
### 1) ROS Node
Set the target location as the odometry in the text file.
 Before starting the navigation, the pose_initlization.py file can initialize the current location of the robot.
 
#### * Sequence of execution
 1. gazebo or turtlebot3_bringup
 2. roslaunch turtlebot3_navigation turtlebot3_navigiation map_file:=$HOME/catkin_ws/src/slamdog/maps/map.yaml
 3. rosrun slamdog_srv_navi send_loc_data_server.py
 4. rosrun slamdog_srv_navi loc_file_reader_server.py
 5. Run your application.
 
### 2) Application (blind guide)
This application supports voice guidance for the blind, similar to Samsung Voice Assistance, which is a well-known application for the blind.
