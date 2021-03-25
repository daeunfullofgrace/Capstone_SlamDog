# SLAMDOG
시각장애인을 위한 안내 로봇으로 ROS 기반이다. 
<br>사용된 로봇은 turtlebot3 burger와 동일하게 동작하며 burger를 이용하는 것도 가능하다.
 
 ## Functions
 1. SLAM 과정 진행 시, 안드로이드 스마트폰을 이용한 로봇 제어 및 QR 코드 인식, 서버에 맵 & QR 코드 및 QR 코드 위치 저장 가능.
 <br>(QR  코드의 위치가 목적지의 위치가 되고 데이터는 목적지의 이름)
 2. Navigation과정 진행 시, 안드로이드 스마트폰을 이용 시 음성 안내 및 어플을 통한 목적지 선택 가능.
 3. 선택한 목적지 전송 및 목적지까지 이동.
 
 ## SLAM 
  내비게이션을 위한 지도를 생성하는 과정이다.
  <br>ROS를 이용해 어플과 PC가 통신하게 되며, 어플을 이용해 로봇을 조종하고 데이터를 서버 PC에 저장할 수 있다.
  
  ### 1) ROS Node
   ROS Service를 이용하여 서버 PC에 데이터가 저장되도록 구현하였다.
   <br>QR 코드 인식 시 현재의 Odometry를 받아 Stack에 저장되고 이 내용이 텍스트 파일로 저장된다.
   
   #### ** 실행 순서
    1. gazebo or turtlebot3_bringup
    2. rosrun slamdog_srv_slam ros_tutorial_srv_server
    3. rosrun slamdog_qrcode qrcode_scanner
    4. roslaunch turtlebot3_slam turtlebot3_slam
    5. Run Your Application
  
  ### 2) Application (make_a_map)
  기본적인 구조는 ROS Open Source인 android_apps의 make_a_map을 이용하였다.
  <br>차이점은 로봇을 조종하는 부분이 조이스틱에서 버튼으로 변경되었고 서버 PC에 데이터를 저장할 수 있도록 구현하였다.
  
 ## Navigation
 사용자가 설정한 목적지까지 안내하는 과정으로 사용자가 시각장애인임을 고려하여 UI를 설계하였다.
 <br>주행 후 도착 완료 여부를 사용자에게 안내해준다.
 
 ### 1) ROS Node
 저장된 텍스트 파일을 읽어 파일의 Odometry를 Navigation의 목적지로 설정한다.
 <br>Navigation 시작 전, 로봇의 현재 위치 파악을 위한 pose_intialization.py 파일이 존재한다.
 
 #### ** 실행 순서
    1. gazebo or turtlebot3_bringup
    2. roslaunch turtlebot3_navigation turtlebot3_navigiation map_file:=$HOME/catkin_ws/src/slamdog/maps/map.yaml
    3. rosrun slamdog_srv_navi send_loc_data_server.py
    4. rosrun slamdog_srv_navi loc_file_reader_server.py
    5. Run Your Application
 
 ### 2) Application (blindguide)
 시각장애인의 원활한 목적지 설정을 위해 음성 안내를 지원하며 시각장애인이 많이 사용하는 Samsung Voice Assistant와 같이 사용할 수 있도록 구현하였다.
