# SlamDog
    Hardware : Turtlebot3 Burger 사용
&nbsp;
## SLAM
    내비게이션을 위한 지도 생성
    SLAM 과정에서 QR 코드를 스캔하여 목적지로 저장
    어플 : 로봇을 조종하고 데이터를 서버에 저장시킴

### android_apps/make_a_map
    안드로이드 스튜디오에서 구현
    기본적인 구조는 ROS의 오픈소스를 참고하였으나 앱에서 서버에 맵을 저장할 수 있는 기능 등을 추가로 구현
&nbsp;
## Navigation
    사용자가 설정한 목적지까지 안내
    
### android_apps/blindguide
    시각장애인을 위한 앱으로, 목적지 선택이 가능
    SLAM 과정에서 미리 저장해둔 목적지를 Navigation의 목적지로 설정할 수 있음
