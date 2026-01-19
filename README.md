# ROS2 주행 프로젝트
Python 기반의 a* + mppi + llm을 활용한 프로젝트입니다.

## 코드 실행 방법
### ros2 humble을 활성화합니다.  
$ source /opt/ros/humble/setup.bash  
### 터틀봇3의 bringup 노드를 활성화합니다.  
$ export TURTLEBOT3_MODEL=waffle_pi  
$ ros2 launch turtlebot3_bringup robot.launch.py  
### 터틀봇3의 카메라 노드를 활성화합니다.  
$ ros2 run v4l2_camera v4l2_camera_node  
### ros 패키지로 이동해서 빌드를 합니다.  
$ cd ros2_drive_team_project  
$ colcon build  
### 빌드 후 source 적용을 합니다.  
$ source install/setup.bash  
### 런치 파일을 실행합니다.  
$ ros2 launch my_robot_pkg real.launch.py
