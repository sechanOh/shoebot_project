<img src="https://github.com/addinedu-ros-2nd/robot-repo-2/assets/140477778/86723d3a-4c09-41ea-a812-f5b6df4cb52a" width= "110%" height="110%"/>

## 1. 프로젝트 요약
### 1-1. 설명
식당 등에서 신발을 분실하는 경우가 있다. 로봇이 사람 대신 신발을 관리해 줄 수 있다면 분실의 위험성을 줄일 수 있다.

본 프로젝트에서는 손님의 얼굴을 인식하고 ID를 부여하여 입장 및 퇴장을 감지할 수 있다. 신발을 잡기 위해 6 자유도를 가진 Manipulator를 제작하고, 딥러닝 기반 motion planning을 통해 신발을 pick 하는 기능을 구현하였다. 또한, LiDAR SLAM 및 Navigation을 통해 Autonomous Mobile Robot이 신발장까지 자율주행을 할 수 있도록 구현하였다. 이 모든 기능의 진행 상황을 관리자가 그래픽으로 확인할 수 있도록 GUI 프로그램도 제작하였다.
### 1-2. 개발 규모
- 인원 : 6명
- 일시 : 2023.08 ~ 2023.11
### 1-3. 개발환경
- Ubuntu 22.04 Desktop
- ROS2 Humble
- C++ Library : OpenMANIPULATOR SDK, DYNAMIXEL SDK
- Python Library : Scikit-learn, slam_toolbox, Yolov3, opencv-python, PyQt5
### 1-4. 하드웨어
- Raspberry pi 4B
- Arduino Mega
- YDLIDAR X2
- OpenMANIPULATOR-X
- DYNAMIXEL XM430-W350-T
- mono usb camera

## 2. 프로젝트 구현
### 2-1. 데모 영상
[![youtube playlist](http://img.youtube.com/vi/AA3_XLoDVm0/0.jpg)](https://www.youtube.com/playlist?list=PLx5EbqT-6Y09LVs4YXfSDFbNwMVvL6dq-)
### 2-2. 

# Member
|성함|기술 담당|GITHUB|
|---|---|---|
|**오세찬**|manipulator control ROS2 cpp package 작성<br/><br/>  Human 3D trajectory estimation system 구현|[링크](https://github.com/Ohsechan)|
|**김동우**|2D point & shoot SLAM 맵 구현 및 Mobile robot 제어<br/><br/>  shoe pick & place를 위한 joint angle 데이터 생성 및 딥러닝 모델을 이용한 데이터 증강|[링크](https://github.com/DongUKim)|Face recognition library를 활용한 얼굴인식 및 ID 관리 system 구현<br/><br/>  YOLOv8-Pose, LSTM, Tensor Flow의 CNN을 활용하여 Human Pose estimation 을 위한 LSTM 모델 구현|---|
|**김명섭**|Face recognition library를 활용한 얼굴인식 및 ID 관리 system 구현<br/><br/>  YOLOv8-Pose, LSTM, Tensor Flow의 CNN을 활용하여 Human Pose estimation 을 위한 LSTM 모델 구현|---|
|**백세진**|PyQT를 활용하여 GUI생성, ROS연결 통신 <br/><br/>   GUI와 Dashboard를 만들어서 전반적인 상황에 관제 시스템을 만든 모델을 구현했음|---|
|**최석원**|shoe detection를 위한 YOLOv3 학습모델 구축<br/><br/>   YOLOv8-Pose,Tensor Flow를 활용하여 Human Pose estimation 을 위한 decision tree모델과 LSTM 모델 구현|---|
|**왕한세**|shoe detection를 위한 YOLOv3 학습모델 구축<br/><br/>   MoveIt을 활용한 6-DOF manipulator URDF 작성 및 제어<br/><br/>   Digital Twin|---|
