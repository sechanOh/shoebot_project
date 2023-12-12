<img src="https://github.com/addinedu-ros-2nd/robot-repo-2/assets/140477778/86723d3a-4c09-41ea-a812-f5b6df4cb52a" width= "110%" height="110%"/>


# Why?
<img src="https://github.com/addinedu-ros-2nd/robot-repo-2/assets/140477778/6d90a1c4-a6e6-44a0-bfa7-9022e40e8950" width="70%" height="70%"/>


직접 손으로 정리하시겠습니까?

아니면 shoebot이 정리해드릴까요?

## Project Introduce


<img src="https://github.com/addinedu-ros-2nd/robot-repo-2/assets/140477778/cc05d8f4-d8a5-4d24-9b5d-f5cf5640731c" width="70%" height="70%"/>

shoebot은 당신의 손이 없어도 신발을 정리해주는 친절한 로봇입니다.

# Demo Video(이미지 클릭시 유튜브로 연결됩니다)


[![image](https://github.com/addinedu-ros-2nd/robot-repo-2/assets/140477778/d7c48a46-50a9-424f-b440-a319a1571943)](https://www.youtube.com/watch?v=70jTAGOszJk)


## Project Keyword

**1. Manipulator pick & place**
   
**2. Face Recognition**
   
**3. Pose Estimation with LSTM & CNN**
   
**4. Autonomous Driving with SLAM**

**5. Main Control System with ROS2**

   
## Project Summary


**1. Manipulator pick & place**

      YOLOv3로 학습한 신발객체 인식 모델을 활용하여 manipulator가 신발을 집고, 신발장에 정리해줍니다.

      이때, manipulator은 MoveIt와 MLP model based DeepLearning로 정밀하게 제어됩니다.


**2. Face Recognition**

      Face recognition으로 제공하는 고유ID는 똑같은 신발들 사이에서도 여러분의 신발을 찾을 수 있게 도와줍니다.


**3. Pose Estimation with LSTM & CNN**

      LSTM 과 CNN으로 학습된 행동예측 모델은 ShoeBot이 신발을 정리하면서도 여러분과 부딪히지않게 해줍니다.

      당신이 걷던, 뛰던, 혹은 물구나무를 서더라도요.

**4. Autonomous Driving with SLAM**

      manipulator가 여러분의 손을 대신해준다면, SLAM을 기반으로 구축한 Automous Driving은 여러분의 발이 되어줄겁니다.

      다만, 여러분들이 찾지않는다면 ShoeBot은 충전소에서 쉬고있겠죠

**5. Main Control System with ROS2**

      ShoeBot에게는 든든한 친구 ShoeManager가 있습니다.

      Shoemanager은 여러분이 도움을 필요로 할때마다 ShoeBot에게 귀뜀을 해주지요. 물론 그 반대도 가능하도록 도와줍니다!

      Pose estimation으로 행동예측을 하여 Mobile robot을 회피기동
## Scenario & System Diagram

<img src="https://github.com/addinedu-ros-2nd/robot-repo-2/assets/140477778/5a99e0b4-a979-45f4-9dea-5e57a060ab19" width="40%" height="40%"/>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp

<img src="https://github.com/addinedu-ros-2nd/robot-repo-2/assets/140477778/f5ffd09c-155c-4951-af13-9e8d26a2af4c" width="40%" height="40%"/>



## Manipulator Motion Planning



### Motion Planning Part Summary
      manipulator가 신발을 감지 한 후에 신발을 집고 주행로봇이나 신발장에 올리기 위해 필요한 동작을 구현하는 단계


### Motion Planning Part technology 

#### Custom cpp package 

      1. 기존에는 5-DOF만 제어가능한 패키지를 DOF에 관계없이 제어할 수 있도록 별도로 custom
      2. dynamic cell control table 을 확인 할 수 있도록 기능 추가
      3. ROS2 Humble에서 실행이 가능하도록 custom

#### YOLOv3 신발객체탐지 모델

      1. webcam으로 촬영한 약 450개의 모형신발 사진을 사용하여 YoloV3 학습모델을 생성
      2. manipulator의 gripper에 동일한 webcam을 장착하여 모형신발을 탐지

<img src="https://github.com/addinedu-ros-2nd/robot-repo-2/assets/140477778/d2621362-ff64-4c15-aaed-f4ba40c0d72b" width="40%" height="40%"/>

#### 신발위치추정 모델

      1. shoe pick & place 를 위해서 모형신발의 위치를 추정할 수 있는 모델을 구축
         a. 다양한 위치에 모형신발을 두고, 해당 위치에 신발을 집을수 있는 pose를 설정한다. 이때의 gripper를 제외한 5개의 관절 상태를 기록해둔다.
         b. 특정 위치의 모형신발을 집을때 gripper의 상태 와 YoloV3로 탐지하는 모형신발의 bounding box의 중심좌표를 미리 기록해둔 5개의 관절 상태와 매칭시킨다.
         c. b를 반복하여 생성한 데이터셋을 기반으로 deeplearning 모델을 생성한다.

      

#### MoveIt를 활용한 manipulator 제어

      1. ROS1의 MoveIt로 manipulator path planning 
      (moveit 사진 첨부 예정)



### Motion Planning Part Software & Hardware

#### Software
      * ROS1 & 2
      * YoloV3
      
|Language|Python|C++|
|---|---|---|
|Library|rclpy,pathlib, torch, cv2,ai_manipulation,sklearn.linear_model|rclcpp,Dynamicxel,chrono|

#### Hardware
<img src="https://github.com/addinedu-ros-2nd/robot-repo-2/assets/140477778/05088ef2-2538-4654-9fcd-4d721e202420" width="40%" height="40%"/>





## Face Recognition



## Pose Estimation
   모바일 로봇이 이동시 사람을 인식 후 부딪히지 않도록 행동을 예측하는 모델
   
   YOLOv8n-pose를 사용하여 사람인식 후 Skeleton Keypoints을 추출
   
   Keypoints값 17개는 Skeleton COCO Pose를 기반으로 사용


<h3 align="center">
Models
</h3>


---
**1. Decision Tree**

![DecisionTree](https://github.com/addinedu-ros-2nd/robot-repo-2/assets/132260442/a2118e91-d5fd-491e-8b29-bb1d73c70438)

**2. LSTM(Long Short Term Memory)**

![LSTM](https://github.com/addinedu-ros-2nd/robot-repo-2/assets/132260442/ec9109fa-fbd0-4c6d-972d-f579dab60076)

## Mobile Robot
-----
# Improvements expected in the future

**아쉬웠던 점, 향후 개선점 기술예정**

# Member
|성함|기술 담당|GITHUB|
|---|---|---|
|**오세찬**|manipulator control ROS2 cpp package 작성<br/><br/>  Human 3D trajectory estimation system 구현|---|
|**김동우**|2D point & shoot SLAM 맵 구현 및 Mobile robot 제어<br/><br/>  shoe pick & place를 위한 joint angle 데이터 생성 및 딥러닝 모델을 이용한 데이터 증강|[링크](https://github.com/DongUKim)|Face recognition library를 활용한 얼굴인식 및 ID 관리 system 구현<br/><br/>  YOLOv8-Pose, LSTM, Tensor Flow의 CNN을 활용하여 Human Pose estimation 을 위한 LSTM 모델 구현|---|
|**김명섭**|Face recognition library를 활용한 얼굴인식 및 ID 관리 system 구현<br/><br/>  YOLOv8-Pose, LSTM, Tensor Flow의 CNN을 활용하여 Human Pose estimation 을 위한 LSTM 모델 구현|---|
|**백세진**|PyQT를 활용하여 GUI생성, ROS연결 통신 <br/><br/>   GUI와 Dashboard를 만들어서 전반적인 상황에 관제 시스템을 만든 모델을 구현했음|---|
|**최석원**|shoe detection를 위한 YOLOv3 학습모델 구축<br/><br/>   YOLOv8-Pose,Tensor Flow를 활용하여 Human Pose estimation 을 위한 decision tree모델과 LSTM 모델 구현|---|
|**왕한세**|shoe detection를 위한 YOLOv3 학습모델 구축<br/><br/>   MoveIt을 활용한 6-DOF manipulator URDF 작성 및 제어<br/><br/>   Digital Twin|---|
