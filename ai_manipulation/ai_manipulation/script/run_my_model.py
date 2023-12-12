from math import exp
import os
import rclpy
import select
import sys
import threading

import numpy as np
from multiprocessing import Process
import argparse
import os
import platform
import sys
from pathlib import Path
import torch
import cv2
import time

from ultralytics.utils.plotting import Annotator, colors, save_one_box

from ai_manipulation.utils.models.common import DetectMultiBackend
from ai_manipulation.utils.utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from ai_manipulation.utils.utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from ai_manipulation.utils.utils.torch_utils import select_device, smart_inference_mode
from ai_manipulation.utils.utils.utils import ARUCO_DICT

import joblib
import json
import random
from sklearn.linear_model import LinearRegression
import scipy.stats as stats

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

import subprocess

package_name = 'ai_manipulation'
output = subprocess.check_output(["ros2", "pkg", "prefix", package_name], text=True)
workspace_path = output.split("/install")[0]

# Params
states_backup = os.path.join(workspace_path, "src", package_name, package_name, "datas/openmanipulator_states.json")
acts_backup = os.path.join(workspace_path, "src", package_name, package_name, "datas/openmanipulator_acts.json")
model_backup = os.path.join(workspace_path, "src", package_name, package_name, "datas/openmanipulator_weight.pkl")
k_path = os.path.join(workspace_path, "src", package_name, package_name, "utils/calibration_matrix.npy")
d_path = os.path.join(workspace_path, "src", package_name, package_name, "utils/distortion_coefficients.npy")
random_state_gen_num = 16
angle_gap_origin = 0.09  # radian
angle_gap = 0.07
path_time = 0.4  # second
state_size = 6
action_size = 4
threshold = 0.11 # scenario ending thres
INF = 999999999999

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

e = """
Communications Failed
"""

def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)
    # print(f"ids : {ids}")
    # If markers are detected
    tvec = [[[INF, INF, INF]]]
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # print("coners :", corners[i])
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            # print(f"rotation vector : {rvec}")
            # print(f"translation vector : {tvec.shape}\n")
            # distance = np.sqrt(tvec[0][0][0]**2 + tvec[0][0][1]**2 + tvec[0][0][2]**2)

            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners)

    return tvec[0][0]

@smart_inference_mode()
def run(
        weights=os.path.join(workspace_path, "src", package_name, package_name, "utils/best.pt"),  # model path or triton URL
        source="2",  # file/dir/URL/glob/screen/0(webcam)
        imgsz=(640, 480),  # inference size (height, width)
        conf_thres=0.3,  # confidence threshold
        iou_thres=0.3,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        vid_stride=1,  # video frame-rate stride
):
    global tvec, xy_list, cam_activate
    source = str(source)
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.streams') or (is_url and not is_file)
    screenshot = source.lower().startswith('screen')
    if is_url and is_file:
        source = check_file(source)  # download

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    view_img = check_imshow(warn=True)
    dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
    bs = len(dataset)
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(), Profile(), Profile())
    for path, im, im0s, vid_cap, s in dataset:
        with dt[0]:
            im = torch.from_numpy(im).to(model.device)
            im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

        # Inference
        with dt[1]:
            visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            pred = model(im, augment=augment, visualize=visualize)

        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        # 탐지된 물체 정보 처리
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size가 1 이상인 경우
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)
            
            # aruco marker tvec
            aruco_dict_type = ARUCO_DICT["DICT_5X5_100"]
            k = np.load(k_path)
            d = np.load(d_path)
            tvec = pose_esitmation(im0, aruco_dict_type, k, d)

            p = Path(p)  # 경로를 Path 객체로 변환
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            
            xy_list = []
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to strin
               

                def map_value(value, from_min, from_max, to_min, to_max):
                    from_range = from_max - from_min
                    to_range = to_max - to_min
                    scaled_value = (value - from_min) / from_range
                    mapped_value = to_min + (scaled_value * to_range)
                    return mapped_value


                # cls 변수는 클래스 인덱스를 나타내며, names 리스트에서 클래스 이름을 가져옵니다
                for *xyxy, conf, cls in reversed(det):
                    if view_img:  # 이미지에 바운딩 박스 그리기
                        c = int(cls)  # 정수형 클래스q
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}') # 라벨 표시 여부
                        annotator.box_label(xyxy, label, color=colors(c, True)) # 이미지에 바운딩 박스와 라벨 추가
                        if names[c] == 'shoe_lace' :  # 원하는 클래스 리스트
                            tmp_list = torch.tensor(xyxy).view(1, 4).view(-1).tolist()

                            x_pixel = np.mean([tmp_list[0], tmp_list[2]])
                            y_pixel = np.mean([tmp_list[1], tmp_list[3]])
                                                
                            # print(f"{names[c]} Center (x, y): {x_pixel:.2f}, {y_pixel:.2f} \n")
                            xy_list.append((x_pixel, y_pixel))

            cam_activate = True

            # Stream resultscomplete
            im0 = annotator.result()
            if view_img:
                # im0 = cv2.flip(im0, -1)  # -1은 상하좌우 반전을 의미합니다

                if platform.system() == 'Linux' and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

class Agent():
    def __init__(self):
        self.states             = []
        self.acts               = []
        self.state_size         = state_size
        self.action_size        = action_size
        self.learning_rate      = 0.001
        self.brain              = self._build_model()
    
        if os.path.isfile(states_backup):
            with open(states_backup, 'r') as json_file:
                self.states = json.load(json_file)
        if os.path.isfile(acts_backup):
            with open(acts_backup, 'r') as json_file:
                self.acts = json.load(json_file)

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        model = LinearRegression()

        if os.path.isfile(model_backup):
            model = joblib.load(model_backup)
        return model

    def backup(self):
        joblib.dump(self.brain, model_backup)
        with open(states_backup, 'w') as json_file:
            json.dump(self.states, json_file)
        with open(acts_backup, 'w') as json_file:
            json.dump(self.acts, json_file)

    def generate_random_pose(self, pose, prob_mean, bad_pose):
        global xy_list
        wait_box_detection()
        random_values = [0] * 4
        random_values[0] = np.random.uniform(pose[0] - 0.5 * angle_gap, pose[0] + 0.5 * angle_gap)
        random_values[0] = stats.norm.rvs(loc=pose[0] + ((xy_list[0][0] + xy_list[1][0]) / 2 - 320) * 0.001, scale=0.07 * angle_gap, size=1)[0]
        # print(((xy_list[0][0] + xy_list[1][0]) / 2 - 320) * 0.01)
        while True:
            time.sleep(0.03)
            not_bad = True
            for idx in range(1,3):
                if prob_mean[idx] == pose[idx]:
                    random_values[idx] = np.random.uniform(pose[idx] - angle_gap, pose[idx] + angle_gap)
                else:
                    while True:
                        tmp = stats.norm.rvs(loc=prob_mean[idx], scale=0.4 * angle_gap, size=1)[0]
                        if (pose[idx] - angle_gap < tmp and tmp < pose[idx] + angle_gap):
                            break
                    random_values[idx] = tmp
            # suppress unwanted camera angle movement
            random_values[3] = np.random.uniform(pose[3] - 1.0 * angle_gap, pose[3])
            for each in bad_pose:
                if np.linalg.norm(np.array(random_values) - np.array(each)) < 0.02:
                    prob_mean = pose.copy()
                    not_bad = False
            if not_bad:
                break
        return random_values
    
    def remember(self, state, act):
        self.states.append(state)
        self.acts.append(act)
    
    def learn(self):
        self.brain.fit(self.states, self.acts)
    
    def predict(self, state):
        return self.brain.predict(state)

class TeleopKeyboard(Node):
    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('teleop_keyboard')
        key_value = ''

        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        try:
            self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self, path_time):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.goal_joint_space_req.path_time = path_time

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.tool_control_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.tool_control_req.path_time = path_time

        try:
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)

        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4]

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 5):
                goal_joint_angle[index] = present_joint_angle[index]

def wait_box_detection():
    while (len(xy_list) != 2):
        # print("Detection Failed!", len(xy_list))
        time.sleep(0.5)

def wait_arrive():
    time.sleep(0.1)
    while True:
        tmp_dist = np.sqrt(np.sum(np.fromiter(((p2 - p1)**2 for p1, p2 in zip(goal_joint_angle[:4], present_joint_angle[:4])), dtype=float)))
        if tmp_dist == 0:
            break
    time.sleep(0.1)

def main():
    global tvec, xy_list, cam_activate

    # Camera start
    cam_activate = False
    t1 = threading.Thread(target=run, daemon=True)
    t1.start()

    agent = Agent()

    try:
        # wait for camera
        while True:
            if cam_activate:
                break
            time.sleep(0.5)

        rclpy.init()
        teleop_keyboard = TeleopKeyboard()

        t2 = threading.Thread(target=rclpy.spin, args=(teleop_keyboard,), daemon=True)
        t2.start()
        time.sleep(0.1) # initializing time
        prev_goal_joint_angle = [0.03221359848976135, -0.5783107876777649, 0.1457281857728958, 1.902136206626892, 0.023769033551216123]
        initial_kinematics_pose = [0.08751707100859887, 0.0024335184297788323, 0.12362461889275572]
        
        # initial pose
        goal_joint_angle[:4] = [0.03221359848976135, -0.5783107876777649, 0.1457281857728958, 1.902136206626892]
        teleop_keyboard.send_goal_joint_space(3.0)
        time.sleep(3)

        for i in range(20):
            print(i, "th step")

            # calc box mid
            while (len(xy_list) != 2):
                # print("Detection Failed!")
                pass
            mid_x = (xy_list[0][0] + xy_list[1][0]) / 2
            mid_y = (xy_list[0][1] + xy_list[1][1]) / 2

            # make pose & state
            pose = present_joint_angle[:4].copy()
            state = pose + [mid_x, mid_y]

            state = np.array(state).reshape(1, -1)
            nxt_pose = agent.predict(state)[0]
            goal_joint_angle[:4] = nxt_pose.copy()
            teleop_keyboard.send_goal_joint_space(path_time)
            wait_arrive()

    finally:
        pass

if __name__ == '__main__':
    main()
