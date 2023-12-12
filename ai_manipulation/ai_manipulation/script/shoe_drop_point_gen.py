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

from array import array
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

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

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
        global tvec
        wait_box_detection()
        random_values = [0] * 5
        random_values[0] = np.random.uniform(pose[0] - 0.5 * angle_gap, pose[0] + 0.5 * angle_gap)
        # random_values[0] = stats.norm.rvs(loc=pose[0] + ((xy_list[0][0] + xy_list[1][0]) / 2 - 320) * 0.001, scale=0.07 * angle_gap, size=1)[0]
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
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.goal_joint_state_publisher = self.create_publisher(Float64MultiArray, 'goal_joint_states', 10)
        self.goal_joint_state_publisher

        self.joint_state_subscription = self.create_subscription(
            Float64MultiArray,
            'present_joint_states',
            self.joint_state_callback,
            10)
        self.joint_state_subscription

    def send_goal_joint_space(self):
        msg = Float64MultiArray()
        msg.data = array('d', goal_joint_angle.copy())
        self.goal_joint_state_publisher.publish(msg)

    def joint_state_callback(self, msg):
        global present_joint_angle
        present_joint_angle = list(msg.data).copy()
        # print("I heard:", present_joint_angle)

def wait_box_detection():
    while (len(xy_list) != 2):
        time.sleep(0.5)

def wait_arrive():
    while True:
        tmp_dist = np.sqrt(np.sum(np.fromiter(((p2 - p1)**2 for p1, p2 in zip(goal_joint_angle[:5], present_joint_angle[:5])), dtype=float)))
        if tmp_dist <= 0.02:
            break
        time.sleep(0.1)

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    global tvec, xy_list, cam_activate, goal_joint_angle
    
    
    save_down_joint_angle = ("/home/hanse/git_ws/final_project/src/ai_manipulation/ai_manipulation/script/joint_angle_data")
    save_up_joint_angle = ("/home/hanse/git_ws/final_project/src/ai_manipulation/ai_manipulation/script/joint_angle_data")
    
    save_down_list = ("/home/hanse/git_ws/final_project/src/ai_manipulation/ai_manipulation/script")
    save_up_list = ("/home/hanse/git_ws/final_project/src/ai_manipulation/ai_manipulation/script")

    i_number = ("/home/hanse/git_ws/final_project/src/ai_manipulation/ai_manipulation/script/joint_angle_data")
    j_number = ("/home/hanse/git_ws/final_project/src/ai_manipulation/ai_manipulation/script/joint_angle_data")
    
    
    i = 1
    i_file_path = os.path.join(i_number, "i_number.json")
    if os.path.exists(i_file_path):
        with open(i_file_path, 'r') as i_file:
            i = json.load(i_file)
            
    j = 1
    j_file_path = os.path.join(j_number, "j_number.json")
    if os.path.exists(j_file_path):
        with open(j_file_path, 'r') as j_file:
            j = json.load(j_file)
    
    Down_list = []
    Up_list = []
    self = Agent() 

    # i_number (down) 이전 데이터 불러오기
    down_list_path = os.path.join(save_down_list, "Down_list.json")
    if os.path.exists(down_list_path):
        with open(down_list_path, 'r') as down_list_file:
            for line in down_list_file:
                if line.startswith("Down Joint Angle"):
                    point_data = json.loads(next(down_list_file))
                    Down_list.append(point_data)

    # j_number (up) 이전 데이터 불러오기
    up_list_path = os.path.join(save_up_list, "Up_list.json")
    if os.path.exists(up_list_path):
        with open(up_list_path, 'r') as up_list_file:
            for line in up_list_file:
                if line.startswith("Up Joint Angle"):
                    point_data = json.loads(next(up_list_file))
                    Up_list.append(point_data)




    rclpy.init()
    teleop_keyboard = TeleopKeyboard()
    t2 = threading.Thread(target=rclpy.spin, args=(teleop_keyboard,), daemon=True)
    t2.start()

    # Wait for spin
    while True:
        if present_joint_angle == [0., 0., 0., 0., 0., 0.]:
            time.sleep(0.01)
        else:
            break

    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    while(rclpy.ok()):
        key_value = get_key(settings)

        if key_value == 'd':
            joblib.dump(self.brain, model_backup)
            present_joint_angle_path = os.path.join(save_down_joint_angle, "Down_present_joint_angle.json")
            with open(present_joint_angle_path, 'w') as json_file:
                json.dump(present_joint_angle, json_file)
            # print(f"Down point {i} saved !")            

            Down_joint_list = os.path.join(save_down_joint_angle, "Down_present_joint_angle.json")
            with open(Down_joint_list, 'r') as down_json_file:
                read_down_list = json.load(down_json_file)
            Down_list.append(read_down_list)

            down_list_path = os.path.join(save_down_list, "Down_list.json")
            with open(down_list_path, 'w') as down_list_file:
                down_list_file.write("[")
                # json.dump(Down_list, down_list_file)

                # 줄바꿈과 번호메기기
                for idx, item in enumerate(Down_list, start=1):
                    json.dump(item, down_list_file)
                    down_list_file.write(", ")
                print(f" Down list {i} saved !")
                i += 1
                down_list_file.write("]")
            with open(i_file_path, 'w') as i_file:
                json.dump(i, i_file)

           
        if key_value == 'u':
            joblib.dump(self.brain, model_backup)
            present_joint_angle_path = os.path.join(save_up_joint_angle, "Up_present_joint_angle.json")
            with open(present_joint_angle_path, 'w') as json_file:
                json.dump(present_joint_angle, json_file)
            # print(f"Up point {i} saved !")            

            Up_joint_list = os.path.join(save_up_joint_angle, "Up_present_joint_angle.json")
            with open(Up_joint_list, 'r') as up_json_file:
                read_up_list = json.load(up_json_file)
            Up_list.append(read_up_list)

            up_list_path = os.path.join(save_up_list, "Up_list.json")
            with open(up_list_path, 'w') as up_list_file:
                up_list_file.write("[")
                # json.dump(Up_list, up_list_file)

                # 줄바꿈과 번호메기기
                for idx, item in enumerate(Up_list, start=1):                    
                    json.dump(item, up_list_file)
                    up_list_file.write(", ")
                print(f" Up list {j} saved !")
                j += 1
                up_list_file.write("]")
            with open(j_file_path, 'w') as j_file:
                json.dump(j, j_file)



        # if key_value == 'u':
        #     joblib.dump(self.brain, model_backup)
        #     present_joint_angle_path = os.path.join(save_up_joint_angle, "Up_present_joint_angle.json")
        #     with open(present_joint_angle_path, 'a') as json_file:
        #         json.dump(present_joint_angle, json_file)
        #     print(f"Up point {j} saved !")
        #     j += 1
        elif key_value == 'q':
            break
   
    # print("{}th point generation:")
    
    # keyboard input : D

    # way_point_1 = present_joint_angle
    # print( {}th point saved! )

    # keyboard input : U

    # way_point_2 = present_joint_angle
    # print( {}th point saved! )

    teleop_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()