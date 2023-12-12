from math import exp
import os
import rclpy
import select
import sys
import threading

import random
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
import json

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
states_backup = os.path.join(workspace_path, "src", package_name, package_name, "datas/six_dxl_states.json")
acts_backup = os.path.join(workspace_path, "src", package_name, package_name, "datas/six_dxl_acts.json")
model_backup = os.path.join(workspace_path, "src", package_name, package_name, "datas/six_dxl_model.pkl")
k_path = os.path.join(workspace_path, "src", package_name, package_name, "utils/calibration_matrix.npy")
d_path = os.path.join(workspace_path, "src", package_name, package_name, "utils/distortion_coefficients.npy")
way_point_1_path = os.path.join(workspace_path, "src", package_name, package_name, "datas/way_point_1.json")
way_point_2_path = os.path.join(workspace_path, "src", package_name, package_name, "datas/way_point_2.json")
manipulator_angle_range_path = os.path.join(workspace_path, "src", package_name, package_name, "datas/manipulator_angle_range.json")

with open(manipulator_angle_range_path, 'r') as json_file:
    manipulator_angle_range = json.load(json_file)
random_state_gen_num = 16
# angle_gap_origin = 0.09  # radian
# angle_gap = 0.1
# path_time = 0.4  # second
state_size = 9
action_size = 5
# threshold = 0.11 # scenario ending thres
INF = 999999999999

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
search_start_point = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
        weights=os.path.join(workspace_path, "src", package_name, package_name, "utils/best_last.pt"),  # model path or triton URL
        source="0",  # file/dir/URL/glob/screen/0(webcam)
        imgsz=(640, 480),  # inference size (height, width)
        conf_thres=0.1,  # confidence threshold
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
    global tvec, xy_shoelace, xy_shoe, cam_activate, x_pixel, y_pixel
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
            
            xy_shoelace, xy_shoe = [], []
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

                        tmp_list = torch.tensor(xyxy).view(1, 4).view(-1).tolist()
                        x_pixel = np.mean([tmp_list[0], tmp_list[2]])
                        y_pixel = np.mean([tmp_list[1], tmp_list[3]])
                        if names[c] == 'ShoeLace' :
                            xy_shoelace.append((x_pixel, y_pixel))
                        if names[c] == 'point' :
                            xy_shoe.append((x_pixel, y_pixel))

            cam_activate = True

            # Stream resultscomplete
            im0 = annotator.result()
            if view_img:
                im0 = cv2.flip(im0, -1)  # -1은 상하좌우 반전을 의미합니다

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
            print(len(self.states), "states loaded")
        if os.path.isfile(acts_backup):
            with open(acts_backup, 'r') as json_file:
                self.acts = json.load(json_file)
            print(len(self.acts), "acts loaded")

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        model = LinearRegression()

        if os.path.isfile(model_backup):
            model = joblib.load(model_backup)
            print("Model loaded")
        return model

    def backup(self):
        joblib.dump(self.brain, model_backup)
        with open(states_backup, 'w') as json_file:
            json.dump(self.states, json_file)
        with open(acts_backup, 'w') as json_file:
            json.dump(self.acts, json_file)

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

def return_box_detection():
    global xy_shoelace, xy_shoe
    for i in range(200):
        if ((len(xy_shoelace) != 2) or (len(xy_shoe) != 2)):
            time.sleep(0.001)
            # print("pass!")
        else:
            result = [xy_shoelace, xy_shoe]
            return result
    return None
searching_point=0


def gripper_searching_action():
    global xy_shoe, xy_shoelace, teleop_keyboard, searching_point, way_point_1, center_x, center_y, tmp_xy, fail_list, success_count
    gripper_searching_start_point = [0.13345633447170258, -0.4862719178199768, 0.04601942375302315, 2.1491072177886963, -1.1, -1.3606410026550293] # 그립퍼 서칭 시작
    x_lower, x_upper, y_lower, y_upper = 200, 550, 125, 350    #### 두번째 300, 340, 225, 250  ### 맨 첫 기준 317, 323, 237, 243
    # goal_joint_angle[4] = -1.1
    # teleop_keyboard.send_goal_joint_space()
    
    agent = Agent()
    # way_point_2 = json.load(json_file)
    
    move_softly_to(gripper_searching_start_point)
    wait_arrive(0.1)
    time.sleep(2)
    print("Searching Start")
    # 바운딩 박스가 4개가 생성되면 success_count가 +1
    success_count = 0
    while True:
        
        goal_joint_angle[4] += 0.01
        teleop_keyboard.send_goal_joint_space()
        wait_arrive(0.05)

        
        print("shoe's box :", len(xy_shoe),"shoelace's box : ", len(xy_shoelace) )
        print("roll is", present_joint_angle[4])
        
        # gripper 의 각도가 특정각만큼만 searching
        if (present_joint_angle[4] < 1.25):
            
            # 바운딩 박스가 각각 2개씩일때만 좌표 기록
            if (len(xy_shoe) == 2 and len(xy_shoelace) == 2):
                
                result = [xy_shoelace, xy_shoe]
                tmp_xy = xy_shoe.copy()
                # print("detect two object, move x value")
                center_x = abs((tmp_xy[0][0] + tmp_xy[1][0])/2)
                # center_y = abs((tmp_xy[0][1] + tmp_xy[1][1])/2)
                # x_lower, x_upper, y_lower, y_upper = 200, 600, 125, 350 
                
                
                print("center_x = ", center_x)
                if (x_lower < center_x and center_x < x_upper): # and y_lower < center_y and center_y < y_upper
                    time.sleep(0.5)
                    success_count += 1
                    tmp_shoelace, tmp_shoe = result
                    
                    # if successs, remember this state
                    print("Point Checked! I will remember this point")
                    print(f"Now shoebot learned Points {success_count} times")
                    state = [present_joint_angle[4]] + list(tmp_shoelace[0]) + list(tmp_shoelace[1]) + list(tmp_shoe[0]) + list(tmp_shoe[1])
                    agent.remember(state, way_point_2[episode][:5])
                    print("!!!!!!!!!OK! Save Information about Joint angle and X,Y Position!!!!!!!!!!!!!!!")
                    # return result
                
        else:
            if success_count == 0:
                print(f"oops...{fail_list} episode failed!")
                return success_count, fail_list
                         
            else:
                print(f"Ok! {episode + 1} episode complete searching {success_count} points!!")
                agent.learn()
                agent.backup()
                return success_count, fail_list
        # print("result is ", result)
                   

def get_distance(list1, list2):
    return np.sqrt(np.sum(np.fromiter(((p2 - p1)**2 for p1, p2 in zip(list1, list2)), dtype=float)))

def wait_arrive(thres = 0.1):
    while True:
        tmp_dist = get_distance(goal_joint_angle[:5], present_joint_angle[:5])
        # print(tmp_dist)
        if tmp_dist < thres:
            break
        time.sleep(0.01)

def move_softly_to(goal_point, mul = 12):
    global teleop_keyboard
    start_point = present_joint_angle[:5]
    full_step = int(get_distance(start_point, goal_point) * mul)
    for step in range(1, full_step+1):
        goal_joint_angle[:5] = [(step * x + (full_step - step) * y)/full_step for x, y in zip(goal_point, start_point)]
        teleop_keyboard.send_goal_joint_space()
        wait_arrive(0.1)

def generate_random_pose(pose, angle_gap = 0.1):
    random_values = [0] * 5
    for idx in range(0,5):
        if idx == 3:
            continue
        random_values[idx] = np.random.uniform(pose[idx] - angle_gap, pose[idx] + angle_gap)
    random_values[3] = np.random.uniform(pose[3] - angle_gap, pose[3] + 0.5 * angle_gap)
    return random_values

# ## 한세 코드 ##
# def generate_random_pose(pose, angle_gap = 0.1):
#     global xy_shoelace, xy_shoe, x_pixel, y_pixel
#     random_values = [0] * 5

#     if (x_pixel > xy_shoelace and y_pixel > xy_shoelace ):
        




    # 
    # for idx in range(0,5):
    #     if idx == 3:
    #         continue
    #     random_values[idx] = np.random.uniform(pose[idx] - angle_gap, pose[idx] + angle_gap)
    # random_values[3] = np.random.uniform(pose[3] - angle_gap, pose[3] + 0.5 * angle_gap)
    # return random_values

    
    

def check_manipulator_angle(pose):
    for idx, angle_range in enumerate(manipulator_angle_range):
        if (pose[idx] > angle_range[0] or pose[idx] < angle_range[1]):
            print("angle overload")
            return False
    return True

def main():
    global tvec, xy_shoelace, xy_shoe, cam_activate, goal_joint_angle, teleop_keyboard, way_point_1, fail_list, success_count, episode  

    fail_list = []
    # Camera start
    cam_activate = False
    t1 = threading.Thread(target=run, daemon=True)
    t1.start()

    agent = Agent()

    rclpy.init()
    teleop_keyboard = TeleopKeyboard()
    t2 = threading.Thread(target=rclpy.spin, args=(teleop_keyboard,), daemon=True)
    t2.start()
    # Wait for spin
    while True:
        if present_joint_angle == [0., 0., 0., 0., 0., 0.]:
            time.sleep(0.1)
        else:
            break
    # Check dxl count
    if len(present_joint_angle) != 6:
        print("DXL missing!\n DXL count :", len(present_joint_angle))
        teleop_keyboard.destroy_node()
        rclpy.shutdown()
        return
    
    # wait for camera
    while True:
        if cam_activate:
            break
        time.sleep(0.5)
    # search_start_point = [-1.0, -0.8866409063339233, 0.1395922601222992, 2.032524585723877, 0., -1.1520196199417114]
    # search_start_point = [-1.0, -1.3115535974502563, 0.7470486760139465, 1.8208352327346802, 0, -1.0983302593231201] # 10/24 백업
    # search_start_point = [-1.0, -0.915535974502563, 0.3470486760139465, 2.2208352327346802, 0, -1.0983302593231201] # 10/27 백업 서칭 액션_1 포인트 
    # search_start_point = [-0.9, -0.2929903268814087, 0.0, 2.2073984146118164, 0, -1.0983302593231201] # 10/27 서칭 두번 하는 코드
    search_start_point = [0.13345633447170258, -0.4862719178199768, 0.04601942375302315, 2.1491072177886963, 0.11044661700725555, -1.3606410026550293] # 서칭 액션 없이 고정 위치
    #search_start_point2 = [1.0, -0.3344078063964844, -0.013805827125906944, 1.9159420728683472, 0, -1.0983302593231201]
    gripper_open, gripper_close = -1.100485634803772, 0.3093204975128174
    shoes_check_point = [0.02454369328916073, -0.2316311001777649, 0.21475732326507568, -0.28378644585609436, 0.02147573232650757, -0.8574953079223633]

    goal_joint_angle = present_joint_angle.copy()

    # Read JSON
    with open(way_point_1_path, 'r') as json_file:
        way_point_1 = json.load(json_file)
    with open(way_point_2_path, 'r') as json_file:
        global way_point_2
        way_point_2 = json.load(json_file)

    # Gripper Close
    goal_joint_angle[5] = gripper_close
    teleop_keyboard.send_goal_joint_space()
    wait_arrive(0.05)
    
    ## Check Gripper picks shoes
    # move_softly_to(shoes_check_point)
    # wait_arrive(5)
    # time.sleep(2)
    

    
    fail_list = []
    
    
    # Move to Way Point

    for idx, point in enumerate(way_point_1):
        episode = idx
        print ("Episode", episode + 1 , "Studying")
        # print("Episode", idx + 1 + episode_start, "/", len(way_point_1), "Start!")
        # print ("Episode", 1 + episode_start , "~", len(way_point_1[episode_start:]), "Studying")
        # print("Episode", idx + 1 + episode_start, "/", len(way_point_1[episode_start:]), "Start!")
        
        
        #### Initial Position Setting #####
        move_softly_to(way_point_2[episode][:5])
        move_softly_to(way_point_1[episode][:5])
        # Gripper Open
        print("open?")
        goal_joint_angle[5] = gripper_open
        teleop_keyboard.send_goal_joint_space()
        wait_arrive(0.05)
        time.sleep(1) 
        print("open!")
        # Go to way2
        move_softly_to(way_point_2[episode][:5])

        # Go Searching point 
        move_softly_to(search_start_point)
        wait_arrive(0.05)
        time.sleep(1)
        

        
        
        print(" Hmm.. Where is shoes? Searching Start !")
        result = gripper_searching_action()
        
            
        # result = return_box_detection()

        
        # # 기다리는 동안 조건 확인
        # while not (len(xy_shoe) == 2 and len(xy_shoelace) == 2):
        #     gripper_searching_action()
        #     pass  # 조건이 만족될 때까지 아무 작업도 하지 않음

        # # go to way_point
        # if len(xy_shoe) == 2 and len(xy_shoelace) == 2:
        # Go to way2
        move_softly_to(way_point_2[episode][:5])
        # Go DOWN
        move_softly_to(way_point_1[episode][:5])
        wait_arrive()
        # Gripper Close
        goal_joint_angle[5] = gripper_close
        teleop_keyboard.send_goal_joint_space()
        wait_arrive(0.05)
        time.sleep(1)
        # Go UP
        
        log_file = "/home/kimdu/hanse/src/ai_manipulation/ai_manipulation/script/log_left_again.txt"
        if result != None:
            with open(log_file, 'a') as file:
                file.write(f"\n {episode + 1} episode succeed {success_count} times!")
                # file.write(f"Now {fail_list} episodes are failed... So Sad")
            
        else:
            fail_list.append(episode + 1)
            print("this episode is failed!")
            with open(log_file, 'a') as file:
                # file.write(f"{episode} episode succeed {success_count} times! \n")
                file.write(f"\n Now {fail_list} episode is failed... So Sad")
            

    # Final action
    move_softly_to(way_point_1[0][:5])
    # Gripper Open
    goal_joint_angle[5] = gripper_open
    teleop_keyboard.send_goal_joint_space()
    wait_arrive(0.05)

    # agent.learn()
    # agent.backup()
    print("Data generation finished!")

    # Destroy Node
    teleop_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()