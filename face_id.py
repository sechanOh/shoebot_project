import rclpy as rp
import cv2
import face_recognition as fr
import numpy as np
import os

from rclpy.node import Node
from std_msgs.msg import Int32
from cv_bridge import CvBridge

bridge = CvBridge()
face_encode_dict = {"dumy" : np.ndarray(128,)}
class PublishSubscribeNode(Node):
    def __init__(self):
        super().__init__('FaceID_pub_node')
        global face_encode_dict
        self.publisher = self.create_publisher(Int32, 'FaceID', 10)
        # self.subscription = self.create_subscription(Image, '/image_raw', self.callback, 10)
        time_period = 0.01
        self.timer = self.create_timer(time_period, self.callback)
        
        self.cap = cv2.VideoCapture(0)
        
        model = './../res10_300x300_ssd_iter_140000_fp16.caffemodel'
        config = './../deploy.prototxt'
        self.net = cv2.dnn.readNet(model, config)
        self.dnn_confidence = 0.95
        self.face_file_path = '../../src/face_img'
        self.threshold = 0.4
        self.enter_cnt = 10
        self.unknown_faces = {}
        self.max_enter = 10
        self.person = ""
        def load_faces_img(face_file_path):
            if not os.path.isdir(face_file_path):
                os.mkdir(face_file_path)
            face_img_files = os.listdir(face_file_path)
            return face_img_files

        def process_and_encode_faces_from_images(face_file_path):
            face_img_files = load_faces_img(face_file_path)
            for face_num in face_img_files:
                face_path = os.path.join(face_file_path, face_num)
                user_name = face_num.split(".")[0]
                face = cv2.imread(face_path, cv2.COLOR_BGR2RGB)
                encoding_face = fr.face_encodings(face)[0]
                face_encode_dict[int(user_name)] = encoding_face
            return face_img_files

        process_and_encode_faces_from_images(self.face_file_path)

    def known_name_face_location(self, frame, face_location, face_num):
        cv2.rectangle(frame, (face_location[2], face_location[0]), (face_location[3], face_location[1]), (0, 255, 0), 2)
        cv2.putText(frame, str(face_num), (face_location[2], face_location[0]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        print(face_num)
        return frame, face_num
    def unknown_name_face_location(self, frame, face_location):
        cv2.rectangle(frame, (face_location[2], face_location[0]), (face_location[3], face_location[1]), (0, 0, 255), 2)
        cv2.putText(frame, "come closer", (face_location[2], face_location[0]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return frame
    def callback(self):
        _, frame = self.cap.read()
        # frame = bridge.imgmsg_to_cv2(data, 'bgr8')
        blob = cv2.dnn.blobFromImage(frame, 1, (300, 300), (104,177, 123))
        self.net.setInput(blob)
        out = self.net.forward()
        detect = out[0,0,:,:]
        h, w = frame.shape[:2]
        face_detected = False
        for i in range(detect.shape[0]):
            confidence = detect[i, 2]
            if confidence >= self.dnn_confidence:
                x1 = int(detect[i, 3] * w)
                y1 = int(detect[i, 4] * h)
                x2 = int(detect[i, 5] * w)
                y2 = int(detect[i, 6] * h)
                h2 = int(y2 / 10)
                w2 = int(x2 / 10)
                face_location = [np.clip(y1 - h2, 0, 480), np.clip(y2 + h2, 0, 480),
                    np.clip(x1 - w2, 0, 640), np.clip(x2 + w2, 0, 640)]
                frame_face = frame[
                    face_location[0]:face_location[1],
                    face_location[2]:face_location[3],
                ]
                frame_face = cv2.cvtColor(frame_face, cv2.COLOR_BGR2RGB)
                try:
                    cam_encode = fr.face_encodings(frame_face)[0]
                    is_known_face = False
                    for person, encoding_result in face_encode_dict.items():
                        compare_face = fr.compare_faces([cam_encode], encoding_result, self.threshold)
                        if compare_face == [True]:
                            self.person = person
                            self.known_name_face_location(frame, face_location, person)
                            is_known_face = True
                            break

                    if not is_known_face:
                        self.unknown_name_face_location(frame, face_location)
                        if face_location[3] - face_location[2] > 170 and face_location[1] - face_location[0] > 230:
                            if self.enter_cnt in face_encode_dict:
                                self.enter_cnt +=1
                            if self.enter_cnt not in face_encode_dict:
                                face_img_path = f"{self.face_file_path}/{self.enter_cnt}.png"
                                cv2.imwrite(face_img_path, cv2.cvtColor(frame_face, cv2.COLOR_RGB2BGR))
                                unknown_encoding = fr.face_encodings(frame_face)[0]
                                self.unknown_faces[self.enter_cnt] = unknown_encoding
                                print(face_encode_dict.keys)
                    face_detected = True
                except:
                    pass
        if not face_detected:
            self.person = ""
        
        face_encode_dict.update(self.unknown_faces)
        if self.enter_cnt > self.max_enter-1:
            self.enter_cnt = 1
        cv2.imshow('img', frame)
        key = cv2.waitKey(33)
        if key == 27:
            PublishSubscribeNode.destroy_node()
            cv2.destroyAllWindows()
            rp.shutdown()
            self.cap.release()
    def publish_message(self):
        msg = Int32()
        if self.person:
            msg.data = self.person
        else:
            msg.data = 0
        self.publisher.publish(msg)
def main(args=None):
    rp.init(args=args)
    node = PublishSubscribeNode()
    try:
        while rp.ok():
            node.publish_message()
            rp.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rp.shutdown()
if __name__ == '__main__':
    main()