import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPalette, QColor
import time
import threading
from my_new_package.sound import play_sound 
import random




# ros ver
from_class = uic.loadUiType("src/Gui_ShoeBot/my_new_package/Gui_ShoeBot_PyQt.ui")[0]
# #python ver
# from_class = uic.loadUiType("basic.ui")[0]

class Listen(Node):
    def __init__(self):
        super().__init__('listener_node')

        self.subscriptiont_faceidget_ = self.create_subscription(Int32, 'faceid_to_main_givetopic_facenumber', self.faceid_to_main_givetopic_facenumber, 10)
        self.subscriptiont_faceidget_
        self.subscription_minibotfinHtoS_ = self.create_subscription(String, 'minibot_to_main_move_finish_home_to_shoebox', self.minibotfinHtoS_callback, 10)
        self.subscription_minibotfinHtoS_
        self.subscription_minibotfinAtoH_ = self.create_subscription(String, 'minibot_to_main_move_finish_anyposition_to_home', self.minibotfinAtoH_callback, 10)
        self.subscription_minibotfinAtoH_
        self.subscription_minibotfinStoH_ = self.create_subscription(String, 'minibot_to_main_move_finish_shoebox_to_home', self.minibotfinStoH_callback, 10)
        self.subscription_minibotfinStoH_
        self.subscription_roboarmfinpick_ = self.create_subscription(String, 'roboarm_to_main_pickupactive_finish', self.roboarmfinpick_callback, 10)
        self.subscription_roboarmfinpick_
        self.subscription_roboarmfinunleash_ = self.create_subscription(String, 'roboarm_to_main_unleashactive_finish', self.roboarmfinunleash_callback, 10)
        self.subscription_roboarmfinunleash_

#===publisher
        self.publisherminibotshoetohome_ = self.create_publisher(String, 'main_to_minibot_move_start_shoebox_to_home', 10)
        self.publisherminibotanytohome_ = self.create_publisher(String, 'main_to_minibot_move_start_anyposition_to_home', 10)
        self.publisherminibothometoshoe_ = self.create_publisher(String, 'main_to_minibot_move_start_home_to_shoebox', 10)
        self.publisherroboarmpick_ = self.create_publisher(String, 'main_to_roboarm_pipupactive_start', 10)
        self.publisherroboarmunleash_ = self.create_publisher(String, 'main_to_roboarm_unleashactive_start', 10)

        # self.publisherroboarm_= self.create_publisher(Bool, 'roboarm', 10)
# 타임순서로 만들기 사람이한명 왔다고 가정한후에 숫자 1을 줄거임 후에 (0~5까지줘야함)
    def faceid_to_main_givetopic_facenumber(self, msg):
        global testfaceid
        testfaceid = msg.data
# 미니봇이 도착한후에 퍼블리셔를 주고 받는 함수 이거 를 받으면 finish로 받게된다면 msg가 무조건 에스를 주고 그다음
# 이것 을 받은 친구가 다음동작을 확인후에 동작을 하면됩니다.
    def minibotfinHtoS_callback(self, msg):
        global minibotfinHtoS
        minibotfinHtoS = msg.data

    def minibotfinAtoH_callback(self, msg):
        global minibotfinAtoH
        minibotfinAtoH = msg.data

    def minibotfinStoH_callback(self, msg):
        global minibotfinStoH
        minibotfinStoH = msg.data
        # if msg.data == 'finish':
        #     self.get_logger().info('Received minibotfin pub worked')
        # else:
        #     self.get_logger().info('no work minibot checking now')

    # 로보암이 이용후에 퍼블리셔를 주고 받는 함수 이거 를 받으면 finish로 받게된다면 msg가 무조건 에스를 주고 그다음
# 이것 을 받은 친구가 다음동작을 확인후에 동작을 하면됩니다.
    def roboarmfinpick_callback(self, msg):
        global roboarmfinpick
        roboarmfinpick = msg.data

    def roboarmfinunleash_callback(self, msg):
        global roboarmfinunleash
        roboarmfinunleash = msg.data
        # if msg.data == 'finish':
        #     self.get_logger().info('Received roboarmfin pub worked')
        # else:
        #     self.get_logger().info('no work roboarm hecking now')

#publisher 관련함수임 auto function(왜냐면 testfaceid가 값을줌)
#애는 qaplication에 함수안에 함수를 넣어서 자동 실행하게금만듬
# # 타임순서 2번은 미니봇이 testfaceid 1번 을 가져가고 1번을 받는 동작이지 움직일수도있고아닐수도있음
#     def minibot_pub(self, testfaceid):
#         msg = Int32()
#         msg.data = testfaceid
#         # num = get_location(movingloc)
#         self.publisherminibotgo_.publish(msg)
#         self.get_logger().info(f'Publishing: "{testfaceid}"')

# 이 두개의 함수는 특정 행동을 해주려는 것이고 => 퍼블리셔로 발행 노드로 해서 람다 값으로 줘서
#클릭으로 이루어주면된다. 


    def main_to_minibot_move_start_shoebox_to_home(self):
        msg = String()
        msg.data = 'main_to_minibot_move_start_shoebox_to_home'
        self.publisherminibotshoetohome_.publish(msg)
        self.get_logger().info(f'Main_to_move_start_home_to_shoe: {msg.data}"')

    def main_to_minibot_move_start_anyposition_to_home(self):
        msg = String()
        msg.data = str(testfaceid)
        self.publisherminibotanytohome_.publish(msg)
        self.get_logger().info(f'anytposition_to_home: {msg.data}"')

    def main_to_minibot_move_start_home_to_shoebox(self):
        msg = String()
        msg.data = str(testfaceid)
        self.publisherminibothometoshoe_.publish(msg)
        self.get_logger().info(f'Publising: {msg.data}"')

    def main_to_roboarm_pipupactive_start(self):
        msg = String()
        msg.data = 'main_to_roboarm_pipupactive_start'
        self.publisherroboarmpick_.publish(msg)
        self.get_logger().info(f'Publising: {msg.data}"')

    def main_to_roboarm_unleashactive_start(self):
        msg = String()
        msg.data = 'main_to_roboarm_unleashactive_start'
        self.publisherroboarmunleash_.publish(msg)
        self.get_logger().info(f'Publising: {msg.data}"')


# #주는 것은 불 값으로 줌 트루 라면 트루로 받고 있고 실행할것이다.
#     def roboarm(self):
#         self.msg_ = Bool()
#         self.msg_.data = bool(random.getrandbits(1))
#         self.publisherroboarm_.publish(self.msg_)

#다른상황에 대해서{face id 에 두번
# list 정리해서 하는 것에대해서 
# } 두번 만약에 페이스 아이디 두번 이면 어떻게할까에서 기현
    # def testfaceidsec_callback(self, msg):
    #     global testfaceidsec
    #     testfaceidsec = msg.data
    # 
 
    # 실제 FaceID 이용할떄 이 콜백을 사용하면 됨
    # def FaceID_callback(self, msg):
    #     global FaceID
    #     FaceID = msg.data

class WindowClass(QMainWindow, from_class) :
    def __init__(self, node):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Shoebot_Ver.0.95!")
        
        
# #auto trigger===
#         1000ms (1초) 마다 타이머 이벤트 발생
#  #        어투 콜백을 하지말고 손으로 일일히하자
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.auto_refresh)
        # self.timer.start(1000) 
#===================

        self.getfaceidbtn.clicked.connect(self.faceid_to_main_givetopic_facenumber)
    
        self.miniStoHfinbtn.clicked.connect(lambda: self.minibotfinStoHer(node))
        self.miniAtoHfinbtn.clicked.connect(lambda: self.minibotfinAtoHer(node))
        self.miniHtoSfinbtn.clicked.connect(lambda: self.minibotfinHtoSer(node))
        self.roboUfinbtn.clicked.connect(lambda: self.roboarmfinpicker(node))
        self.roboPfinbtn.clicked.connect(lambda: self.roboarmfinunleasher(node))
        self.miniHtoSbtn.clicked.connect(lambda: self.minibotHtoS(node))
        self.miniStoHbtn.clicked.connect(lambda: self.minibotStoH(node))
        self.miniAtoHbtn.clicked.connect(lambda: self.minibotAtoH(node))
        self.roboPbtn.clicked.connect(lambda: self.robopickupact(node))
        self.roboUbtn.clicked.connect(lambda: self.robounleachact(node))
        self.resetbtn.clicked.connect(self.reset)
        
       
        self.rosMobilestart.clicked.connect(lambda: self.mainmobile(node, testfaceid))
        self.rosMobileRstart.clicked.connect(lambda: self.mainmobileRev(node, testfaceid))
# # each button have trigger publihser
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.finish_color_count)
        self.timer2 = QTimer(self)
        self.timer2.timeout.connect(self.finish_color_count2)
    
        global testfaceidlist
        testfaceidlist = []

        global active_color
        active_color = QColor(0,0, 255) # pure blue

        global finish_color
        finish_color = QColor(169,169,169)

        global Inactive_color
        Inactive_color = QColor(255,255,255)



    def finish_color_count(self):
        
        QTimer.singleShot(500, lambda: self.guiline.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(1000, lambda: self.guiline2.setStyleSheet
                            (f"background-color: {active_color.name()}"))
        self.timer.stop()
    def finish_color_count2(self):
        QTimer.singleShot(1500, lambda: self.guiline2.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(2000, lambda: self.guiline3.setStyleSheet
                            (f"background-color: {active_color.name()}"))     
    def finish_color_count3(self):
        QTimer.singleShot(3000, lambda: self.guiline3.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(3500, lambda: self.guiline4.setStyleSheet
                            (f"background-color: {active_color.name()}"))
    def finish_color_count4(self):
        QTimer.singleShot(4500, lambda: self.guiline4.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(5000, lambda: self.guiline5.setStyleSheet
                            (f"background-color: {active_color.name()}"))
    def finish_color_count5(self):
        QTimer.singleShot(6000, lambda: self.guiline5.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(6500, lambda: self.guiline6.setStyleSheet
                            (f"background-color: {active_color.name()}"))
    def finish_color_homecount(self):
        QTimer.singleShot(7500, lambda: self.guiline6.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(8000, lambda: self.guiline7.setStyleSheet
                            (f"background-color: {active_color.name()}"))
    def finish_color_revcount(self):
        QTimer.singleShot(0, lambda: self.guiline7.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(500, lambda: self.guiline8.setStyleSheet
                            (f"background-color: {active_color.name()}"))
    def finish_color_revcount2(self):
        QTimer.singleShot(1000, lambda: self.guiline8.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(1500, lambda: self.guiline9.setStyleSheet
                            (f"background-color: {active_color.name()}"))
    def finish_color_revcount3(self):
        QTimer.singleShot(2500, lambda: self.guiline9.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(3000, lambda: self.guiline10.setStyleSheet
                            (f"background-color: {active_color.name()}"))
    def finish_color_revcount4(self):
        QTimer.singleShot(4000, lambda: self.guiline10.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(4500, lambda: self.guiline11.setStyleSheet
                            (f"background-color: {active_color.name()}"))
    def finish_color_revcount5(self):
        QTimer.singleShot(5500, lambda: self.guiline11.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(6000, lambda: self.guiline12.setStyleSheet
                            (f"background-color: {active_color.name()}"))
    def finish_color_revhomecount(self):
        QTimer.singleShot(7000, lambda: self.guiline12.setStyleSheet
                            (f"background-color: {finish_color.name()}"))
        QTimer.singleShot(7500, lambda: self.guiline7.setStyleSheet
                            (f"background-color: {active_color.name()}"))
   #===================================== 
    def reset(self):
        self.lineEdit.setText('')
        self.lineEdit_2.setText('')
        self.lineEdit_3.setText('')
        self.lineEdit_4.setText('')
        self.lineEdit_5.setText('')
        self.lineEdit_6.setText('')
        self.roslog.setText('')
        self.faceidlog.setText('')
        self.roboarmlog.setText('')
        self.minibotlog.setText('')
        self.guiline.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline2.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline3.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline4.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline5.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline6.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline7.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline8.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline9.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline10.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline11.setStyleSheet(f"background-color: {Inactive_color.name()};")
        self.guiline12.setStyleSheet(f"background-color: {Inactive_color.name()};")
       

    def faceid_to_main_givetopic_facenumber(self):
        self.lineEdit.setText(str(testfaceid))
        # faceId INt 32 로 받아온것을 str로 반환
        # self.roslog.append('testfaceid worked')
        # faceID 받아오는거 확인 
    def minibotHtoS(self,node):
        node.main_to_minibot_move_start_home_to_shoebox()
        self.lineEdit_2.setText('main_to_minibot_move_start_home_to_shoebox')
    def minibotStoH(self,node):
        node.main_to_minibot_move_start_shoebox_to_home()
        self.lineEdit_2.setText('main_to_minibot_move_start_shoebox_to_home')
    def minibotAtoH(self,node):
        node.main_to_minibot_move_start_anyposition_to_home()
        self.lineEdit_2.setText('main_to_minibot_move_start_anyposition_to_home')
    def robopickupact(self,node):
        node.main_to_roboarm_pipupactive_start()
        self.lineEdit_2.setText('main_to_roboarm_pipupactive_start')
    def robounleachact(self,node):
        node.main_to_roboarm_unleashactive_start()
        self.lineEdit_2.setText('main_to_roboarm_unleashactive_start')
    def minibotfinStoHer(self, node):
        if minibotfinStoH == 'minibot_to_main_move_finish_shoebox_to_home':
            self.lineEdit_4.setText('minibot_done')
        else:
           self.lineEdit_4.setText('minibot_NOTDone')
    def minibotfinHtoSer(self, node):
        if minibotfinHtoS == 'minibot_to_main_move_finish_home_to_shoebox':
            self.lineEdit_4.setText('minibot_done')
        else:
           self.lineEdit_4.setText('minibot_NOTDone')
    def minibotfinAtoHer(self, node):
        if minibotfinAtoH == 'minibot_to_main_move_finish_anyposition_to_home':
            self.lineEdit_4.setText('minibot_done')
        else:
           self.lineEdit_4.setText('minibot_NOTDone')
    def roboarmfinpicker(self, node):
        if roboarmfinpick == 'roboarm_to_main_pickupactive_finish':
            self.lineEdit_5.setText('roboarm_to_main_pickupactive_finish_done')
        else:
            self.lineEdit_5.setText('roboarm_NOTDone')
    def roboarmfinunleasher(self, node):
        if roboarmfinunleash == 'roboarm_to_main_unleashactive_finish':
            self.lineEdit_5.setText('roboarm_to_main_unleashactive_finish')
        else:
            self.lineEdit_5.setText('roboarm_NOTDone')
                
    # def auto_refresh(self):
    #     self.rosMobilestart.click()
    #     self.getfaceidbtn.click()
    #     self.rosMobileRstart.click()
        # self.roboPbtn.click()
        # self.miniHtoSbtn.click()
        # self.miniStoHbtn.click()
        # self.miniAtoHbtn.click()
        # self.roboUbtn.click()
   
    def mainmobile(self, node, testfaceid):
        testfaceid_tmp = str(testfaceid)
        if testfaceid == 0:
            self.lineEdit_4.setText('Face_ID not recognized')
            self.lineEdit_5.setText('')
            self.lineEdit_6.setText('')
        elif testfaceid == 6: # 미니봇이 특별한 위치가아닌 랜덤위치에 있을떄 """ anyposition to home을 줍니다 """
            self.lineEdit_4.setText('홈위치에갈것')
            self.lineEdit_5.setText('')
            self.lineEdit_6.setText('')
            self.lineEdit_4.setText('main_to_minibot_move_start_anyposition_to_home')
            
            self.roslog.append('main_to_minibot_move_start_anyposition_to_home')
            self.minibotlog.append('main_to_minibot_move_start_anyposition_to_home')
            self.miniAtoHbtn.click
            node.minibotfinAtoH_callback
            if minibotfinAtoH == 'Minibot arrived Any to Home Successfully!!!':
                self.lineEdit_4.setText('minibot_to_main_move_finish_anyposition_to_home')
                self.roslog.append('minibot_to_main_move_finish_anyposition_to_home')   
                self.minibotlog.append('minibot_to_main_move_finish_anyposition_to_home')     
# 신발장 1~5번의 신호를 줄것입니다.   
        elif 1 <= testfaceid <= 5:
            if testfaceid not in testfaceidlist:
                testfaceidlist.append(testfaceid)
                self.faceidlog.append('first to see :' + testfaceid_tmp)
                self.lineEdit_3.setText('faceid_to_main_givetopic_facenumber')
                self.guiline.setStyleSheet(f"background-color: {active_color.name()};")
                self.roslog.append(f'faceid_to_main_givetopic_facenumber : {testfaceid_tmp}')
                
                self.finish_color_count()
                self.lineEdit_4.setText('main_to_roboarm_pipupactive_start')
                self.roslog.append('main_to_roboarm_pipupactive_start')
                self.roboarmlog.append('main_to_roboarm_pipupactive_start')
                
                self.roboPbtn.click()
                # 일정 행동후에 pickupact _finishbtn 클릭하라고 주면됨
                self.roboPfinbtn.clicked.connect(lambda: self.roboarmfinpicker(node))
                
                self.finish_color_count2()
                
                
                node.roboarmfinpick_callback
                if roboarmfinpick == 'roboarm_to_main_pickupactive_finish':
                    

                    self.lineEdit_5.setText('roboarm_to_main_pickupactive_finish')
                    self.roslog.append('roboarm_to_main_pickupactive_finish')
                    self.roboarmlog.append('roboarm_to_main_pickupactive_finish')
                    self.finish_color_count3()

                    self.lineEdit_4.setText(f'main_to_minibot_move_start_home_to_{testfaceid_tmp} shoebox')
                    self.roslog.append(f'main_to_minibot_move_start_home_to_{testfaceid_tmp} shoebox')
                    self.minibotlog.append(f'main_to_minibot_move_start_home_to_{testfaceid_tmp} shoebox')
                    

                    self.miniHtoSbtn.click()
                    self.miniHtoSfinbtn.clicked.connect(lambda: self.minibotfinHtoSer(node))
                    node.minibotfinHtoS_callback

                    if minibotfinHtoS == 'Minibot arrived Home to Shoebox Successfully!!!':
                        self.lineEdit_4.setText(f'minibot_to_main_move_finish_home_to_{testfaceid_tmp}shoebox')
                        self.roslog.append(f'minibot_to_main_move_finish_home_to_{testfaceid_tmp} shoebox')
                        self.minibotlog.append(f'minibot_to_main_move_finish_home_to_{testfaceid_tmp} shoebox')
                        

                        self.lineEdit_5.setText('main_to_roboarm_unleashactive_start')
                        self.roslog.append('main_to_roboarm_unleashactive_start')
                        self.roboarmlog.append('main_to_roboarm_unleashactive_start')
                        self.finish_color_count4()

                        self.roboUbtn.click()       
                        self.roboUfinbtn.clicked.connect(lambda: self.roboarmfinunleasher(node))
                        node.roboarmfinunleash_callback
                        if roboarmfinunleash == 'roboarm_to_main_unleashactive_finish':
                            self.lineEdit_5.setText('roboarm_to_main_unleashactive_finish')
                            self.roslog.append('roboarm_to_main_unleashactive_finish')
                            self.roboarmlog.append('roboarm_to_main_unleashactive_finish')
                            

                            self.lineEdit_4.setText(f'main_to_minibot_move_start_{testfaceid_tmp}shoebox_to_home')
                            self.roslog.append(f'main_to_minibot_move_start_{testfaceid_tmp}shoebox_to_home')
                            self.minibotlog.append(f'main_to_minibot_move_start_{testfaceid_tmp}shoebox_to_home')
                            self.finish_color_count5()
                            
                            self.miniStoHbtn.click
                            self.miniStoHfinbtn.clicked.connect(lambda: self.minibotfinStoHer(node))
                            node.minibotfinStoH_callback
                            if minibotfinStoH == 'Minibot arrived Shoebox to Home Successfully!!!':
                                self.lineEdit_4.setText(f'minibot_to_main_move_finish_{testfaceid_tmp}shoebox_to_home')
                                self.roslog.append(f'minibot_to_main_move_finish_{testfaceid_tmp}shoebox_to_home')
                                self.minibotlog.append(f'minibot_to_main_move_finish_{testfaceid_tmp}shoebox_to_home')
                                self.finish_color_homecount()

                                self.lineEdit_5.setText(f'shoe -{testfaceid_tmp}box in -completed')
                                self.roslog.append('minibot state 0 location-completed')
                                self.lineEdit_6.setText(f'shoe -{testfaceid_tmp}-completed')
                                self.roslog.append(f'shoe -{testfaceid_tmp}-completed')
                            else:
                                return None
            else:
                self.faceidlog.append('Error alread exist num : ' + testfaceid_tmp)
#신발장 1~5 있는상태에서 가져가는방법입니다.
    def mainmobileRev(self, node, testfaceid):
                testfaceid_tmp = str(testfaceid)
                if testfaceid == 0:
                    self.lineEdit_4.setText('Face_ID not recognized')
                    self.lineEdit_5.setText('')
                    self.lineEdit_6.setText('')
                elif testfaceid == 6: # 미니봇이 특별한 위치가아닌 랜덤위치에 있을떄 """ anyposition to home을 줍니다 """
                    self.lineEdit_4.setText('홈위치에갈것')
                    self.lineEdit_5.setText('')
                    self.lineEdit_6.setText('')
                    self.lineEdit_4.setText('main_to_minibot_move_start_anyposition_to_home')
                    self.roslog.append('main_to_minibot_move_start_anyposition_to_home')
                    self.minibotlog.append('main_to_minibot_move_start_anyposition_to_home')
                    self.miniAtoHbtn.click
                    node.minibotfinAtoH_callback
                    if minibotfinAtoH == 'Minibot arrived Any to Home Successfully!!!':
                        self.lineEdit_4.setText('minibot_to_main_move_finish_anyposition_to_home')
                        self.roslog.append('minibot_to_main_move_finish_anyposition_to_home')
                        self.minibotlog.append('minibot_to_main_move_finish_anyposition_to_home')        
                elif 1 <= testfaceid <= 5:
                    if testfaceid in testfaceidlist:
                        testfaceidlist.remove(testfaceid)
                        self.finish_color_revcount()
                        self.faceidlog.append('second to see : ' + testfaceid_tmp)
                        self.lineEdit_3.setText('faceid_to_main_givetopic_facenumber')
                        self.roslog.append(f'faceid_to_main_givetopic_facenumber : {testfaceid_tmp}')

                    
                        self.lineEdit_4.setText(f'main_to_minibot_move_start_home_to_{testfaceid_tmp}shoebox')
                        self.roslog.append(f'main_to_minibot_move_start_home_to_{testfaceid_tmp}shoebox')
                        self.minibotlog.append(f'main_to_minibot_move_start_home_to_{testfaceid_tmp}shoebox')
                        self.finish_color_revcount2()

                        self.miniHtoSbtn.click()
                        self.miniStoHfinbtn.clicked.connect(self.minibotfinHtoSer)

                        node.minibotfinHtoS_callback

                        if minibotfinHtoS == 'Minibot arrived Home to Shoebox Successfully!!!':
                            self.finish_color_revcount3()
                            self.lineEdit_4.setText(f'minibot_to_main_move_finish_home_to_{testfaceid_tmp}shoebox')
                            self.roslog.append(f'minibot_to_main_move_finish_home_to_{testfaceid_tmp}shoebox')
                            self.minibotlog.append(f'minibot_to_main_move_finish_home_to_{testfaceid_tmp}shoebox')
                            
                            self.lineEdit_4.setText('main_to_roboarm_pipupactive_start')
                            self.roslog.append('main_to_roboarm_pipupactive_start')
                            self.roboarmlog.append('main_to_roboarm_pipupactive_start')
                            self.roboPbtn.click()
                            
                            self.roboPfinbtn.clicked.connect(self.roboarmfinpicker)
                            node.roboarmfinpick_callback
                        if roboarmfinpick == 'roboarm_to_main_pickupactive_finish':
                            self.finish_color_revcount4()
                            self.lineEdit_5.setText('roboarm_to_main_pickupactive_finish')
                            self.roslog.append('roboarm_to_main_pickupactive_finish')
                            self.roboarmlog.append('roboarm_to_main_pickupactive_finish')
                            
                            self.lineEdit_4.setText(f'main_to_minibot_move_start_{testfaceid_tmp}shoebox_to_home')
                            self.roslog.append(f'main_to_minibot_move_start_{testfaceid_tmp}shoebox_to_home')
                            self.minibotlog.append(f'main_to_minibot_move_start_{testfaceid_tmp}shoebox_to_home')

                            self.miniStoHbtn.click()
                            self.miniStoHfinbtn.clicked.connect(self.minibotfinStoHer)
                            node.minibotfinStoH_callback
                            if minibotfinStoH == 'Minibot arrived Shoebox to Home Successfully!!!':
                                
                                self.lineEdit_4.setText(f'minibot_to_main_move_finish_{testfaceid_tmp}shoebox_to_home')
                                self.roslog.append(f'minibot_to_main_move_finish_{testfaceid_tmp}shoebox_to_home')
                                self.minibotlog.append(f'minibot_to_main_move_finish_{testfaceid_tmp}shoebox_to_home')

                                self.lineEdit_5.setText('main_to_roboarm_unleashactive_start')
                                self.roslog.append('main_to_roboarm_unleashactive_start')
                                self.roboarmlog.append('main_to_roboarm_unleashactive_start')


                                self.roboUbtn.click()    
                                self.roboUfinbtn.clicked.connect(self.roboarmfinunleasher)
                                node.roboarmfinunleash_callback

                                if roboarmfinunleash == 'roboarm_to_main_unleashactive_finish':
                                    self.finish_color_revcount5()
                                    self.lineEdit_5.setText('roboarm_to_main_unleashactive_finish')
                                    self.roslog.append('roboarm_to_main_unleashactive_finish')
                                    self.roboarmlog.append('roboarm_to_main_unleashactive_finish')

                                    self.lineEdit_5.setText(f'shoe -{testfaceid_tmp}box in -completed')
                                    self.roslog.append('minibot state 0 location-completed')
                                    self.lineEdit_6.setText(f'shoe -{testfaceid_tmp}-completed')
                                    self.roslog.append(f'shoe -{testfaceid_tmp}-completed')
                                    self.finish_color_revhomecount()
                                else:
                                            
                                    return  self.roslog.append('-1- section not works')  
                    else:
                        self.faceidlog.append(f'not works check the {testfaceid_tmp} list') 
    
#version 0.98
def main():
    rclpy.init()
    node = Listen()
    app = QApplication(sys.argv)
    myWindows = WindowClass(node)
    thread1 = threading.Thread(target=rclpy.spin, args=[node], daemon=False)
    thread1.start()
    myWindows.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()