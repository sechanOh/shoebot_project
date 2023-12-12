import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time

class MobileRobot(Node):

    def __init__(self):
        super().__init__('minibotgo_minibotgohome_minibotgobtn_pub__minibot_finished')

        #위에거가 메인에서 싸줘서 애가 받았어 그다음에 애는
        #인정을해야함 . 받았고, 그다음에 해야하는것
        self.subscriptionminiStoH_ = self.create_subscription(String, 'main_to_minibot_move_start_shoebox_to_home', self.miniStoH_callback, 10)
        self.subscriptionminiStoH_
        self.subscriptionminiAtoS_ = self.create_subscription(String, 'main_to_minibot_move_start_anyposition_to_home', self.miniAtoH_callback, 10)
        self.subscriptionminiAtoS_
        self.subscriptionminiHtoS_ = self.create_subscription(String, 'main_to_minibot_move_start_home_to_shoebox', self.miniHtoS_callback, 10)
        self.subscriptionminiHtoS_

    #publisher 3초에 하나씩 준다. 만약 끝났을떄 주고싶으면



    #작업이 끝난후에 함수 지정에서 minibotfinishetimer 을 주면된다.
    # 작업이 끝난후에 msg.data에 finish를를 메인에게 주면 메인은
    # finish를 알아듣는다. 그리고 다음행동가능 .   주면 
    # finish를 발행한다. 그러면 그 finish 
        self.publisher_minibotfinHtoS_ = self.create_publisher(String, 'minibot_to_main_move_finish_home_to_shoebox', 10)
        #5초 후에 퍼블리셔 발행한다.
        self.timer_ = self.create_timer(5, self.minibotfinHtoStimer)
        self.publisher_minibotfinAtoH_ = self.create_publisher(String, 'minibot_to_main_move_finish_anyposition_to_home', 10)
        #5초 후에 퍼블리셔 발행한다.
        self.timer_ = self.create_timer(5, self.minibotfinAtoHtimer)
        self.publisher_minibotfinStoH_ = self.create_publisher(String, 'minibot_to_main_move_finish_shoebox_to_home', 10)
        #5초 후에 퍼블리셔 발행한다.
        self.timer_ = self.create_timer(5, self.minibotfinStoHtimer)







    def minibotfinHtoStimer(self):
        msg = String()
        msg.data = 'Minibot arrived Home to Shoebox Successfully!!!'
        self.publisher_minibotfinHtoS_.publish(msg)
        self.get_logger().info(f'HometoSFin: "{msg.data}"')
    def minibotfinAtoHtimer(self):
        msg = String()
        msg.data = 'Minibot arrived Any to Home Successfully!!!'
        self.publisher_minibotfinAtoH_.publish(msg)
        self.get_logger().info(f'AnytoHomeFin: "{msg.data}"')
    def minibotfinStoHtimer(self):
        msg = String()
        msg.data = 'Minibot arrived Shoebox to Home Successfully!!!'
        self.publisher_minibotfinStoH_.publish(msg)
        self.get_logger().info(f'ShoetoHomeFin: "{msg.data}"')

        



    def miniStoH_callback(self, msg):
        # 로케이션에대한 정의를 지정해준후에 msg_는 숫자로 받는거임.
         # location = msg.location
        self.get_logger().info(f'I heard: "{msg.data}"')

    def miniAtoH_callback(self, msg):
        # 로케이션에대한 정의를 지정해준후에 msg_는 숫자로 받는거임.
         # location = msg.location
        self.get_logger().info(f'I heard: "{msg.data}"')


    def miniHtoS_callback(self, msg):
        # 로케이션에대한 정의를 지정해준후에 msg_는 숫자로 받는거임.
         # location = msg.location
        self.get_logger().info(f'I heard: "{msg.data}"')

    #listener_callback 에서는 일단 지금 다받지만, 각각 받을시에는
    #홈으로 받았을때 홈의 위치로 가는 함수 만들고,
    #그러면 메인에서 클릭을 누르면 퍼블리쉬를 하고, 그다음 여기서 서브스크립션을한다.
    #하고 나면 여기서 여기서 함수를 받은후에 터미널에 로그 써주고, 
    # 고홈이면 홈의 위치로 가라고 보내주면 된다.
    # 
    # 1) minibot_pub 함수에관하여
    # def minibot_pub_callback(self, msg):
    #     msg = Int32()
        
    #     if msg.data == 1:
    #         move(1)
    #     elif msg.data ==2:
    #         move(2)
    # 움직임이 끝난 후에나서 애보고 주면됨. 
    

    # 2) minibotgohome 함수에 관하여
    # def minibotgohome(self, msg):
    #     msg = String()

    #     if msg.data == 'minibotgohome':

    #         {working something go home}
    #         pass
    #     else: 
    #         return None
    # 3) minibotgobtn(self, msg)
    # def minibotgobtn(self, msg, tsetfaceid):
    #     msg =String()
    #     if msg.data == 'minibotgoanybtn':
    #         {working go {1.2.3.4,5 {test faceid}}}
    #         pass
    #     else:
    #         return None



def main(args=None):
    rclpy.init(args=args)
    node = MobileRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()