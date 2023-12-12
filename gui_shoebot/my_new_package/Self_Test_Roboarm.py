import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class Roboarm(Node):

    def __init__(self):
        super().__init__('listener_node')
        self.subscriptionroboarmpick_ = self.create_subscription(String, 'main_to_roboarm_pipupactive_start', self.roboarm_pickup_callback, 10)
        self.subscriptionroboarmpick_
        self.subscriptionunleash_ = self.create_subscription(String, 'main_to_roboarm_unleashactive_start', self.roboarm_unleash_callback, 10)
        self.subscriptionunleash_

        self.publisher_roboarmfinpick_ = self.create_publisher(String, 'roboarm_to_main_pickupactive_finish', 10)
        #5초 후에 퍼블리셔 발행한다.
        self.timer_ = self.create_timer(5, self.roboarm_to_main_pickupactive_finish_publisher)
        self.publisher_roboarmfinunleash_ = self.create_publisher(String, 'roboarm_to_main_unleashactive_finish', 10)
        #5초 후에 퍼블리셔 발행한다.
        self.timer_ = self.create_timer(5, self.roboarm_to_main_unleashactive_finish_publisher)






# after finished action @@@
    def roboarm_to_main_pickupactive_finish_publisher(self):
        msg = String()
        msg.data = 'roboarm_to_main_pickupactive_finish'
        self.publisher_roboarmfinpick_.publish(msg)
        self.get_logger().info(f'pickup: "{msg.data}"')
        

    
    def roboarm_to_main_unleashactive_finish_publisher(self):
        msg = String()
        msg.data = 'roboarm_to_main_unleashactive_finish'
        self.publisher_roboarmfinunleash_.publish(msg)
        self.get_logger().info(f'unleash: "{msg.data}"')

##@@@@after finished action



# trigger give them
    def roboarm_pickup_callback(self, msg):
         # something doing what
         self.get_logger().info(f'I heard pick up: "{msg.data}"')


    def roboarm_unleash_callback(self, msg):
         # something doing what
         self.get_logger().info(f'I heard unleash: "{msg.data}"')



def main(args=None):
    rclpy.init(args=args)
    node = Roboarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
