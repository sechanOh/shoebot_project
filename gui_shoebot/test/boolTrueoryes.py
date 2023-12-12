import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import random

class Testnode(Node):

    def __init__(self):
        super().__init__('faceID2_node')
        self.publisher_ = self.create_publisher(Bool, 'FaceID2', 10)
        self.timer_ = self.create_timer(4, self.timer_callback)
        

    def timer_callback(self):
        self.msg_ = Bool()
        self.msg_.data = bool(random.getrandbits(1))
        self.publisher_.publish(self.msg_)
        self.get_logger().info(f'Publishing: "{self.msg_.data}"')
        


def main(args=None):
    rclpy.init(args=args)
    node = Testnode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
