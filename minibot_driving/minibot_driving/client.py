
import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberPublisher(Node):

    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Int32, 'FaceID', 10)
        self.timer = self.create_timer(1, self.publish_number)
        self.get_logger().info("Type a number to publish...")

    def publish_number(self):
        number = int(input("Please select location number between 1 & 4 : "))  # 입력된 숫자를 받음
        msg = Int32()
        msg.data = number
        self.publisher_.publish(msg)
        self.get_logger().info(f"You selected : {number} location")

def main(args=None):
    rclpy.init(args=args)
    publisher = NumberPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

