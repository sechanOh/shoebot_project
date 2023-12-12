import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class MyPublisher(Node):

    def __init__(self):
        super().__init__('faceid_to_main_givetopic_facenum')
        self.publisher = self.create_publisher(Int32, 'faceid_to_main_givetopic_facenumber', 10)
        self.timer = self.create_timer(1, self.publish_data)
        self.counter = 0

    def publish_data(self):
        for value in range(3 ,5):  # 0부터 5까지의 정수를 반복
            msg = Int32()
            msg.data = value
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')
            time.sleep(3)

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()