from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from rclpy.node import Node
import threading
from std_msgs.msg import Int32
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult


class FindLocation(Node):
    def __init__(self):
        super().__init__('find_locate')
        
        # topic을 subscribe 하기 위해 subscrption을 생성
        self.subscription = self.create_subscription(
            Int32, # 수신할 메시지 타입 지정
            'FaceID', # 구독할 토픽의 이름 지정
            self.Location_Select, # 메시지가 도착했을때 실행할 callback 함수 지정
            10 ) # 메세지를 저장하는 버퍼의 크기 설정
        self.subscription  # prevent unused variable warning
        
        
        self.get_logger().info("Waiting command")
        
        # gazebo 연결 설정
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        
        # 좌표 설정
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        
        # 결과값 설정
        self.result = self.nav.getResult()
        
        
    def Location_Select(self,msg):
        self.get_logger().info("You select %d Location" %(msg.data))
        print("msg.data = ",msg.data)
        
        if msg.data == 1:
            # 1st location
            self.goal_pose.pose.position.x = -0.8791484832763672
            self.goal_pose.pose.position.y = -1.8262792825698853
            self.goal_pose.pose.position.z = -0.001434326171875
            self.goal_pose.pose.orientation.x = 0.0
            self.goal_pose.pose.orientation.y = 0.0
            self.goal_pose.pose.orientation.z = 0.0
            self.goal_pose.pose.orientation.w = 0.0
            
        elif msg.data == 2:
            self.goal_pose.pose.position.x = -0.710483193397522
            self.goal_pose.pose.position.y = 1.8095852136611938
            self.goal_pose.pose.position.z = -0.001434326171875
            self.goal_pose.pose.orientation.x = 0.0
            self.goal_pose.pose.orientation.y = 0.0
            self.goal_pose.pose.orientation.z = 0.0
            self.goal_pose.pose.orientation.w = 0.0
            
        elif msg.data == 3:
            self.goal_pose.pose.position.x = 1.2454776763916016
            self.goal_pose.pose.position.y = 1.5881965160369873
            self.goal_pose.pose.position.z = -0.001434326171875
            self.goal_pose.pose.orientation.x = 0.0
            self.goal_pose.pose.orientation.y = 0.0
            self.goal_pose.pose.orientation.z = 0.0
            self.goal_pose.pose.orientation.w = 0.0
            
        elif  msg.data == 4:
            self.goal_pose.pose.position.x = 1.3337560892105103
            self.goal_pose.pose.position.y = -1.5501230955123901
            self.goal_pose.pose.position.z = -0.001434326171875
            self.goal_pose.pose.orientation.x = 0.0
            self.goal_pose.pose.orientation.y = 0.0
            self.goal_pose.pose.orientation.z = 0.0
            self.goal_pose.pose.orientation.w = 0.0
            print(self.goal_pose.pose.position.x)

        else:
            print("Please enter between 1 and 4")
            
        self.nav.goToPose(self.goal_pose)
        while not self.nav.isTaskComplete():
            pass
        
        if self.nav.isTaskComplete():
            print("minibot is %d location arrived!" %(msg.data))
            time.sleep(3)
        
        # if self.result == TaskResult.SUCCEEDED:
        #     print('Goal succeeded')
        # elif self.result == TaskResult.CANCELED:
        #     print('Goal was canceled!')
        #     self.nav.goToPose(self.goal_pose)
        # elif self.result == TaskResult.FAILED:
        #     print('Goal failed')
        #     self.nav.goToPose(self.goal_pose)
            
        # if self.nav is not None:
        #     self.nav.goToPose(self.nav.goal_pose)
        # else:
        #     print("wait press number")
        
            
            

def main(args=None):
    # Gazebo Setup!
    rclpy.init(args=args)
    
    find_locate = FindLocation()
    rclpy.spin(find_locate)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    