from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from rclpy.node import Node


class FindLocation(Node):
    def __init__(self):
        super().__init__('minimal_service')
        
    def Location_Select(self, location):
        self.get_logger().info("You select %d Location" %(location.a))
        
        if location.a == 1:
            

def main(args=None):
    # Gazebo Setup!
    rclpy.init(args=args)
    find_locate = FindLocation()
    rclpy.spin(find_locate)
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    goal_poses = []

    for i in range(1,5):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        goal_poses.append(goal_pose)
        # print("goal_poses[0]")

    # 1st location
    goal_poses[0].pose.position.x = -0.8791484832763672
    goal_poses[0].pose.position.y = -1.8262792825698853
    goal_poses[0].pose.position.z = -0.001434326171875
    goal_poses[0].pose.orientation.x = 0.0
    goal_poses[0].pose.orientation.y = 0.0
    goal_poses[0].pose.orientation.z = 0.0
    goal_poses[0].pose.orientation.w = 0.0


    # 2nd location
    goal_poses[1].pose.position.x = -0.710483193397522
    goal_poses[1].pose.position.y = 1.8095852136611938
    goal_poses[1].pose.position.z = -0.001434326171875

    goal_poses[1].pose.orientation.x = 0.0
    goal_poses[1].pose.orientation.y = 0.0
    goal_poses[1].pose.orientation.z = 0.0
    goal_poses[1].pose.orientation.w = 0.0


    # 3rd location
    goal_poses[2].pose.position.x = 1.3337560892105103
    goal_poses[2].pose.position.y = -1.5501230955123901
    goal_poses[2].pose.position.z = -0.001434326171875

    goal_poses[2].pose.orientation.x = 0.0
    goal_poses[2].pose.orientation.y = 0.0
    goal_poses[2].pose.orientation.z = 0.0
    goal_poses[2].pose.orientation.w = 0.0


    # 4th location
    goal_poses[3].pose.position.x = 1.2454776763916016
    goal_poses[3].pose.position.y = 1.5881965160369873
    goal_poses[3].pose.position.z = -0.001434326171875

    goal_poses[3].pose.orientation.x = 0.0
    goal_poses[3].pose.orientation.y = 0.0
    goal_poses[3].pose.orientation.z = 0.0
    goal_poses[3].pose.orientation.w = 0.0

if __name__ == "__main__":
    main()
    