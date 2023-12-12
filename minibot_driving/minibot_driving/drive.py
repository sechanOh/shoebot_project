from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

# Gazebo Setup!
rclpy.init()
nav = BasicNavigator()
nav.waitUntilNav2Active()
goal_poses = []

for i in range(1,5):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_poses.append(goal_pose)
    # print("goal_poses[0]")


goal_poses[0].pose.position.x = -0.8791484832763672
goal_poses[0].pose.position.y = -1.8262792825698853
goal_poses[0].pose.position.z = -0.001434326171875
goal_poses[0].pose.orientation.x = 0.0
goal_poses[0].pose.orientation.y = 0.0
goal_poses[0].pose.orientation.z = 0.0
goal_poses[0].pose.orientation.w = 0.0


# 2nd position
goal_poses[1].pose.position.x = -0.710483193397522
goal_poses[1].pose.position.y = 1.8095852136611938
goal_poses[1].pose.position.z = -0.001434326171875

goal_poses[1].pose.orientation.x = 0.0
goal_poses[1].pose.orientation.y = 0.0
goal_poses[1].pose.orientation.z = 0.0
goal_poses[1].pose.orientation.w = 0.0


# 3rd position
goal_poses[2].pose.position.x = 1.3337560892105103
goal_poses[2].pose.position.y = -1.5501230955123901
goal_poses[2].pose.position.z = -0.001434326171875

goal_poses[2].pose.orientation.x = 0.0
goal_poses[2].pose.orientation.y = 0.0
goal_poses[2].pose.orientation.z = 0.0
goal_poses[2].pose.orientation.w = 0.0


# 4th position
goal_poses[3].pose.position.x = 1.2454776763916016
goal_poses[3].pose.position.y = 1.5881965160369873
goal_poses[3].pose.position.z = -0.001434326171875

goal_poses[3].pose.orientation.x = 0.0
goal_poses[3].pose.orientation.y = 0.0
goal_poses[3].pose.orientation.z = 0.0
goal_poses[3].pose.orientation.w = 0.0

# Driving course 1->2->3->4 course
for i,pose in enumerate(goal_poses):
    j = 0
    print("i = ",i)
    nav.goToPose(goal_poses[i])
    # Calculating remained Distance
    while not nav.isTaskComplete():
        j = j + 1
        feedback = nav.getFeedback()
        # print("feedback =", feedback)
        if feedback and i % 5 == 0:
            print("Distance remaining: " + '{:.2f}'.format(
                feedback.distance_remaining) + ' meters.'
                )
    if nav.isTaskComplete():
        print("minibot is arrived!")
        time.sleep(3)
        # nav.goToPose(goal_poses[i+1])
        
        

        
# if 
        
# result = nav.getResult()
# if result == TaskResult.SUCCEEDED:
#     print('Goal succeeded')
# elif result == TaskResult.CANCELED:
#     print('Goal was canceled!')
# elif result == TaskResult.FAILED:
#     print('Goal failed')