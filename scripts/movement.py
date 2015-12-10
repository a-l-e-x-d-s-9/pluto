#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def move_forward():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world 11 %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        
        walkPub = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)
        rate = rospy.Rate(10)
        walkTwist = Twist()

        walkTwist.linear.x = 0.3
        walkPub.publish(walkTwist)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
        
'''
import rospy
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospy.init_node("simple_navigation_goals")
move_base_client = SimpleActionClient('move_base', MoveBaseAction)
rospy.loginfo('Connecting to server')
move_base_client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'komodo_1/base_link'
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = -5.0
goal.target_pose.pose.orientation.w = 5.0

rospy.loginfo('Sending goal')
move_base_client.send_goal(goal)

move_base_client.wait_for_result()

if move_base_client.get_state() == GoalStatus.SUCCEEDED:
    rospy.loginfo('Hooray, the base moved 1 meter forward')
else:
    rospy.loginfo('The base failed to move forward 1 meter for some reason')
'''
