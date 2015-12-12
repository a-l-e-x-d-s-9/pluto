#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Movement:
    counter          = 1
    command_api      = 0
    move_command     = "STOP"

    def SAMPLE_FREQUENCY( self ):
        return 20
        
    def MOVE_LEFT( self ):
        return "LEFT"
        
    def MOVE_RIGHT( self ):
        return "RIGHT"
        
    def MOVE_FORWARD( self ):
        return "FORWARD"
        
    def MOVE_BACKWARD( self ):
        return "BACKWARD"
        
    def MOVE_STOP( self ):
        return "STOP"
        
    def SPEED_LINEAR( self ):
        return 0.5
        
    def SPEED_ANGULAR( self ):
        return 0.5
        
    def __init__( self ):
        rospy.loginfo( "Movement innitialized " )
        self.command_api = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)
        rospy.Subscriber("pluto_movement_command", String, pluto_movement_command_callback)

    def move_linear( self, velocity ):
        
        move_robot_command = Twist()

        move_robot_command.linear.x = velocity
        self.command_api.publish(move_robot_command)
        
    def move_turn( self, velocity ):
        
        move_robot_command = Twist()

        move_robot_command.angular.z = velocity
        self.command_api.publish(move_robot_command)
        
    def move_stop( self ):
        move_robot_command = Twist()

        self.command_api.publish(move_robot_command)

    def move_receive_command( self, command ):
        self.move_command = command
            
            
    def move( self ):
        if      self.MOVE_LEFT()      == self.move_command:
        
            rospy.loginfo("Movement: LEFT")
            self.move_turn( self.SPEED_ANGULAR() )
            
        elif    self.MOVE_RIGHT()     == self.move_command:
        
            rospy.loginfo("Movement: RIGHT")
            self.move_turn( -self.SPEED_ANGULAR() )
            
        elif    self.MOVE_FORWARD()   == self.move_command:
        
            rospy.loginfo("Movement: FORWARD")
            self.move_linear( self.SPEED_LINEAR() )
            
        elif    self.MOVE_BACKWARD()  == self.move_command:
        
            rospy.loginfo("Movement: BACKWARD")
            self.move_linear( -self.SPEED_LINEAR() )
            
        elif    self.MOVE_STOP()      == self.move_command:
        
            rospy.loginfo("Movement: STOP")
            self.move_stop()
            
        else:
            rospy.loginfo("Movement: unknown command! \"{0}\"".format(self.move_command))
            self.move_stop()
        
'''

import rospy
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

    def set_goal( self ):
        #rospy.init_node("simple_navigation_goals")
        # rate = rospy.Rate(10) # 10hz
        
        rospy.loginfo("Started: set_goal, {}".format(self.counter) )
        
        move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Connecting to server, {}".format(self.counter) )
        
        move_base_client.wait_for_server(rospy.Duration(5.0))

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = '/komodo_1/base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 5.0
        goal.target_pose.pose.orientation.w = 5.0

        rospy.loginfo("Sending goal, {}".format(self.counter) )
        
        move_base_client.send_goal(goal)
        
        
        move_base_client.wait_for_result()
        
        if move_base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Hooray, the base moved 1 meter forward')
        else:
            rospy.loginfo('The base failed to move forward 1 meter for some reason')
        
        # rate.sleep()
        
        self.counter = self.counter + 1
        #time.sleep(1000)
        
if __name__ == '__main__':
    try:
        time.sleep(20)
        rospy.init_node("simple_navigation_goals")
        movement = Movement()
        #mc = MyClass()

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            movement.set_goal()

            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass
'''

movement = 0

def pluto_movement_command_callback( command ):
    global movement
    movement.move_receive_command( command.data )
        
if __name__ == '__main__':
    try:
        global movement
        rospy.init_node("move pluto")
        movement = Movement()

        rate = rospy.Rate( movement.SAMPLE_FREQUENCY() )
        while not rospy.is_shutdown():

            movement.move()

            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass
