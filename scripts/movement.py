#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class Movement:
    counter          = 1
    command_to_robot = 0
    move_command     = 0

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
        
    def MOVE_TIME_SECONDS( self ):
        return 0.6
        
    def __init__( self ):
        rospy.loginfo( "Movement innitialized " )
        
        self.move_command = self.MOVE_STOP()

        self.command_to_robot = rospy.Publisher('/komodo_1/diff_driver/command', Twist, queue_size=10)
        
        rospy.Subscriber("pluto/movement/command", String, self.move )
        self.move_result_publisher = rospy.Publisher('pluto/movement/done', String, queue_size=10 )

    def move_linear( self, velocity ):
        
        move_robot_command = Twist()

        move_robot_command.linear.x     = velocity
        move_robot_command.linear.y     = 0
        move_robot_command.linear.z     = 0
        move_robot_command.angular.x    = 0
        move_robot_command.angular.y    = 0
        move_robot_command.angular.z    = 0
        
        self.command_to_robot.publish(move_robot_command)
        
    def move_turn( self, velocity ):
        
        move_robot_command = Twist()

        move_robot_command.linear.x     = 0
        move_robot_command.linear.y     = 0
        move_robot_command.linear.z     = 0
        move_robot_command.angular.x    = 0
        move_robot_command.angular.y    = 0
        move_robot_command.angular.z    = velocity
        
        self.command_to_robot.publish(move_robot_command)
        
    def move_stop( self ):
        move_robot_command = Twist()

        self.command_to_robot.publish(move_robot_command)

    #def move_receive_command( self, command ):
    #    self.move_command = command
            
            
    def move( self, command ):
        self.move_command = command.data
        
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
        
        time.sleep( self.MOVE_TIME_SECONDS() )
        self.move_stop()
        # rospy.loginfo("Movement: before done")
        self.move_result_publisher.publish( "move_done" )
        

'''
movement = 0

def pluto_movement_command_callback( command ):
    global movement
    movement.move_receive_command( command.data )
'''

if __name__ == '__main__':
    try:
        # lobal movement
        rospy.init_node("pluto_movement")
        movement = Movement()

        #rate = rospy.Rate( movement.SAMPLE_FREQUENCY() )
        while not rospy.is_shutdown():

            rospy.spin()

            #rate.sleep()
        
    except rospy.ROSInterruptException:
        pass

