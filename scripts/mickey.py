#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import String
from pluto.msg import DetectResult
import time

class Mickey:
    state                           = 0
    counter_scans                   = 0
    counter_approches               = 0
    counter_wonder_straight         = 0
    
    counter_scans_limit             = 30
    counter_approches_limit         = 6
    counter_wonder_straight_limit   = 10
    
    detect_result                   = 0
    
    def RATE( self ):
        return 5
        
    def BOOT_TIME_SECONDS( self ):
        return 20
        
    def STATE_SCAN( self ):
        return 0
        
    def STATE_SCAN_DETECT_CALLBACK( self ):
        return 1
        
    def STATE_SCAN_TURN_CALLBACK( self ):
        return 2
        
    def STATE_CORRECT_DIRECTION( self ):
        return 3
        
    def STATE_CORRECT_DIRECTION_CALLBACK( self ):
        return 4
        
    def STATE_APPROACH( self ):
        return 5
        
    def STATE_APPROACH_CALLBACK( self ):
        return 6
        
    def STATE_WAIT( self ):
        return 7
        
    def STATE_WONDER( self ):
        return 8
        
    def STATE_WONDER_CALLBACK( self ):
        return 9
        
    def STATE_ARM_PICK( self ):
        return 10
        
    def STATE_ARM_PICK_CALLBACK( self ):
        return 11
        
    def __init__( self ):
        rospy.loginfo( "Mickey innitialized " )
        
        self.state = self.STATE_SCAN()

        self.move_publisher = rospy.Publisher('pluto/movement/command', String, queue_size=10)
        rospy.Subscriber("pluto/movement/done", String, self.move_done )
        
        self.detect_publisher = rospy.Publisher('pluto/detect/command', String, queue_size=10)
        rospy.Subscriber("pluto/detect/result", DetectResult, self.detect_result )

    def move_done( self, done_message ):
        
        rospy.loginfo( "Mickey move_done " )
        
        self.main_loop()
        
    def detect_result( self, detect_result ):
        
        rospy.loginfo( "Mickey detect_result " )
        
        self.detect_result = detect_result
        
        self.main_loop()

    
    def main_loop( self ):
        rospy.loginfo( "Mickey main_loop " )
        
        if self.STATE_SCAN() == self.state:
            
            rospy.loginfo( "Mickey STATE_SCAN " )
            
            self.state = self.STATE_SCAN_DETECT_CALLBACK()
            
            self.detect_publisher.publish("please_scan")


        elif self.STATE_SCAN_DETECT_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_SCAN_DETECT_CALLBACK " )
            
            self.counter_scans = self.counter_scans + 1
            
            if True == self.detect_result.is_ball_detected:
                
                # TODO: Add a termination condition, to start work with arm!
                
                if 0 == self.detect_result.discrete_turns_needed:
                    
                    self.state              = self.STATE_APPROACH()
                    self.counter_approches  = 0
                    self.main_loop()
                    
                else:
                    self.state = self.STATE_CORRECT_DIRECTION()
                    
                    self.main_loop()
                    
            else:
                
                self.state = self.STATE_SCAN_TURN_CALLBACK()
                
                self.move_publisher.publish("RIGHT")
                
        elif self.STATE_SCAN_TURN_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_SCAN_TURN_CALLBACK " )
            
            self.state = self.STATE_SCAN()
            
            self.main_loop()
            
        elif self.STATE_CORRECT_DIRECTION() == self.state:
            
            rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION " )
            
            if 0 == self.detect_result.discrete_turns_needed:
                
                # rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION -> SCAN" )
                
                self.state = self.STATE_SCAN()
                
                self.main_loop()
                
            else:
                
                self.state = self.STATE_CORRECT_DIRECTION_CALLBACK()
                
                if 0 < self.detect_result.discrete_turns_needed:
                    
                    # rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION -> RIGHT {}".format(self.detect_result.discrete_turns_needed) )
                    
                    self.detect_result.discrete_turns_needed = self.detect_result.discrete_turns_needed - 1
                    
                    self.move_publisher.publish("RIGHT")
                    
                else:
                    
                    # rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION -> LEFT {}".format(self.detect_result.discrete_turns_needed) )
                    
                    self.detect_result.discrete_turns_needed = self.detect_result.discrete_turns_needed + 1
                    
                    self.move_publisher.publish("LEFT")
                
            
        elif self.STATE_CORRECT_DIRECTION_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION_CALLBACK " )
            
            self.state = self.STATE_CORRECT_DIRECTION()
            
            self.main_loop()
            
        elif self.STATE_APPROACH() == self.state:
            
            rospy.loginfo( "Mickey STATE_APPROACH " )
            
            if self.counter_approches >= self.counter_approches_limit:
                
                self.counter_approches = 0
                
                self.state = self.STATE_SCAN()
                
                self.main_loop()
                
            else:
                
                self.state = self.STATE_APPROACH_CALLBACK()
                
                self.move_publisher.publish("FORWARD")
                
            
        elif self.STATE_APPROACH_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_APPROACH_CALLBACK " )
            
            self.counter_approches = self.counter_approches + 1
            
            self.state = self.STATE_APPROACH()
            
            self.main_loop()
            
        elif self.STATE_WAIT() == self.state:
            raise Exception('Unsupported state.')
        elif self.STATE_WONDER() == self.state:
            raise Exception('Unsupported state.')
        elif self.STATE_WONDER_CALLBACK() == self.state:
            raise Exception('Unsupported state.')
        elif self.STATE_ARM_PICK() == self.state:
            raise Exception('Unsupported state.')
        elif self.STATE_ARM_PICK_CALLBACK() == self.state:
            raise Exception('Unsupported state.')
        else:
            raise Exception('Unexisting state.')


if __name__ == '__main__':
    try:
        rospy.init_node("pluto_mickey")
        mickey = Mickey()
        
        
        while not rospy.is_shutdown():
            
            time.sleep( mickey.BOOT_TIME_SECONDS() )
            mickey.main_loop()
            rospy.spin()
            
            
    except rospy.ROSInterruptException:
        pass
