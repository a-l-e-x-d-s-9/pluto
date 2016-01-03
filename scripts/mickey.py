#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import String
from pluto.msg import DetectResult
import time
from pluto_common import *

class Mickey:
    state                               = 0
    counter_scans                       = 0
    counter_approches                   = 0
    counter_wonder_straight             = 0
    counter_blind_spot_correction       = 0
    discrete_turns_needed               = 0
    
    counter_scans_limit                 = 30
    counter_approches_limit             = 6
    counter_wonder_straight_limit       = 10
    counter_blind_spot_correction_limit = 0
    
    detect_result                       = 0
    
    detected_close_to_ball              = False
    
    def RATE( self ):
        return 5
        
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
        
    def STATE_IN_BLIND_SPOT( self ):
        return 12
        
    def STATE_IN_BLIND_SPOT_CALLBACK( self ):
        return 13
        
    def STATE_INIT_ARM( self ):
        return 14
        
    def STATE_INIT_ARM_CALLBACK( self ):
        return 15
        
    def CORRECTION_TURN_TOLERANCE( self, detected_close_to_ball ):
        if True == detected_close_to_ball:
            return 0
        else:
            return 1
            
    def DETECTOR_IMAGE_WIDTH( self ):
        return 640
        
    def DETECTOR_IMAGE_HEIGHT( self ):
        return 480
        
    def DETECTOR_DISCRETE_TURN_PIXELS( self ):
        return 30
        
    def cooldown_approch( self ):
        #time.sleep( 1 )
        pass
        
    def cooldown_turn( self ):
        #time.sleep( 0.1 )
        pass
        
    def cooldown_boot( self ):
        time.sleep( 20 )
        
    def cooldown_correction( self ):
        #time.sleep( 0.5 )
        pass
        
    def is_in_close_range( self ):

        if True == self.is_simulation:
            # Gazebo mesurments, ball distance to y value in picture:
            #     4m  : y: 286, r: 2
            #     3m  : y: 304, r: 2
            #     2m  : y: 336, r: 4
            #     1.5m: y: 366, r: 5
            #     1m  : y: 434, r: 6
            return self.detect_result.detected_y > 285;
        else:
            # Real robot, when camera most down
            #     4m  : y: 310
            #     3m  : y: 346
            #     2m  : y: 416
            return self.detect_result.detected_y > 346;
        
        
    def __init__( self ):
        rospy.loginfo( "Mickey initialized " )
        
        init_arguments( self )
        
        self.state = self.STATE_INIT_ARM()

        self.move_publisher = rospy.Publisher('pluto/movement/command', String, queue_size=10)
        rospy.Subscriber("pluto/movement/done", String, self.move_done )
        
        self.detect_publisher = rospy.Publisher('pluto/detect/command', String, queue_size=10)
        rospy.Subscriber("pluto/detect/result", DetectResult, self.detect_result )
        
        self.arm_move_publisher = rospy.Publisher('pluto/move_arm/command', String, queue_size=10)
        rospy.Subscriber('pluto/move_arm/done', String, self.move_done)


    def calculate_turns_needed( self, x, y, r ):

        return ( x // self.DETECTOR_DISCRETE_TURN_PIXELS() ) - ( self.DETECTOR_IMAGE_WIDTH() // self.DETECTOR_DISCRETE_TURN_PIXELS() // 2 )
        
    def move_done( self, done_message ):
        done_message_str = done_message.data
        rospy.loginfo( "Mickey " + done_message_str )
        
        if done_message_str == "arm_init_done":
	    self.state = self.STATE_INIT_ARM_CALLBACK()
        
        self.main_loop()
        
    def detect_result( self, detect_result ):
        
        if "scan_top" == detect_result.request_tag :
            rospy.loginfo( "Mickey detect_result " )
            
            self.detect_result = detect_result
            
            self.main_loop()

    
    def main_loop( self ):
        
        rospy.loginfo( "Mickey main_loop " )
        
        if self.STATE_INIT_ARM() == self.state:
            
            rospy.loginfo( "Mickey STATE_INIT_ARM " )
            
            self.arm_move_publisher.publish("INIT")
            #self.state = self.STATE_INIT_ARM_CALLBACK()
            
            #self.main_loop()
            
        elif self.STATE_INIT_ARM_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_INIT_ARM_CALLBACK " )
            
            self.state = self.STATE_SCAN()
            
            self.main_loop()
        
        elif self.STATE_SCAN() == self.state:
            
            rospy.loginfo( "Mickey STATE_SCAN " )
            
            self.state = self.STATE_SCAN_DETECT_CALLBACK()
            
            self.detect_publisher.publish("scan_top")


        elif self.STATE_SCAN_DETECT_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_SCAN_DETECT_CALLBACK " )
            
            self.counter_scans = self.counter_scans + 1
            
            if True == self.detect_result.is_ball_detected:
                
                self.discrete_turns_needed = self.calculate_turns_needed( self.detect_result.detected_x, self.detect_result.detected_y, self.detect_result.detected_r )
                
                if abs( self.discrete_turns_needed ) <= self.CORRECTION_TURN_TOLERANCE( self.detected_close_to_ball ):
                    
                    self.state                  = self.STATE_APPROACH()
                    self.counter_approches      = 0
                    
                    if False == self.detected_close_to_ball:
                        self.detected_close_to_ball = self.is_in_close_range()
                    
                else:
                    self.state              = self.STATE_CORRECT_DIRECTION()
                    
                if True == self.detected_close_to_ball:
                    self.counter_approches_limit = 1
                    rospy.loginfo( "Mickey detected_close_to_ball " )
                
                self.main_loop()
                    
            else:
                
                if True == self.detected_close_to_ball:
                    
                    self.state = self.STATE_IN_BLIND_SPOT()
                    self.counter_blind_spot_correction = 0;
                    
                    self.main_loop()
                    
                else:
                    
                    self.state = self.STATE_SCAN_TURN_CALLBACK()
                
                    self.move_publisher.publish("RIGHT")
                    
                
        elif self.STATE_SCAN_TURN_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_SCAN_TURN_CALLBACK " )
            
            self.cooldown_turn()
            
            self.state = self.STATE_SCAN()
            
            self.main_loop()
            
        elif self.STATE_CORRECT_DIRECTION() == self.state:
            
            rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION " )
            
            if abs(self.discrete_turns_needed) <= self.CORRECTION_TURN_TOLERANCE(self.detected_close_to_ball):
                
                # rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION -> SCAN" )
                
                self.cooldown_correction()
                
                self.state = self.STATE_SCAN()
                
                self.main_loop()
                
            else:
                
                self.state = self.STATE_CORRECT_DIRECTION_CALLBACK()
                
                if 0 < self.discrete_turns_needed:
                    
                    # rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION -> RIGHT {}".format(self.discrete_turns_needed) )
                    
                    self.discrete_turns_needed = self.discrete_turns_needed - 1
                    
                    self.move_publisher.publish("RIGHT")
                    
                else:
                    
                    # rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION -> LEFT {}".format(self.discrete_turns_needed) )
                    
                    self.discrete_turns_needed = self.discrete_turns_needed + 1
                    
                    self.move_publisher.publish("LEFT")
                
            
        elif self.STATE_CORRECT_DIRECTION_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_CORRECT_DIRECTION_CALLBACK " )
            
            self.state = self.STATE_CORRECT_DIRECTION()
            
            self.main_loop()
            
        elif self.STATE_APPROACH() == self.state:
            
            rospy.loginfo( "Mickey STATE_APPROACH " )
            
            if self.counter_approches >= self.counter_approches_limit:
                
                self.cooldown_approch()
                
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
            
            rospy.loginfo( "Mickey STATE_ARM_PICK " )
            # TODO: Add arm related code
            

        elif self.STATE_ARM_PICK_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_ARM_PICK_CALLBACK " )
            # TODO: Add arm related code

        elif self.STATE_IN_BLIND_SPOT()  == self.state:
            
            rospy.loginfo( "Mickey STATE_IN_BLIND_SPOT " )

            if self.counter_blind_spot_correction >= self.counter_blind_spot_correction_limit:
                
                self.state = self.STATE_ARM_PICK()
            
                self.main_loop()
                
            else:
                
                self.state = self.STATE_IN_BLIND_SPOT_CALLBACK()
                
                self.move_publisher.publish("FORWARD")

        elif self.STATE_IN_BLIND_SPOT_CALLBACK() == self.state:
            
            rospy.loginfo( "Mickey STATE_IN_BLIND_SPOT_CALLBACK " )
            
            self.counter_blind_spot_correction = self.counter_blind_spot_correction + 1
            
            self.state = self.STATE_ARM_PICK()
            
            self.main_loop()
            
        else:
            raise Exception('Unexisting state.')


if __name__ == '__main__':
    try:
        rospy.init_node("pluto_mickey")
        mickey = Mickey()
        
        
        while not rospy.is_shutdown():
            
            mickey.cooldown_boot()
            mickey.main_loop()
            rospy.spin()
            
            
    except rospy.ROSInterruptException:
        pass
