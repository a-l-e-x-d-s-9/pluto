#!/usr/bin/env python 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from std_msgs.msg import String, Float64
from pluto_common import *
from control_msgs.msg import JointControllerState
from dynamixel_msgs.msg import JointState as dxl_JointState
from pluto.msg import DetectResult
from pluto.srv import Grip

class MoveArm:
    move_command                = 0
    arm_group                   = 0
    target_base                 = 0
    target_shoulder             = 0
    target_elbow1               = 0
    target_elbow2               = 0
    target_wrist                = 0
    is_accomplished_base        = False
    is_accomplished_shoulder    = False
    is_accomplished_elbow1      = False
    is_accomplished_elbow2      = False
    is_accomplished_wrist       = False
    completion_message          = ""
    error_value                 = 0.02
    shoulder_err_value          = 0.13
    detect_result          	    = 0
    state                   	= 0
    grab_service		        = 0
    scan_counter                = 2
    detect_result_counter       = 1

    def STATE_MOVE_BASE( self ):
        return 1
        
    def STATE_MOVE_ELBOW( self ):
        return 2
    
    def STATE_MOVE_SHOULDER( self ):
        return 3

    def STATE_PREPARE_TO_GRAB( self ):
        return 4

    def STATE_GRAB( self ):
        return 5
        
    
    def detector_image_width( self ):
        if True == self.is_simulation:
            return 640
        else:
            return 640 // 2
        
    def detector_image_height( self ):
        if True == self.is_simulation:
            return 480
        else:
            return 480 // 2
        
    def MOVE_INIT( self ):
        return "INIT"
        
    def INIT_ARM_SCAN( self ):
        return "INIT_ARM_SCAN"
      
    def ARM_PICK( self ):
        return "ARM_PICK"
        
    def TARGET_IGNORE_VALUE( self ):
        return 21.0
    
    def cooldown_joint_move( self ):
        time.sleep( 0.5 )
        pass
    
    def __init__( self ):
        rospy.loginfo( "Arm Movement initialized " )
          
        init_arguments( self )  
        
        #arm publisher and suscriber
        rospy.Subscriber("/pluto/move_arm/command", String, self.move_arm )
        self.move_result_publisher = rospy.Publisher('/pluto/move_arm/done', String, queue_size=10)

        self.target_base        = self.TARGET_IGNORE_VALUE()
        self.target_shoulder    = self.TARGET_IGNORE_VALUE()
        self.target_elbow1      = self.TARGET_IGNORE_VALUE()
        self.target_elbow2      = self.TARGET_IGNORE_VALUE()
        self.target_wrist       = self.TARGET_IGNORE_VALUE()
        
        #joint command publishers
        self.base_command       = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/base_rotation_controller/command' ), Float64, queue_size=10)
        self.shoulder_command   = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/shoulder_controller/command'      ), Float64, queue_size=10)
        self.elbow1_command     = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/elbow1_controller/command'        ), Float64, queue_size=10)
        self.elbow2_command     = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/elbow2_controller/command'        ), Float64, queue_size=10)
        self.wrist_command      = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/wrist_controller/command'         ), Float64, queue_size=10)
        
        if True == self.is_simulation:
            JointType = JointControllerState
        else:
            JointType = dxl_JointState
        
        #joint state suscribers
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/base_rotation_controller/state'    ), JointType, self.validate_state_base_rotation        )
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/shoulder_controller/state'         ), JointType, self.validate_state_shoulder_controller  )
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/elbow1_controller/state'           ), JointType, self.validate_state_elbow1_controller    )
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/elbow2_controller/state'           ), JointType, self.validate_state_elbow2_controller    )
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/wrist_controller/state'            ), JointType, self.validate_state_wrist_controller     )
	
        #ball detector
        self.detect_publisher = rospy.Publisher('/pluto/detect/command', String, queue_size=10)
        rospy.Subscriber("/pluto/detect/result", DetectResult, self.detect_result )
            
        #gripper service
        rospy.wait_for_service('gripper')

        try:
            self.grab_service = rospy.ServiceProxy('gripper', Grip)
            self.grab_service("Open", -1.0)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def move_arm( self, command):
        self.move_command = command.data
  
        if self.move_command == self.MOVE_INIT():
            self.init_arm_pos()

        elif self.move_command == self.INIT_ARM_SCAN():
            self.init_arm_scan()

        elif self.move_command == self.ARM_PICK():
            
            self.state = self.STATE_MOVE_BASE()
            self.detect_publisher.publish("scan_arm")
 
            
        rospy.loginfo("Arm Movement: "+str(self.move_command))

        
    def send_commands_to_arm( self, base, shoulder, elbow1, elbow2, wrist ):
        self.target_base                = base
        self.target_shoulder            = shoulder
        self.target_elbow1              = elbow1
        self.target_elbow2              = elbow2
        self.target_wrist               = wrist
        
        self.is_accomplished_base       = False
        self.is_accomplished_shoulder   = False
        self.is_accomplished_elbow1     = False
        self.is_accomplished_elbow2     = False
        self.is_accomplished_wrist      = False
        
        
    def validate_state_generic( self, current_state, target_value, publisher, is_accomplished, error_value ):
        if target_value != self.TARGET_IGNORE_VALUE():
            
            if False == is_accomplished:

                if True == self.is_simulation:
                    current_state_value = current_state.process_value
                else:
                    current_state_value = current_state.current_pos
                
                is_accomplished = abs(target_value - current_state_value) <= error_value
                
                if True == is_accomplished:
                    rospy.loginfo( "close_to_target: target_value {}, current_state_value {}, abs {}, err {}".format( target_value, current_state_value, abs(target_value - current_state_value) , error_value ) )

            publisher.publish( target_value )
            
        return is_accomplished
                
    def check_arm_move_done_and_publish( self ):
        if self.completion_message != "":
            is_done = ( True == self.is_accomplished_base      ) and \
                      ( True == self.is_accomplished_shoulder  ) and \
                      ( True == self.is_accomplished_elbow1    ) and \
                      ( True == self.is_accomplished_elbow2    ) and \
                      ( True == self.is_accomplished_wrist     ) 
                      
            if True == is_done:
                self.move_result_publisher.publish( self.completion_message )
                self.completion_message = ""
                
                if self.move_command == self.ARM_PICK():
                    self.detect_publisher.publish("scan_arm")
    
    def validate_state_base_rotation( self, state ):
        self.is_accomplished_base       = self.validate_state_generic( state, self.target_base, self.base_command, self.is_accomplished_base , self.error_value )

        self.check_arm_move_done_and_publish()
                
    def validate_state_shoulder_controller( self, state ):
        self.is_accomplished_shoulder   = self.validate_state_generic( state, self.target_shoulder, self.shoulder_command, self.is_accomplished_shoulder, self.shoulder_err_value )
        self.check_arm_move_done_and_publish() 

    def validate_state_elbow1_controller( self, state ):
        self.is_accomplished_elbow1     = self.validate_state_generic( state, self.target_elbow1, self.elbow1_command, self.is_accomplished_elbow1, self.error_value )

        self.check_arm_move_done_and_publish()
        
    def validate_state_elbow2_controller( self, state ):
        self.is_accomplished_elbow2     = self.validate_state_generic( state, self.target_elbow2, self.elbow2_command, self.is_accomplished_elbow2 , self.error_value )
   
        self.check_arm_move_done_and_publish()
        
    def validate_state_wrist_controller( self, state ):
        self.is_accomplished_wrist      = self.validate_state_generic( state, self.target_wrist, self.wrist_command, self.is_accomplished_wrist , self.error_value )
        self.check_arm_move_done_and_publish()
        
    
    def init_arm_pos(self):
        self.send_commands_to_arm( 0.0, 1.6, 0.0, 2.1, 0.0 )
        self.completion_message = "init_arm_done"

    def init_arm_scan(self):
        self.send_commands_to_arm( 0.15, 0.25, 0.0, 2.1, 0.0 )
        self.completion_message = "init_arm_scan_done"
        
    def move_base(self, begin_step, end_step, radius):
        self.completion_message = "move_base_done"
        rospy.loginfo("BASE MOVE")
        if self.detect_result.detected_r <= radius:
            new_target = self.target_base + begin_step
        else:
            new_target = self.target_base + end_step
        
        #self.send_commands_to_arm(new_target, self.target_shoulder, self.target_elbow1, self.target_elbow2, self.target_wrist )
        
        self.target_base = new_target
        #self.is_accomplished_base = False

        
    
    def move_elbow2(self, begin_step, end_step, radius):
        rospy.loginfo("ELBOW MOVE")
        self.completion_message = "move_elbow_done"
        
        if self.detect_result.detected_r <= radius:
            new_target = self.target_elbow2 + begin_step
        else:
            new_target = self.target_elbow2 + end_step
            
        #self.send_commands_to_arm(self.target_base, self.target_shoulder, self.target_elbow1, new_target, self.target_wrist )

        self.target_elbow2 = new_target
        #self.is_accomplished_elbow2 = False
  
        
    
    def move_shoulder(self, begin_step, end_step, radius):
        
        if self.detect_result.detected_r <= radius:
            new_target = self.target_shoulder + begin_step
        else:
            new_target = self.target_shoulder + end_step
        
        #self.send_commands_to_arm(self.target_base, new_target, self.target_elbow1, self.target_elbow2, self.target_wrist )
        self.target_shoulder = new_target
        #self.is_accomplished_shoulder   = False

        self.completion_message = "move_shoulder_done"
        
    def move_shoulder_elbow2_const(self):
        elbow2_new_target = self.target_elbow2 - 0.33
        self.target_elbow2 = elbow2_new_target
        self.target_shoulder = 2
        #self.send_commands_to_arm( self.target_base, 2, self.target_elbow1, elbow2_new_target, self.target_wrist)
        self.completion_message = "shoulder_elbow2_done"
        
    #move joints to get closer to ball
    def move_joint(self, joint_name, direction):
        is_can_move_farther_at_this_direction = True
        
        if joint_name == "base":
            #start with larger steps
            begin_step = 0.05
            end_step = 0.03
            radius = 24
            
            if direction == "left":
                self.move_base(-begin_step, -end_step, radius)
            else:
                self.move_base(begin_step, end_step, radius)
                
        elif joint_name == "elbow2":
            
            begin_step = 0.05
            end_step = 0.02
            radius = 35
            
            if direction == "up":
                self.move_elbow2(-begin_step, -end_step, radius)
            else:
                if (self.target_elbow2 + begin_step) < 2.1:
                    self.move_elbow2(begin_step, end_step, radius)
                else:
                    is_can_move_farther_at_this_direction = False
                
        elif joint_name == "shoulder":
            begin_step = 0.1
            end_step = 0.05
            radius = 35
            
            self.move_shoulder(begin_step, end_step, radius)
        
        self.cooldown_joint_move()
        
        return is_can_move_farther_at_this_direction
        
        
    def get_arm_close_to_ball(self):
	  
        if True == self.detect_result.is_ball_detected :

            if self.state == self.STATE_MOVE_BASE():
                
                BASE_RANGE = 15
                
                middle_x_range_left = (self.detector_image_width() / 2) - BASE_RANGE
                middle_x_range_right = (self.detector_image_width() / 2) + BASE_RANGE
                rospy.loginfo( "STATE_MOVE_BASE: detected_x {}  middle_x_range_left {}  middle_x_range_right {}".format(self.detect_result.detected_x, middle_x_range_left, middle_x_range_right)) 
                
                joint_name = "base"
                if self.detect_result.detected_x <  middle_x_range_left:
                    self.move_joint(joint_name, "left")
                    
                elif self.detect_result.detected_x > middle_x_range_right:
                    self.move_joint(joint_name, "right")
                   
                else:
                    rospy.loginfo("BASE IS IN THE MIDDLE")
   
                    self.state = self.STATE_MOVE_ELBOW()
                    self.detect_publisher.publish("scan_arm")
                    	

            elif self.state == self.STATE_MOVE_ELBOW():	
                rospy.loginfo( " STATE_MOVE_ELBOW")
                
                ELBOW_RANGE = 10
                
                middle_y_range_top = (self.detector_image_height() / 2) + ELBOW_RANGE
                middle_y_range_bot = (self.detector_image_height() / 2) - ELBOW_RANGE
                
                rospy.loginfo( "STATE_MOVE_ELBOW: detected_Y {}  middle_y_range_top {}  middle_y_range_bot {}".format(self.detect_result.detected_y, middle_y_range_top, middle_y_range_bot)) 
                joint_name = "elbow2"
                if self.detect_result.detected_y <  middle_y_range_bot:
                    
                    is_can_move_farther_at_this_direction = self.move_joint(joint_name, "down")
                    if False == is_can_move_farther_at_this_direction:
                        self.state = self.STATE_MOVE_SHOULDER()
                    
                        self.get_arm_close_to_ball()
                    
                 
                elif self.detect_result.detected_y >  middle_y_range_top:
                    self.move_joint(joint_name, "up")
               
                else:
                    #rospy.loginfo( "BALL RADIUS " +str(self.detect_result.detected_r))
                    #check if the radius of the ball is big enough to stop
                    if self.detect_result.detected_r >= 90:
                        self.state = self.STATE_PREPARE_TO_GRAB()
                    else:
                        self.state = self.STATE_MOVE_SHOULDER()
                    
                    self.get_arm_close_to_ball()
            
            elif self.state == self.STATE_MOVE_SHOULDER():	
                rospy.loginfo( " STATE_MOVE_SHOULDER")
                
                
                
                self.move_joint("shoulder", "down")
                
                self.detect_publisher.publish("scan_arm")
                self.state = self.STATE_MOVE_BASE()
      
                

            elif self.state == self.STATE_PREPARE_TO_GRAB():
             
                rospy.loginfo( "state STATE_PREPARE_TO_GRAB " )
                self.move_shoulder_elbow2_const()
    
                self.state = self.STATE_GRAB()

            elif self.state == self.STATE_GRAB():		
                response = 0;	
                response = self.grab_service("Close", 0.04)
                if response != 0:
                    self.target_shoulder = 0.5
                    #self.send_commands_to_arm( self.target_base, 0.5, self.target_elbow1, self.target_elbow2, self.target_wrist)
                    self.completion_message = "shoulder_done"
                    rospy.loginfo("gripper reAding   " + str(response.Reading) )
    
    def detect_result( self, detect_result ):
        rospy.loginfo( "moveArm detect_result " +str(self.detect_result_counter))
        #scan twice to get updated picture
        if self.detect_result_counter == self.scan_counter:
            self.detect_result_counter = 1
            self.detect_result = detect_result
            self.get_arm_close_to_ball()
        else:
            self.detect_result_counter += 1
            self.detect_publisher.publish("scan_arm")


    
if __name__=='__main__':
  try:
    rospy.init_node("pluto_move_arm", anonymous=True)
    move_arm = MoveArm()
    
    while not rospy.is_shutdown():
      rospy.spin()
      
  except rospy.ROSInterruptException:
    pass
