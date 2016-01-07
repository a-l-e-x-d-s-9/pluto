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
    error_value                 = 0.13
    
    def MOVE_INIT( self ):
        return "INIT"
        
    def INIT_ARM_SCAN( self ):
        return "INIT_ARM_SCAN"
        
    def TARGET_IGNORE_VALUE( self ):
        return 21.0
    
    #command_to_robot = 0
    def __init__( self ):
        rospy.loginfo( "Arm Movement initialized " )
          
        init_arguments( self )  
        
        rospy.Subscriber("/pluto/move_arm/command", String, self.move_arm )
        self.move_result_publisher = rospy.Publisher('/pluto/move_arm/done', String, queue_size=10)
        
        self.target_base        = self.TARGET_IGNORE_VALUE()
        self.target_shoulder    = self.TARGET_IGNORE_VALUE()
        self.target_elbow1      = self.TARGET_IGNORE_VALUE()
        self.target_elbow2      = self.TARGET_IGNORE_VALUE()
        self.target_wrist       = self.TARGET_IGNORE_VALUE()
        
        self.base_command       = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/base_rotation_controller/command' ), Float64, queue_size=10)
        self.shoulder_command   = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/shoulder_controller/command'      ), Float64, queue_size=10)
        self.elbow1_command     = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/elbow1_controller/command'        ), Float64, queue_size=10)
        self.elbow2_command     = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/elbow2_controller/command'        ), Float64, queue_size=10)
        self.wrist_command      = rospy.Publisher( pluto_add_namespace( self.is_simulation, '/wrist_controller/command'         ), Float64, queue_size=10)
        
        if True == self.is_simulation:
            JointType = JointControllerState
        else:
            JointType = dxl_JointState
        
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/base_rotation_controller/state'    ), JointType, self.validate_state_base_rotation        )
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/shoulder_controller/state'         ), JointType, self.validate_state_shoulder_controller  )
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/elbow1_controller/state'           ), JointType, self.validate_state_elbow1_controller    )
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/elbow2_controller/state'           ), JointType, self.validate_state_elbow2_controller    )
        rospy.Subscriber( pluto_add_namespace( self.is_simulation, '/wrist_controller/state'            ), JointType, self.validate_state_wrist_controller     )
    
    def move_arm( self, command):
        self.move_command = command.data
  
        if self.move_command == self.MOVE_INIT():
            self.init_arm_pos()
            #time.sleep(10)
            #self.move_result_publisher.publish("init_arm_done")
        elif self.move_command == self.INIT_ARM_SCAN():
            self.init_arm_scan()
            
        rospy.loginfo("Arm Movement: "+str(self.move_command))
            #time.sleep(10)
            #self.move_result_publisher.publish("init_arm_scan_done")

        
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
        
        
    def validate_state_generic( self, current_state, target_value, publisher, is_accomplished ):
        if target_value != self.TARGET_IGNORE_VALUE():
            
            if False == is_accomplished:

                if True == self.is_simulation:
                    current_state_value = current_state.process_value
                else:
                    current_state_value = current_state.current_pos
                    
                is_accomplished = abs(target_value - current_state_value) <= self.error_value
            
                if True == is_accomplished:
                    rospy.loginfo( "close_to_target, target_value {}, current_state_value {}, abs {}, err {}".format(target_value, current_state_value, abs(target_value - current_state_value) , self.error_value) )

                   
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
    
    def validate_state_base_rotation( self, state ):
        self.is_accomplished_base       = self.validate_state_generic( state, self.target_base, self.base_command, self.is_accomplished_base )
        self.check_arm_move_done_and_publish()
                
    def validate_state_shoulder_controller( self, state ):
        self.is_accomplished_shoulder   = self.validate_state_generic( state, self.target_shoulder, self.shoulder_command, self.is_accomplished_shoulder )
        self.check_arm_move_done_and_publish() 
        
    def validate_state_elbow1_controller( self, state ):
        self.is_accomplished_elbow1     = self.validate_state_generic( state, self.target_elbow1, self.elbow1_command, self.is_accomplished_elbow1 )
        self.check_arm_move_done_and_publish()
        
    def validate_state_elbow2_controller( self, state ):
        self.is_accomplished_elbow2     = self.validate_state_generic( state, self.target_elbow2, self.elbow2_command, self.is_accomplished_elbow2 )
        self.check_arm_move_done_and_publish()
        
    def validate_state_wrist_controller( self, state ):
        self.is_accomplished_wrist      = self.validate_state_generic( state, self.target_wrist, self.wrist_command, self.is_accomplished_wrist )
        self.check_arm_move_done_and_publish()
        
    
    def init_arm_pos(self):
        self.send_commands_to_arm( 0.0, 1.6, 0.0, 2.1, 0.0 )
        self.completion_message = "init_arm_done"

    def init_arm_scan(self):
        self.send_commands_to_arm( 0.0, 0.5, 0.0, 2.1, 0.0 )
        self.completion_message = "init_arm_scan_done"
    
if __name__=='__main__':
  try:
    rospy.init_node("pluto_move_arm", anonymous=True)
    move_arm = MoveArm()
    
    while not rospy.is_shutdown():
      rospy.spin()
      
  except rospy.ROSInterruptException:
    pass
