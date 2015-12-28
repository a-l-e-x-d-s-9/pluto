#!/usr/bin/env python 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

class MoveArm:
    move_command = 0
    arm_group = 0
    
    def MOVE_INIT( self ):
	  return "INIT"
    
    #command_to_robot = 0
    def __init__( self ):
	  rospy.loginfo( "Arm Movement initialized " )
	  
	  moveit_commander.roscpp_initialize(sys.argv)
	  self.arm_group = moveit_commander.MoveGroupCommander("arm")
	  #self.move_command = self.MOVE_STOP()

	  #self.command_to_robot = rospy.Publisher('/diff_driver/command', Twist, queue_size=10)
	  
	  rospy.Subscriber("pluto/move_arm/command", String, self.move_arm )
	  self.move_result_publisher = rospy.Publisher('pluto/move_arm/done', String, queue_size=10)
    
    def move_arm( self, command):
	  self.move_command = command.data
      
	  if self.move_command == self.MOVE_INIT():
	
	      rospy.loginfo("Arm Movement: INIT")
	      self.init_arm_pos()
	      rospy.sleep(15)
	      self.move_joints()
	      rospy.sleep(5)
	      self.move_result_publisher.publish( "arm_init_done" )
      
      
    def init_arm_pos(self):
	pose_target = geometry_msgs.msg.Pose()
    
	#position
	pose_target.position.x = 0.74;
	pose_target.position.y = 0.0;
	#height of the arm
	pose_target.position.z = 0.3;
	#orientation
	pose_target.orientation.x = 0.0;
	pose_target.orientation.y = 0.01;
	pose_target.orientation.z = 0.2;
	pose_target.orientation.w = 0.1;
	
	self.arm_group.set_pose_target(pose_target)
	self.arm_group.set_goal_tolerance(0.31);
	plan1 = self.arm_group.plan()
	#move robot arm
	self.arm_group.go(wait=True)
	
    def move_joints(self):
      self.arm_group.clear_pose_targets()
      group_variable_values = self.arm_group.get_current_joint_values()
      print "============ Joint values: ", group_variable_values
      
      group_variable_values[3] = 1.5
      #group_variable_values[1] = 1.5
      self.arm_group.set_joint_value_target(group_variable_values)
      plan2 = self.arm_group.plan()
      self.arm_group.go(wait=True) 
    
if __name__=='__main__':
  try:
    rospy.init_node("pluto_move_arm", anonymous=True)
    move_arm = MoveArm()
    
    while not rospy.is_shutdown():
      rospy.spin()
      
  except rospy.ROSInterruptException:
    pass