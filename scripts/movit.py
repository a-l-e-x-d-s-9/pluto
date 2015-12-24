#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  robot = moveit_commander.RobotCommander()

  scene = moveit_commander.PlanningSceneInterface()

  group = moveit_commander.MoveGroupCommander("arm")

  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

  print "============ Waiting for RVIZ..."
  rospy.sleep(10)
  print "============ Starting tutorial "

  print "============ Reference frame: %s" % group.get_planning_frame()
  print "============ Reference frame: %s" % group.get_end_effector_link()
  print "============ Robot Groups:"
  print robot.get_group_names()
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
  
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()

  pose_target.orientation.w = 1.0;
  pose_target.orientation.x = 0.0;
  pose_target.orientation.y = 0.0;
  pose_target.orientation.z = 0.0;
  pose_target.position.x = 0.5;
  pose_target.position.y = 0.0;
  pose_target.position.z = 0.5;
  group.set_pose_target(pose_target)
  group.set_goal_tolerance(0.31);
  plan1 = group.plan()

  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(5)
  
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  print "============ Waiting while plan1 is visualized (again)..."
  rospy.sleep(5)
  group.go(wait=True)
  
if __name__=='__main__':
  
  try:
    rospy.init_node("movit")
    move_group_python_interface_tutorial()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass