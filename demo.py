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
# Author: Acorn Pooley, Mike Lautman

#I am using a modified version of the MoveIt Python interface tutorial found at 
#http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
#See the above statements for all copyright information.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class MoveManipulator(object):
  """MoveManipulator"""
  def __init__(self):
  
  #intialization of moveit_commander and a rospy node
    super(MoveManipulator, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_manipulator', anonymous=True)
    
    #instatiate RobotCommander object 
    robot = moveit_commander.RobotCommander()
    
    #instantiate Planning Scene Interface object, 
    #represents the world surrounding the robot
    scene = moveit_commander.PlanningSceneInterface()

    #instantiate MoveGroupCommander object that represents a group of joints
    #in the UR package, the UR10e is represented by the planning group 'manipulator'
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
	
    #DisplayTrajectory publisher publishes trajectories for RViz to visualize	
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    
    #get name of reference frame
    planning_frame = group.get_planning_frame()
    
    #define variables in the environment
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    
  #function for robot to move to waypoint above desired coordinates
  #here is where you would specify the location of the desired object
  def go_to_waypoint(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.5
    pose_goal.position.z = 0.5
    group.set_pose_target(pose_goal)

    #execute plan and then stop, clear targets
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

  #arm lowers to the object from the waypoint
  def go_to_object(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.5
    pose_goal.position.z = 0.1
    group.set_pose_target(pose_goal)

    #execute plan and then stop, clear targets
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

 
  #function to move the robot to another set of coordinates
  #here is where you would specify where you want the object to be placed
  def go_to_place(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.25
    pose_goal.position.y = 0.75
    pose_goal.position.z = 0.1
    group.set_pose_target(pose_goal)

    #execute plan and then stop, clear targets
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
  
  #function to move robot to home position, which was chosen for optimal camera view
  #home is defined by joint values rather than xyz coordiantes
  def go_to_home(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    #shoulder_pan_joint
    joint_goal[0] = pi/2 
    #shoulder_lift_joint
    joint_goal[1] = -pi/2 
    #elbow_joint	
    joint_goal[2] = pi/3 
    #wrist_1_joint
    joint_goal[3] = 0 
    #wrist_2_joint	
    joint_goal[4] = 0 
    #wrist_3_joint
    joint_goal[5] = 0 
	
    #execute plan, then stop	
    group.go(joint_goal, wait=True)
    group.stop()

def main():
  #terminal interface
  #the user is required to press Enter between each movement for verification purposes
  try:
    #script starts by initializing
    print "============ Press `Enter` to begin the sequence (press ctrl-d to exit) ..."
    raw_input() 
    tutorial = MoveManipulator()
    #pick sequence starts by moving to the home position
    print "============ Press `Enter` to pick the object ..."
    raw_input()
    print "============ Attempting home position."
    tutorial.go_to_home()
    print "============ Home position executed. Press 'Enter' to continue."
    raw_input()
    #the robot then moves to the waypoint above the object
    print "============ Attempting waypoint."
    tutorial.go_to_waypoint()
    print "============ Waypoint executed. Press 'Enter' to continue."
    raw_input()
    tutorial.go_to_object()
    #robot arm lowers to object
    print "============ Object picked!"
    print "============ Press 'Enter' to place the object ..."
    raw_input()
    #robot arm places the object at some other coordinate
    tutorial.go_to_place()
    print "============ Object placed!"
    print "============ Press 'Enter' to return home..."
    raw_input()
    #returns to home position
    tutorial.go_to_home()
    print "============ Done!"

  #keyboard interrupt
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
