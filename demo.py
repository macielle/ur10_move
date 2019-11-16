#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveManipulator(object):
  """MoveManipulator"""
  def __init__(self):
    super(MoveManipulator, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_manipulator', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
	
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    planning_frame = group.get_planning_frame()
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    

  def go_to_waypoint(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.5
    pose_goal.position.z = 0.5
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

  def go_to_object(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = 0.5
    pose_goal.position.z = 0.1
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

  def go_to_place(self):
    group = self.group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.25
    pose_goal.position.y = 0.75
    pose_goal.position.z = 0.1
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
  
  def go_to_home(self):
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = pi/2
    joint_goal[1] = -pi/2
    joint_goal[2] = pi/3
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    group.go(joint_goal, wait=True)
    group.stop()
    current_joints = group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.1)

def main():

  try:
    print "============ Press `Enter` to begin the sequence by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveManipulator()
    print "============ Press `Enter` to pick the object ..."
    raw_input()
    print "============ Attempting home position."
    tutorial.go_to_home()
    print "============ Home position executed.Press 'Enter' to continue."
    raw_input()
    tutorial.go_to_waypoint()
    print "============ Waypoint executed."
    tutorial.go_to_object()
    print "============ Object picked!"
    print "============ Press 'Enter' to place the object ..."
    raw_input()
    tutorial.go_to_place()
    print "============ Object placed!"
    print "============ Press 'Enter' to return home..."
    raw_input()
    tutorial.go_to_home()
    print "============ Done!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
