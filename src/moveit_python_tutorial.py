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
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
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

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_python_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "xarm7"
    group = moveit_commander.MoveGroupCommander(group_name)

    #planning_frame = group.get_planning_frame()
    #print ("============ Reference frame: %s" % planning_frame)

    #eef_link = group.get_end_effector_link()
    #print ("============ End effector: %s" % eef_link)

    #group_names = robot.get_group_names()
    #print ("============ Robot Groups:", robot.get_group_names())

    #print ("============ Printing robot state")
    #print (robot.get_current_state())
    #print ("")

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    #self.planning_frame = planning_frame
    #self.eef_link = eef_link
    #self.group_names = group_names

  def go_to_pose_goal(self):
    
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    #plan = group.plan()
    #group.go(wait=True)
    #group.stop()
    plan_success, traj, planning_time, error_code = group.plan()
    
    current_pose = self.group.get_current_pose().pose
    return traj, all_close(pose_goal, current_pose, 0.01)

  def execute_plan(self, plan):
    
    group = self.group

    #group.go(wait=True)
    group.execute(plan, wait=True)

def main():
  try:
    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
    input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print ("============ Press `Enter` to plan a movement using a pose goal ...")
    input()
    plan, bool_debug = tutorial.go_to_pose_goal()

    print ("============ Press `Enter` to execute a saved path ...")
    input()
    tutorial.execute_plan(plan)

    print ("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()