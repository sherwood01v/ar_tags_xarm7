#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from math import pi
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from moveit_commander.conversions import pose_to_list

param = 0
k = 0

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

def callback(data):
    global marker_pose, param
    #print("I am in callback", data)
    if param == 1:
        #print("I am in callback")
        for marker in data.markers:
            if marker.id == int(marker_ids[k]):
                marker_pose = marker.pose.pose
                print("Marker is set!", marker.id)
        param = 0

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

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group

#   def callback(data):
#     global marker_pose, param
#     print("I am in callback", data)
#     if param == 1:
#         #print("I am in callback")
#         for marker in data.markers:
#             if marker.id == int(marker_ids[k]):
#                 marker_pose = marker.pose.pose
#                 print("Marker is set!")
#         param = 0

  def go_to_pose_goal(self):
    
    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    current_pose = group.get_current_pose().pose
    pose_goal = current_pose

    marker_orientation = (
      marker_pose.orientation.x,
      marker_pose.orientation.y,
      marker_pose.orientation.z,
      marker_pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(marker_orientation)
    print(euler)
    roll = 3.14 + euler[0]
    pitch = euler[1]
    yaw = euler[2]
    print(roll, pitch, yaw)
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    # pose_goal.orientation.x = marker_pose.orientation.x
    # pose_goal.orientation.y = marker_pose.orientation.y
    # pose_goal.orientation.z = marker_pose.orientation.z
    # pose_goal.orientation.w = marker_pose.orientation.w

    pose_goal.position.x = marker_pose.position.x
    pose_goal.position.y = marker_pose.position.y
    pose_goal.position.z = marker_pose.position.z
    print(pose_goal)
    group.set_pose_target(pose_goal)

    #plan = group.plan()
    #group.go(wait=True)
    #group.stop()
    plan_success, traj, planning_time, error_code = group.plan()
    group.clear_pose_targets()
    
    current_pose = self.group.get_current_pose().pose
    return traj, all_close(pose_goal, current_pose, 0.01)

  def execute_plan(self, plan):
    global param, k
    
    group = self.group

    #group.go(wait=True)
    group.execute(plan, wait=True)
    k = k+1
    #param = 0

  def start_position(self):
    
    group = self.group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0.36
    joint_goal[1] = -0.68
    joint_goal[2] = 0
    joint_goal[3] = 0.83
    joint_goal[4] = 0
    joint_goal[5] = 1.5
    joint_goal[6] = 3

    group.go(joint_goal, wait=True)

    #group.stop()

def main():
  try:
    global param, k, marker_ids
    print ("============ Press `Enter` to begin by setting up the moveit_commander")
    input()
    tutorial = MoveGroupPythonIntefaceTutorial()
    marker_ids = rospy.get_param('~marker_ids').split(",")
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("ar_tf_markers", AlvarMarkers, callback)

    for i in range (0, len(marker_ids)):
        print ("============ Press `ENTER` to set up the next marker")
        input()
        #print("Go to callback")
        #rospy.Subscriber("ar_tf_markers", AlvarMarkers, tutorial.callback())
        param = 1
        print ("============ Press `ENTER` to plan a movement")
        input()
        plan, bool_debug = tutorial.go_to_pose_goal()
        print ("============ Press `ENTER` to execute a saved path")
        input()
        tutorial.execute_plan(plan)
        print ("============ Press `ENTER` to return to start position")
        input()
        tutorial.start_position()

    #print ("============ Type `finish` to finish to finish navigation to the marker")

    print ("============ Demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()