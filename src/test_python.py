#!/usr/bin/env python

import sys
import copy
import rospy
import tf
from tf import TransformListener
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('test_python', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("xarm7")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

current_pose = group.get_current_pose().pose
#print(current_pose)

pose_goal = geometry_msgs.msg.Pose()
pose_goal = current_pose

#Without any transform
# pose_goal.orientation.x = -0.991340977383005
# pose_goal.orientation.y = -1.645654718752489e-05
# pose_goal.orientation.z = 4.572084246769934e-06
# pose_goal.orientation.w = 1.9124301266909828e-05
# pose_goal.position.x = -0.09198567484376788
# pose_goal.position.y = -0.12695438903298856
# pose_goal.position.z = 0.6799618592785727

#After transform from usb_cam with no rotation transform to base
# pose_goal.orientation.x = 0.992546027453606
# pose_goal.orientation.y = 0.02805216296427737
# pose_goal.orientation.z = 0.09166410601775578
# pose_goal.orientation.w = 0.9961579231132731
# pose_goal.position.x = -0.05443288362902329
# pose_goal.position.y = -0.2951517934651171
# pose_goal.position.z = 0.6182630098192186

#After correct transforms
# pose_goal.orientation.x = 0.0
# pose_goal.orientation.y = 0.0
# pose_goal.orientation.z = 0.0
# pose_goal.orientation.w = 1.0
# pose_goal.position.x = 0.4228705820983594
# pose_goal.position.y = -0.2613678168141453
# pose_goal.position.z = 0.4746613555971296

roll = 0
pitch = 0
yaw = 1.57
quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
pose_goal.orientation.x = quaternion[0]
pose_goal.orientation.y = quaternion[1]
pose_goal.orientation.z = quaternion[2]
pose_goal.orientation.w = quaternion[3]
pose_goal.position.x = 0.5
pose_goal.position.y = -0.2
pose_goal.position.z = 0.3

group.set_pose_target(pose_goal)
print(pose_goal)

plan = group.plan()
group.go(wait=True)

#group.stop()
group.clear_pose_targets()
#group.execute(plan, wait=True)
#rospy.sleep(5)

moveit_commander.roscpp_shutdown()