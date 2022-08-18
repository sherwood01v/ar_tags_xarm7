import rospy
import sys
import moveit_commander
import geometry_msgs.msg
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from moveit_commander.conversions import pose_to_list

black_list_markers = []
param = 0
k = 0

def callback(data):
    global marker_pose, storage_pose, param, marker_id_stor_1, marker_id_obj_1, marker_id_stor_2, marker_id_obj_2

    if param == 1:
        for marker in data.markers:
            if marker.id == marker_id_obj_1 and marker.id not in black_list_markers:
                marker_pose = marker.pose.pose
                for marker in data.markers:
                    if marker.id == marker_id_stor_1:
                        storage_pose = marker.pose.pose
            elif marker.id == marker_id_obj_2 and marker.id not in black_list_markers:
                marker_pose = marker.pose.pose
                for marker in data.markers:
                    if marker.id == marker_id_stor_2:
                        storage_pose = marker.pose.pose

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class ARtagNavigation(object):
    def __init__(self):
        super(ARtagNavigation, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('marker_node', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        xarm7 = moveit_commander.MoveGroupCommander("xarm7")

        gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.xarm7 = xarm7
        self.gripper = gripper

    def Xarm7ToObject(self):
    
        xarm7 = self.xarm7

        pose_goal = geometry_msgs.msg.Pose()
        current_pose = xarm7.get_current_pose().pose
        pose_goal = current_pose

        marker_orientation = (
        marker_pose.orientation.x,
        marker_pose.orientation.y,
        marker_pose.orientation.z,
        marker_pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(marker_orientation)

        roll = 3.14 + euler[0]
        pitch = euler[1]
        yaw = euler[2]

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        pose_goal.position.x = marker_pose.position.x
        pose_goal.position.y = marker_pose.position.y
        pose_goal.position.z = marker_pose.position.z + 0.1

        xarm7.set_pose_target(pose_goal)

        plan_success, traj, planning_time, error_code = xarm7.plan()
        xarm7.clear_pose_targets()
        
        current_pose = self.xarm7.get_current_pose().pose
        return traj, all_close(pose_goal, current_pose, 0.01)

    def Xarm7ToStorage(self):
        xarm7 = self.xarm7

        pose_goal = geometry_msgs.msg.Pose()
        current_pose = xarm7.get_current_pose().pose
        pose_goal = current_pose

        pose_goal.position.z += 0.1
        xarm7.set_pose_target(pose_goal)
        xarm7.go(wait=True)
        xarm7.clear_pose_targets()

        storage_orientation = (
        storage_pose.orientation.x,
        storage_pose.orientation.y,
        storage_pose.orientation.z,
        storage_pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(storage_orientation)

        roll = 3.14 + euler[0]
        pitch = euler[1]
        yaw = euler[2]

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        pose_goal.position.x = storage_pose.position.x + 0.1
        pose_goal.position.y = storage_pose.position.y
        pose_goal.position.z = storage_pose.position.z + 0.1

        xarm7.set_pose_target(pose_goal)

        plan_success, traj, planning_time, error_code = xarm7.plan()
        xarm7.clear_pose_targets()
        
        current_pose = self.xarm7.get_current_pose().pose
        return traj, all_close(pose_goal, current_pose, 0.01)

    def Gripper(self, state):
        gripper = self.gripper
        gripper_values = gripper.get_current_joint_values()
        if state == "open":
            gripper_values[0] = 0
        elif state == "close":
            gripper_values[0] = 0.5
        gripper.go(gripper_values, wait=True)

    def ExecutePlan(self, plan):    
        xarm7 = self.xarm7
        xarm7.execute(plan, wait=True)

        current_pose = xarm7.get_current_pose().pose
        pose_goal = current_pose
        pose_goal.position.z -= 0.1
        xarm7.set_pose_target(pose_goal)
        xarm7.go(wait=True)
        xarm7.clear_pose_targets()

    def Xarm7ToStart(self):
        xarm7 = self.xarm7
        joint_goal = xarm7.get_current_joint_values()
        joint_goal[0] = 0.36
        joint_goal[1] = -0.68
        joint_goal[2] = 0
        joint_goal[3] = 0.83
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 3

        xarm7.go(joint_goal, wait=True)

def main():
    global param, marker_id_stor_1, marker_id_obj_1, marker_id_stor_2, marker_id_obj_2
    try:
        # print ("============ Enter id of storage area 1, storage area 2, object 1, object 2")
        # marker_id = input().split()
        # marker_ids = []
        # for id in marker_id:
        #     marker_ids.append(int(id))
        
        print ("============ Enter id of storage area 1")
        marker_id_stor_1 = int(input())
        print ("============ Enter id of object needs to be placed to the storage area 1")
        marker_ids_obj_1 = input().split()
        marker_id_obj_1 = list(map(int, marker_id_obj_1))
        print ("============ Enter id of storage area 2")
        marker_id_stor_2 = int(input())
        print ("============ Enter id of object needs to be placed to the storage area 2")
        marker_ids_obj_2 = input().split()
        marker_id_obj_2 = list(map(int, marker_id_obj_2))


        print ("============ Press `Enter` to begin by setting up the moveit_commander")
        input()
        param = 1
        move = ARtagNavigation()
        rospy.loginfo("Subscribing to ar_pose_marker")

        while True:

            rospy.Subscriber("/ar_tf_markers", AlvarMarkers, callback)
            print ("============ Press `Enter` to start Xarm7 movement")
            inp = input()
            #param = 1
            plan, bool_debug = move.Xarm7ToObject()
            print ("============ Press `ENTER` to execute a saved path")
            inp = input()
            move.ExecutePlan(plan)
            move.Gripper("close")
            plan, bool_debug = move.Xarm7ToStorage()
            print ("============ Press `ENTER` to execute a saved path")
            inp = input()
            move.ExecutePlan(plan)
            move.Gripper("open")
            move.Xarm7ToStart()
            k = k + 1
            if k == 1: 
                black_list_markers.append(marker_id_obj_1)
            else:
                black_list_markers.append(marker_id_obj_2)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        exit(0)

if __name__ == '__main__':
    main()