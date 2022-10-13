import rospy
import sys
import moveit_commander
import geometry_msgs.msg
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from moveit_commander.conversions import pose_to_list

def callback(data):
    global marker_pose, storage_pose, marker_id_stor_1, marker_id_obj_1, marker_id_stor_2, marker_id_obj_2
    try:
        for marker in data.markers:
            if marker.id == marker_id_stor_1:
                storage_pose_1 = marker.pose.pose
            elif marker.id == marker_id_stor_2:
                storage_pose_2 = marker.pose.pose

        for marker in data.markers:
            if marker.id == marker_id_obj_1:
                marker_position = marker.pose.pose
                if marker_position.position.x < storage_pose_1.position.x:
                    marker_pose = marker_position
                    storage_pose = storage_pose_1
            elif marker.id == marker_id_obj_2:
                marker_position = marker.pose.pose
                if marker_position.position.x < storage_pose_2.position.x:
                    marker_pose = marker_position
                    storage_pose = storage_pose_2
    except: return
 

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

        rospy.sleep(2)

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        p.pose.position.x = -0.22
        p.pose.position.y = -0.13
        p.pose.position.z = -0.05
        p.pose.orientation.x = 0.
        p.pose.orientation.y = 0.
        p.pose.orientation.z = 0.2570806
        p.pose.orientation.w = 0.96639

        scene.add_box("table_1", p, (1, 1, 0.1))

        p.pose.position.x = 0.46
        p.pose.position.y = 0.26
        p.pose.position.z = -0.14
        p.pose.orientation.x = 0.
        p.pose.orientation.z = 0.2570806
        p.pose.orientation.w = 0.96639

        scene.add_box("table_2", p, (1, 1, 0.1))

        p.pose.position.x = -0.37
        p.pose.position.y = -0.20
        p.pose.position.z = 0.4
        p.pose.orientation.x = -0.1848861
        p.pose.orientation.y = 0.6863426
        p.pose.orientation.z = 0.1848861
        p.pose.orientation.w = 0.6786516

        scene.add_box("wall", p, (1, 1, 0.1))

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
        pose_goal.position.z = marker_pose.position.z+0.05

        xarm7.set_pose_target(pose_goal)

        plan_success, traj, planning_time, error_code = xarm7.plan()
        xarm7.clear_pose_targets()
        
        current_pose = self.xarm7.get_current_pose().pose
        return traj, all_close(pose_goal, current_pose, 0.01)

    def Xarm7ToStorage(self):
        xarm7 = self.xarm7

        # joint_goal = xarm7.get_current_joint_values()
        # joint_goal[1] = 0.1

        # xarm7.go(joint_goal, wait=True)

        pose_goal = geometry_msgs.msg.Pose()
        current_pose = xarm7.get_current_pose().pose
        pose_goal = current_pose

        # storage_orientation = (
        # storage_pose.orientation.x,
        # storage_pose.orientation.y,
        # storage_pose.orientation.z,
        # storage_pose.orientation.w)

        # euler = tf.transformations.euler_from_quaternion(storage_orientation)

        # roll = 3.14 + euler[0]
        # pitch = euler[1]
        # yaw = euler[2]

        # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        # pose_goal.orientation.x = quaternion[0]
        # pose_goal.orientation.y = quaternion[1]
        # pose_goal.orientation.z = quaternion[2]
        # pose_goal.orientation.w = quaternion[3]

        pose_goal.position.x = storage_pose.position.x
        pose_goal.position.y = storage_pose.position.y
        pose_goal.position.z = storage_pose.position.z + 0.05

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
        joint_goal[6] = 0

        xarm7.go(joint_goal, wait=True)

def main():
    global marker_pose, marker_id_stor_1, marker_id_obj_1, marker_id_stor_2, marker_id_obj_2
    try:
        # print ("============ Enter id of storage area 1, storage area 2, object 1, object 2")
        # marker_id = input().split()
        # marker_ids = []
        # for id in marker_id:
        #     marker_ids.append(int(id))
        
        # print ("============ Enter id of storage area 1")
        # marker_id_stor_1 = int(input())
        # print ("============ Enter id of object needs to be placed to the storage area 1")
        # marker_id_obj_1 = int(input())
        # print ("============ Enter id of storage area 2")
        # marker_id_stor_2 = int(input())
        # print ("============ Enter id of object needs to be placed to the storage area 2")
        # marker_id_obj_2 = int(input())
        # marker_id_stor_1 = 7
        # marker_id_obj_1 = 8
        # marker_id_stor_2 = 18
        # marker_id_obj_2 = 6
        marker_id_stor_1 = 6
        marker_id_obj_1 = 0
        marker_id_stor_2 = 12
        marker_id_obj_2 = 9

        param = 1
        move = ARtagNavigation()
        rospy.loginfo("Subscribing to ar_pose_marker")
        rospy.Subscriber("/ar_tf_markers", AlvarMarkers, callback)

        move.Xarm7ToStart()

        while True:
            
            #rospy.Subscriber("/ar_tf_markers", AlvarMarkers, callback)

            try:
                print ("============ Press `Enter` to start Xarm7 movement")
                inp = input()
                if inp == "stop": break
                else: inp = "plan"

                while inp == "plan":
                    plan, bool_debug = move.Xarm7ToObject() 

                    print ("============ Press `ENTER` to execute a saved path, plan to replan")
                    inp = input()
                    
                if inp == "stop": break
                elif inp == "restart": main()
                
                move.ExecutePlan(plan)
                move.Gripper("close")
                inp = "plan"

                while inp == "plan":
                    plan, bool_debug = move.Xarm7ToStorage()

                    print ("============ Press `ENTER` to execute a saved path, plan to replan")
                    inp = input()
                
                if inp == "stop": break
                elif inp == "restart": main()

                move.ExecutePlan(plan)
                move.Gripper("open")
                marker_pose = 0
                move.Xarm7ToStart()
            except:
                print("No marker detected")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        exit(0)

if __name__ == '__main__':
    main()