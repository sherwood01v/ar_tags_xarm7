import rospy
import sys
import moveit_commander
import geometry_msgs.msg
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
        p.pose.position.x = 0.01
        p.pose.position.y = 0.00
        p.pose.position.z = -0.07

        scene.add_box("table", p, (1, 1, 0.1))

        p.pose.position.x = -0.44
        p.pose.position.y = 0.00
        p.pose.position.z = 0.48
        p.pose.orientation.y = 0.7068252
        p.pose.orientation.w = 0.7073883

        scene.add_box("wall", p, (1, 1, 0.1))

    def Xarm7ToObject(self):
        xarm7 = self.xarm7

        pose_goal = geometry_msgs.msg.Pose()
        current_pose = xarm7.get_current_pose().pose
        pose_goal = current_pose

        pose_goal.position.x = marker_pose.position.x
        pose_goal.position.y = marker_pose.position.y - 0.09687462526

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

        pose_goal.position.x = storage_pose.position.x + 0.15
        pose_goal.position.y = storage_pose.position.y + 0.05

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
            gripper_values[0] = 0.3
        gripper.go(gripper_values, wait=True)
    
    def lineMotion(self, direction):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose

        waypoints = []
        if direction == "down": 
            current_pose.position.z = 0.03
        elif direction == "up": 
            current_pose.position.z = 0.3

        waypoints.append(current_pose)
        (traj, fraction) = xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)

        xarm7.clear_pose_targets()

        self.ExecutePlan(traj)

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
        marker_id_stor_1 = 6
        marker_id_obj_1 = 2
        marker_id_stor_2 = 12
        marker_id_obj_2 = 0

        param = 1
        move = ARtagNavigation()
        rospy.loginfo("Subscribing to ar_pose_marker")
        rospy.Subscriber("/ar_tf_markers", AlvarMarkers, callback)

        while True:
            try:
                print ("============ Press `Enter` to start Xarm7 movement")
                inp = input()
                if inp == "stop": break
                else: inp = "plan"
                move.Xarm7ToStart()
                plan, bool_debug = move.Xarm7ToObject()  
                move.ExecutePlan(plan)
                move.lineMotion("down")
                move.Gripper("close")
                move.lineMotion("up")
                plan, bool_debug = move.Xarm7ToStorage()
                move.ExecutePlan(plan)
                move.lineMotion("down")
                move.Gripper("open")
                move.lineMotion("up")
                move.Xarm7ToStart()
                marker_pose = 0
            except:
                print("No marker detected")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        exit(0)

if __name__ == '__main__':
    main()
