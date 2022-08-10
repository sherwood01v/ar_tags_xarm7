#!/usr/bin/python3

import rospy
import tf
import numpy
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import atan2, sqrt

def callback(data):
        global last_heartbeat, marker_ids, k, param, twist_msg, cmd_vel_pub, finish, distance_achieved
        if len(data.markers)!=0:
            for marker in data.markers:
                #print(marker)
                marker.pose.header = marker.header
                pub.publish(marker.pose)
                #rospy.loginfo(rospy.get_caller_id() + " I heard %s", marker)

def ar_demo():
        global marker_ids, pub
        # Initialize this ROS node
        rospy.init_node('test_transform_tags', anonymous=True)
        # get target marker id
        #marker_ids = rospy.get_param('~marker_ids').split(",")

        # Create publisher for command velocity
        #global cmd_vel_pub
        #cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        pub = rospy.Publisher('pose', PoseStamped, queue_size=1)

        # Set up subscriber for /ar_pose_marker
        rospy.loginfo("Subscribing to ar_pose_marker")
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)

        rospy.spin()

if __name__ == '__main__':
    try:
        ar_demo()
    except rospy.ROSInterruptException:
        pass