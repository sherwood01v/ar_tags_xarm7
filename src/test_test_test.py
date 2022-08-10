#!/usr/bin/python3

import rospy
import tf
import geometry_msgs.msg
import ar_track_alvar_msgs.msg
from tf import TransformListener
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped

def callback(data):
    if len(data.markers)!=0:
        for marker in data.markers:
            marker.pose.header.frame_id = "ar_marker_" + str(marker.id)
            rospy.loginfo(marker)
            p_in_cam = listener.transformPose("usb_cam", marker.pose)
            rospy.loginfo(p_in_cam)
            pub.publish(p_in_cam)

def main():
    global listener, pub, pub1, pub_markers
    rospy.init_node('transform_pose_marker', anonymous=True)
    listener = tf.TransformListener()
    pub = rospy.Publisher('test_publisher', PoseStamped, queue_size=1)
    #pub1 = rospy.Publisher('pose_link_base', PoseStamped, queue_size=1)
    #pub_markers = rospy.Publisher('ar_tf_markers', AlvarMarkers, queue_size=1)
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    rospy.spin()
                    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass