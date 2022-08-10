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
        tf_markers = ar_track_alvar_msgs.msg.AlvarMarkers()
        #tf_markers = data
        #tf_markers.markers = []
        #rospy.loginfo(data)
        for marker in data.markers:
            #rospy.loginfo("I exist")
            tf_marker = marker
            marker.pose.header = marker.header
            pub.publish(marker.pose)
            t = listener.getLatestCommonTime("/link_base", "/usb_cam")
            (trans,rot) = listener.lookupTransform('/link_base', '/usb_cam', t)
            p_in_base = listener.transformPose("link_base", marker.pose)
            pub1.publish(p_in_base)
            tf_marker.pose = p_in_base
            tf_marker.header.frame_id = "link_base"
            tf_markers.markers.append(tf_marker)
        tf_markers.header = data.header
        tf_markers.header.frame_id = "link_base"
        pub_markers.publish(tf_markers)

def main():
    global listener, pub, pub1, pub_markers
    rospy.init_node('transform_pose_marker', anonymous=True)
    listener = tf.TransformListener()
    pub = rospy.Publisher('pose_usb_cam', PoseStamped, queue_size=1)
    pub1 = rospy.Publisher('pose_link_base', PoseStamped, queue_size=1)
    pub_markers = rospy.Publisher('ar_tf_markers', AlvarMarkers, queue_size=1)
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    rospy.spin()
                    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
