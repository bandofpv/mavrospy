#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

def pose_callback(pose_msg):
    """
    Callback function for the mocap pose subscriber

    Converts the mocap pose to a TransformStamped message and publishes it to the /mavros/fake_gps/mocap/tf topic
    """
    # Create a TransformStamped message
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"  # or "world" depending on your setup
    t.child_frame_id = "fix"  # frame of your vehicle

    # Copy the position
    t.transform.translation.x = pose_msg.pose.position.x
    t.transform.translation.y = pose_msg.pose.position.y
    t.transform.translation.z = pose_msg.pose.position.z

    # Copy the orientation
    t.transform.rotation.x = pose_msg.pose.orientation.x
    t.transform.rotation.y = pose_msg.pose.orientation.y
    t.transform.rotation.z = pose_msg.pose.orientation.z
    t.transform.rotation.w = pose_msg.pose.orientation.w

    # Publish the transform to the /mavros/fake_gps/mocap/tf topic
    tf_pub.publish(t)

def main():
    rospy.init_node('pose_to_tf_node')

    # Subscribe to the /mavros/vision_pose/pose topic
    rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, pose_callback)

    # Publisher to the /mavros/fake_gps/mocap/tf topic
    tf_pub = rospy.Publisher('/mavros/fake_gps/mocap/tf', TransformStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()

