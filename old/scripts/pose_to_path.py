#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PoseToPathNode:
    """
    Node to convert PoseStamed topics into Path for visualization
    """
    def __init__(self):
        rospy.init_node('pose_to_path')

        # Get the topic name from the parameter server
        topic = rospy.get_param('~topic', '/mavros/local_position/pose')

        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = "map"

        # Subscriber to PoseStamped
        self.pose_sub = rospy.Subscriber(topic, PoseStamped, self.pose_callback)

        # Publisher for the Path
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)

    def pose_callback(self, msg):
        # Update header timestamp
        self.path.header.stamp = rospy.Time.now()

        # Append the new pose to the path
        self.path.poses.append(msg)

        # Publish the updated path
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    node = PoseToPathNode()
    rospy.spin()
