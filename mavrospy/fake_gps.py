#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTFNode(Node):
    """
    ROS2 Node to convert PoseStamped messages to TransformStamped and publish them.
    """
    def __init__(self):
        super().__init__('pose_to_tf_node')

        # Create a TransformBroadcaster to publish TransformStamped messages
        self.tf_broadcaster = TransformBroadcaster(self)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber to /mavros/vision_pose/pose
        self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.pose_callback,
            qos_profile
        )

        self.get_logger().info("pose_to_tf_node initialized and running")

    def pose_callback(self, pose_msg):
        """
        Callback function for the mocap pose subscriber.

        Converts the mocap pose to a TransformStamped message and publishes it.
        """
        # Create a TransformStamped message
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Adjust to your coordinate frame setup
        t.child_frame_id = "fix"  # Frame of the vehicle

        # Copy position data
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z

        # Copy orientation data
        t.transform.rotation.x = pose_msg.pose.orientation.x
        t.transform.rotation.y = pose_msg.pose.orientation.y
        t.transform.rotation.z = pose_msg.pose.orientation.z
        t.transform.rotation.w = pose_msg.pose.orientation.w

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = PoseToTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down pose_to_tf_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

