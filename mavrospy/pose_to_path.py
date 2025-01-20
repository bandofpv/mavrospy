#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PoseToPathNode(Node):
    """
    Node to convert PoseStamped topics into Path for visualization.
    """
    def __init__(self):
        super().__init__('pose_to_path')

        # Get the topic name from parameters (default: /mavros/local_position/pose)
        self.declare_parameter('topic', '/mavros/local_position/pose')
        topic = self.get_parameter('topic').value

        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = "map"

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber to PoseStamped
        self.pose_sub = self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            qos_profile
        )

        # Publisher for the Path
        self.path_pub = self.create_publisher(Path, '/path', qos_profile)

        self.get_logger().info(f"Subscribed to topic: {topic}")
        self.get_logger().info("Publishing Path messages on /path")

    def pose_callback(self, msg: PoseStamped):
        """
        Callback function to process PoseStamped messages and update the Path.
        """
        # Update header timestamp
        self.path.header.stamp = self.get_clock().now().to_msg()

        # Append the new pose to the path
        self.path.poses.append(msg)

        # Publish the updated path
        self.path_pub.publish(self.path)

def main():
    rclpy.init()
    node = PoseToPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down pose_to_path node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
