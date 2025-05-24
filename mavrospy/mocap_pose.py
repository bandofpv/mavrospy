#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from motion_capture_tracking_interfaces.msg import NamedPoseArray
from geometry_msgs.msg import PoseStamped


class MocapPoseConverterNode(Node):

    def __init__(self):
        super().__init__('mocap_pose_converter_node')

        # Subscriber to NamedPoseArray
        self.subscription = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.listener_callback,
            10
        )

        # Publisher for PoseStamped
        self.publisher = self.create_publisher(PoseStamped, '/quad_pose', 10)
        self.get_logger().info('MoacapPoseConverterNode has been started.')

    def listener_callback(self, msg: NamedPoseArray):
        if not msg.poses:
            self.get_logger().warn('Received NamedPoseArray with no poses.')
            return

        named_pose = msg.poses[0]

        pose_stamped = PoseStamped()
        pose_stamped.header = named_pose.header
        pose_stamped.pose = named_pose.pose

        self.publisher.publish(pose_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = MocapPoseConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
