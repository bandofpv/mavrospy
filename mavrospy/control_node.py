#!/usr/bin/env python3

import math
import numpy as np
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandHome
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class MavrospyController(Node):
    """
    Controller class to help interface with MAVROS in ROS2
    """
    def __init__(self, frequency):
        super().__init__('mavrospy_control_node')

        # Declare and retrieve parameters
        self.declare_parameter('vision', False)
        self.vision = self.get_parameter('vision').value

        # Create subscribers
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.create_subscription(ExtendedState, '/mavros/extended_state', self.extended_state_callback, 10)
        self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin', self.origin_callback, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)

        # Create publishers
        self.cmd_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.origin_pub = self.create_publisher(GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)

        # Create service clients
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_home_client = self.create_client(CommandHome, '/mavros/cmd/set_home')

        # ROS messages
        self.pose = Pose()
        self.current_state = State()
        self.current_extended_state = ExtendedState()
        self.current_gps = NavSatFix()
        self.current_gps.status.status = NavSatStatus.STATUS_NO_FIX
        self.timestamp = self.get_clock().now()

        # Initialize constants
        self.pi_2 = math.pi / 2.0
        self.freq = frequency
        self.rate = self.create_rate(frequency)
        self.target_lat = 38.9853504
        self.target_lon = -76.4857648
        self.target_alt = 36.810

        # Status variables
        self.origin_set = False

        # Vision initialization
        if self.vision:
            self.set_global_origin()
            self.wait_for_gps_fix()
            self.set_home_position()

        self.get_logger().info("MavrospyController Initiated")

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.timestamp = msg.header.stamp
        self.pose = msg.pose

    def extended_state_callback(self, msg):
        self.current_extended_state = msg

    def origin_callback(self, msg):
        if (msg.position.latitude == self.target_lat and
                msg.position.longitude == self.target_lon):
            self.origin_set = True
        else:
            self.get_logger().warn(f"Global origin mismatch: Expected "
                                   f"({self.target_lat}, {self.target_lon}) but got "
                                   f"({msg.position.latitude}, {msg.position.longitude})")

    def gps_callback(self, msg):
        self.current_gps = msg

    def set_global_origin(self):
        self.get_logger().info("Setting global origin...")
        while rclpy.ok():
            origin_msg = GeoPointStamped()
            origin_msg.header.stamp = self.get_clock().now().to_msg()
            origin_msg.header.frame_id = "map"
            origin_msg.position.latitude = self.target_lat
            origin_msg.position.longitude = self.target_lon
            origin_msg.position.altitude = self.target_alt
            self.origin_pub.publish(origin_msg)
            if self.origin_set:
                self.get_logger().info("Global origin set.")
                break
            self.rate.sleep()

    def wait_for_gps_fix(self):
        self.get_logger().info("Waiting for GPS fix...")
        while rclpy.ok():
            if self.current_gps.status.status == NavSatStatus.STATUS_FIX:
                self.get_logger().info("GPS fix acquired.")
                break
            self.rate.sleep()

    def set_home_position(self):
        self.get_logger().info("Setting home position...")
        req = CommandHome.Request()
        req.current_gps = True
        if self.set_home_client.call(req).success:
            self.get_logger().info("Home position set.")
        else:
            self.get_logger().error("Failed to set home position.")

    def arm(self, status):
        req = CommandBool.Request()
        req.value = status
        if self.arm_client.call(req).success:
            self.get_logger().info(f"{'Armed' if status else 'Disarmed'} throttle")
        else:
            self.get_logger().error(f"Failed to {'arm' if status else 'disarm'} throttle")

# def main():
#     rclpy.init()
#     node = MavrospyController(10)
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
# 
# if __name__ == '__main__':
#     main()
# 
