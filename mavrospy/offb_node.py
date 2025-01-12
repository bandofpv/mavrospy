"""
 * File: offb_node.py
 * Stack and tested in ROS2 (rclpy)
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import asyncio
from threading import Thread

class OffboardNode(Node):
    def __init__(self):
        super().__init__("offb_node_py")

        self.current_state = State()

        # Subscribers
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_cb, 10)

        # Publishers
        self.local_pos_pub = self.create_publisher(PoseStamped, "mavros/setpoint_position/local", 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, "mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "mavros/set_mode")

        # Wait for services to be available
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /mavros/cmd/arming service...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /mavros/set_mode service...")

        # Initial pose
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 10.0

        # State variables
        self.last_req = self.get_clock().now()
        self.offboard_set_mode = SetMode.Request()
        self.offboard_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd = CommandBool.Request()
        self.arm_cmd.value = True

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # Background asyncio loop
        self.loop = asyncio.new_event_loop()
        self.thread = Thread(target=self.run_asyncio_loop, daemon=True)
        self.thread.start()

    def run_asyncio_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def state_cb(self, msg):
        self.current_state = msg

    async def set_offboard_mode(self):
        if self.current_state.mode != "OFFBOARD" and (self.get_clock().now() - self.last_req).nanoseconds > 5e9:
            future = self.set_mode_client.call_async(self.offboard_set_mode)
            response = await future
            if response and response.mode_sent:
                self.get_logger().info("OFFBOARD enabled")
            self.last_req = self.get_clock().now()

    async def arm_vehicle(self):
        if not self.current_state.armed and (self.get_clock().now() - self.last_req).nanoseconds > 5e9:
            future = self.arming_client.call_async(self.arm_cmd)
            response = await future
            if response and response.success:
                self.get_logger().info("Vehicle armed")
            self.last_req = self.get_clock().now()

    def timer_callback(self):
        # Publish initial pose to establish connection
        self.local_pos_pub.publish(self.pose)

        # Check and set modes
        self.loop.call_soon_threadsafe(asyncio.create_task, self.set_offboard_mode())
        self.loop.call_soon_threadsafe(asyncio.create_task, self.arm_vehicle())

    def destroy_node(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = OffboardNode()

    try:
        # Ensure at least a few setpoints are sent before entering offboard mode
        for _ in range(100):
            node.local_pos_pub.publish(node.pose)
            time.sleep(0.05)  # 20Hz

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

