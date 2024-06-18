#! /usr/bin/env python3

import tf
import rospy
from control_node import MavController


def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20

    c = MavController(rate)
    alt = 3.0

    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            rospy.loginfo("OFFBOARD enabled")
            break

        explicit_quat = [c.pose.orientation.x, c.pose.orientation.y, c.pose.orientation.z, c.pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
        c.goto_xyz_rpy(c.pose.position.x, c.pose.position.y, c.pose.position.z, roll, pitch, yaw, 0, False)

    rospy.loginfo(f"Takeoff: {alt}m")
    c.takeoff(alt, 8)

    c.set_vel(1, 0, 0, 0, 0, 0, 2)
    c.set_vel(0, 0, 0, 0, 0, 0, 2)

    rospy.loginfo("Landing")
    c.land()


if __name__ == "__main__":
    simple_demo()

