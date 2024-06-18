#! /usr/bin/env python3

import math
import rospy
from control_node import MavController


def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    rate = 14

    c = MavController(rate)

    alt = 3.0

    rospy.loginfo("Takeoff " + str(alt))
    c.takeoff(alt, 8)
    c.goto_xyz_rpy(0, 0, alt, 0, 0, 0, 3)

    r = 3.0
    rospy.loginfo("Moving to center")
    c.goto_xyz_rpy(2.0, 3.0, alt, 0, 0, 0, 3)
    rospy.loginfo("Moving to perimeter")
    c.goto_xyz_rpy(2.0 + r, 3.0, alt, 0, 0, 0, 3)

    rospy.loginfo("Starting orbit")
    for i in range(180):
        theta = i * 2.0 * math.pi / 180.0
        x = 2.0 + r * math.cos(theta)
        y = 3.0 + r * math.sin(theta)
        z = alt
        c.goto_xyz_rpy(x, y, z, 0.0, 0.0, theta, 0, False)

    rospy.loginfo("Moving back home")
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 0, 3)

    rospy.loginfo("Landing")
    c.land()


if __name__ == "__main__":
    simple_demo()
