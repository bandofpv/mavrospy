#! /usr/bin/env python

import math
import rospy
from control_node import MavController


def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    rate = 14

    c = MavController(rate)

    alt = 2.0

    rospy.loginfo("Takeoff " + str(alt))
    c.takeoff(alt, 5)
    c.goto_xyz_rpy(0, 0, alt, 0, 0, 0, 3)

    r = 0.3
    c.goto_xyz_rpy(0.2, 0.3, alt, 0, 0, 0, 3)
    c.goto_xyz_rpy(0.2 + r, 0.3, alt, 0, 0, 0, 3)

    for i in range(180):
        theta = i * 2.0 * math.pi / 180.0
        x = 0.2 + r * math.cos(theta)
        y = 0.3 + r * math.sin(theta)
        z = alt
        c.goto_xyz_rpy(x, y, z, 0.0, 0.0, theta, 0, False)

    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 0, 3)

    rospy.loginfo("Landing")
    c.land()


if __name__ == "__main__":
    simple_demo()
