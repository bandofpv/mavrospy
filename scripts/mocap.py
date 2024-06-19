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

    # while True:
        # print(c.pose)

    while not rospy.is_shutdown():
        c.mocap()

if __name__ == "__main__":
    simple_demo()

