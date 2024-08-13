#! /usr/bin/env python3

import tf
import rospy
from control_node import MavrospyController

from geographic_msgs.msg import GeoPoint, GeoPointStamped

def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20

    c = MavrospyController(rate)
    alt = 2.0

    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 1, False, False)

    c.log_info(f"Takeoff: {alt}ft")
    c.takeoff(alt)

    c.log_info("Waypoint 1: position control")
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, -1 * c.pi_2)
    c.goto_xyz_rpy(2.0, 0.0, alt, 0, 0, -1 * c.pi_2)
    c.log_info("Waypoint 2: position control")
    c.goto_xyz_rpy(2.0, 0.0, alt, 0, 0, 0)
    c.goto_xyz_rpy(2.0, 2.0, alt, 0, 0, 0)
    c.log_info("Waypoint 3: position control")
    c.goto_xyz_rpy(2.0, 2.0, alt, 0, 0, c.pi_2)
    c.goto_xyz_rpy(0.0, 2.0, alt, 0, 0, c.pi_2)
    c.log_info("Waypoint 4: position control")
    c.goto_xyz_rpy(0.0, 2.0, alt, 0, 0, 2 * c.pi_2)
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 2 * c.pi_2)

    c.log_info("Landing")
    c.land()

if __name__ == "__main__":
    simple_demo()
