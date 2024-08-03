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
    alt = 2.0

    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        # c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 0, False)

    # c.set_home_position()

    # TODO: Test if this will arm rather than looped goto
    c.goto_xyz_rpy(0,0, 0, 0, 0, 0, 1)

    c.log_info(f"Takeoff: {alt}m")
    c.takeoff(alt, 8)

    c.log_info("Waypoint 1: position control")
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, -1 * c.pi_2, 2)
    c.goto_xyz_rpy(2.0, 0.0, alt, 0, 0, -1 * c.pi_2, 3)
    c.log_info("Waypoint 2: position control")
    c.goto_xyz_rpy(2.0, 0.0, alt, 0, 0, 0, 2)
    c.goto_xyz_rpy(2.0, 2.0, alt, 0, 0, 0, 3)
    c.log_info("Waypoint 3: position control")
    c.goto_xyz_rpy(2.0, 2.0, alt, 0, 0, c.pi_2, 2)
    c.goto_xyz_rpy(0.0, 2.0, alt, 0, 0, c.pi_2, 3)
    c.log_info("Waypoint 4: position control")
    c.goto_xyz_rpy(0.0, 2.0, alt, 0, 0, 2 * c.pi_2, 2)
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 2 * c.pi_2, 3)

    c.log_info("Landing")
    # c.land()

    c.goto_xyz_rpy(0, 0, 0, 0, 0, 2 * c.pi_2, 5)
    c.arm(False)

if __name__ == "__main__":
    simple_demo()

