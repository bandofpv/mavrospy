#! /usr/bin/env python

import rospy
from control_node import MavController


def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20

    c = MavController(rate)
    # rospy.sleep(1)
    alt = 5.0

    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            rospy.loginfo("OFFBOARD enabled")
            break

        c.goto_xyz_rpy(0.0, 0.0, 0.0, 0, 0, 0, 1/rate)

    rospy.loginfo(f"Takeoff: {alt}m")
    c.takeoff(alt, 5)

    rospy.loginfo("Waypoint 1: position control")
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, -1 * c.pi_2, 2)
    c.goto_xyz_rpy(10.0, 0.0, alt, 0, 0, -1 * c.pi_2, 6)
    rospy.loginfo("Waypoint 2: position control")
    c.goto_xyz_rpy(10.0, 0.0, alt, 0, 0, 0, 2)
    c.goto_xyz_rpy(10.0, 10.0, alt, 0, 0, 0, 6)
    rospy.loginfo("Waypoint 3: position control")
    c.goto_xyz_rpy(10.0, 10.0, alt, 0, 0, c.pi_2, 2)
    c.goto_xyz_rpy(0.0, 10.0, alt, 0, 0, c.pi_2, 6)
    rospy.loginfo("Waypoint 4: position control")
    c.goto_xyz_rpy(0.0, 10.0, alt, 0, 0, 2 * c.pi_2, 2)
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 2 * c.pi_2, 6)
    # c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 3 * c.pi_2, 1)
    # c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 4 * c.pi_2, 2)

    rospy.loginfo("Landing")
    c.land()


if __name__ == "__main__":
    simple_demo()

