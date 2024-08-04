#! /usr/bin/env python3

import tf
import rospy
from control_node import MavController


def normal_square():
    """
    Move UAV in a square pattern facing only in the forward direction
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20

    # Initialize MAVROSPY controller
    c = MavController(rate)
    alt = 3.0

    # Start when in OFFBOARD mode
    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        # Before entering OFFBOARD mode, you must have already started streaming setpoints
        c.goto_xyz_rpy(c.pose.position.x, c.pose.position.y, c.pose.position.z,
            roll, pitch, yaw, 0, False, False)

    # Takeoff
    c.log_info(f"Takeoff: {alt}m")
    c.takeoff(alt, 8)

    c.log_info("Waypoint 1: position control")
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, -1 * c.pi_2, 2)
    c.goto_xyz_rpy(10.0, 0.0, alt, 0, 0, -1 * c.pi_2, 6)
    c.log_info("Waypoint 2: position control")
    c.goto_xyz_rpy(10.0, 0.0, alt, 0, 0, 0, 2)
    c.goto_xyz_rpy(10.0, 10.0, alt, 0, 0, 0, 6)
    c.log_info("Waypoint 3: position control")
    c.goto_xyz_rpy(10.0, 10.0, alt, 0, 0, c.pi_2, 2)
    c.goto_xyz_rpy(0.0, 10.0, alt, 0, 0, c.pi_2, 6)
    c.log_info("Waypoint 4: position control")
    c.goto_xyz_rpy(0.0, 10.0, alt, 0, 0, 2 * c.pi_2, 2)
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 2 * c.pi_2, 6)

    c.log_info("Landing")
    c.land()


if __name__ == "__main__":
    normal_square()

