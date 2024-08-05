#! /usr/bin/env python3

import tf
import rospy
from control_node import MavrospyController

def fly_square(c, width, height, repetitions, altitude):
    for r in range(repetitions):
        c.log_info("Waypoint 1: position control")
        c.goto_xyz_rpy(0.0, 0.0, altitude, 0, 0, -1 * c.pi_2)
        c.goto_xyz_rpy(width, 0.0, altitude, 0, 0, -1 * c.pi_2)
        c.log_info("Waypoint 2: position control")
        c.goto_xyz_rpy(width, 0.0, altitude, 0, 0, 0)
        c.goto_xyz_rpy(width, width, altitude, 0, 0, 0)
        c.log_info("Waypoint 3: position control")
        c.goto_xyz_rpy(width, width, altitude, 0, 0, c.pi_2)
        c.goto_xyz_rpy(0.0, width, altitude, 0, 0, c.pi_2)
        c.log_info("Waypoint 4: position control")
        c.goto_xyz_rpy(0.0, width, altitude, 0, 0, 2 * c.pi_2)
        c.goto_xyz_rpy(0.0, 0.0, altitude, 0, 0, 2 * c.pi_2)


def normal_square():
    """
    Move UAV in a square pattern facing only in the forward direction
    """
    rate = 20
    c = MavrospyController(rate)

    max_height = 6.0  # Max height to fly at
    max_width = 10.0  # Width of the square pattern
    repetitions = 3   # Number of times to repeat the square pattern at each altitude

    altitudes = [max_height / 3, max_height * 2 / 3, max_height]

    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        # Before entering OFFBOARD mode, you must have already started streaming setpoints
        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 0, False, False)

    c.log_info(f"Takeoff: {altitudes[0]}m")
    c.takeoff(altitudes[0])

    for altitude in altitudes:
        fly_square(c, max_width, max_height, repetitions, altitude)

    c.log_info("Landing")
    c.land()

def old_normal_square():
    """
    Move UAV in a square pattern facing only in the forward direction
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20

    # Initialize MAVROSPY controller
    c = MavrospyController(rate)
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

