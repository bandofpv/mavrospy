#! /usr/bin/env python3

import tf
import rospy
from control_node import MavrospyController

def fly_square(c, width, repetitions, altitude):
    """
    Fly in a square pattern facing in direction of motion
    """
    for r in range(repetitions):
        c.log_info("Waypoint 1")
        c.goto_xyz_rpy(0.0, 0.0, altitude, 0, 0, -1 * c.pi_2)
        c.goto_xyz_rpy(width, 0.0, altitude, 0, 0, -1 * c.pi_2)
        c.log_info("Waypoint 2")
        c.goto_xyz_rpy(width, 0.0, altitude, 0, 0, 0)
        c.goto_xyz_rpy(width, width, altitude, 0, 0, 0)
        c.log_info("Waypoint 3")
        c.goto_xyz_rpy(width, width, altitude, 0, 0, c.pi_2)
        c.goto_xyz_rpy(0.0, width, altitude, 0, 0, c.pi_2)
        c.log_info("Waypoint 4")
        c.goto_xyz_rpy(0.0, width, altitude, 0, 0, 2 * c.pi_2)
        c.goto_xyz_rpy(0.0, 0.0, altitude, 0, 0, 2 * c.pi_2)
        c.log_info("Square Pattern Complete")

def move():
    """
    Move UAV in a square pattern at given height and width for given
    repititions and altitude levels
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    c = MavrospyController(rate)  # create mavrospy controller instance

    max_height = 6.0  # max height to fly at
    max_width = 10.0  # width of the square pattern
    levels = 3  # number of different altitude to complete square pattern
    repetitions = 3   # number of times to repeat the square pattern at each altitude

    altitudes = []

    # Create list of different altitudes to fly at given max height and number of levels
    for l in range(1, levels + 1):
        altitudes.append((max_height * l) / levels)

    # Wait until drone is in OFFBOARD mode
    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        # Before entering OFFBOARD mode, you must have already started streaming setpoints
        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 1, False, False)

    # Takeoff at lowest altitude in altitudes list
    c.log_info(f"Takeoff: {altitudes[0]}m")
    c.takeoff(altitudes[0])

    # Fly in a square pattern at all altitudes
    for altitude in altitudes:
        fly_square(c, max_width, repetitions, altitude)

    # Land
    c.log_info("Landing")
    c.land()


if __name__ == "__main__":
    move()
