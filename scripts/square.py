#! /usr/bin/env python3

import tf
import rospy
from control_node import MavrospyController


def fly_square(c, width, altitude):
    """
    Fly in a square pattern facing only in the forward direction
    """
    c.log_info("Waypoint 1")
    c.goto_xyz_rpy(width, 0.0, altitude, 0, 0, 0)
    c.log_info("Waypoint 2")
    c.goto_xyz_rpy(width, width, altitude, 0, 0, 0)
    c.log_info("Waypoint 3")
    c.goto_xyz_rpy(0.0, width, altitude, 0, 0, 0)
    c.log_info("Waypoint 4")
    c.goto_xyz_rpy(0.0, 0.0, altitude, 0, 0, 0)
    c.log_info("Square Pattern Complete")


def move():
    """
    Move UAV in a square pattern at given height and width for given
    repititions and altitude levels
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    c = MavrospyController(rate)  # create mavrospy controller instance

    min_height = 1.0  # min height to fly at
    max_height = 3.0  # max height to fly at
    width = 3.0  # width of the square pattern
    levels = 3  # number of different altitudes to complete square pattern
    repetitions = 3   # number of times to repeat the square pattern at each altitude

    # Create list of different altitudes to fly from min to max height and number of levels
    altitudes = [min_height + (max_height - min_height) * l / (levels-1) for l in range(levels)]

    # Wait until drone is in OFFBOARD mode
    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        # Before entering OFFBOARD mode, you must have already started streaming setpoints
        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 1, False, False)

    # Takeoff at lowest altitude in altitudes list
    c.log_info(f"Takeoff: {altitudes[0]}ft")
    c.takeoff(altitudes[0])

    # Fly in a square pattern at all altitudes
    for altitude in altitudes:
        c.log_info(f"Pattern Altitude: {altitude}ft")
        c.goto_xyz_rpy(0.0, 0.0, altitude, 0, 0, 0)  # reset to origin
        for r in range(repetitions):  # repeat square pattern
            fly_square(c, width, altitude)

    # Land
    c.log_info("Landing")
    c.land()


if __name__ == "__main__":
    move()
