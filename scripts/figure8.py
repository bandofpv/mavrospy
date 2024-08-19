#!/usr/bin/env python3

import rospy
import math
from control_node import MavrospyController


def fly_figure_eight(c, width, length, altitude, resolution=360):
    """
    Fly in a figure-eight pattern facing only in forward direction
    """
    # Generate figure-eight points
    for i in range(resolution):
        theta = 2 * math.pi * i / resolution  # increment angle around unit circle
        x = width/2 * math.cos(theta)  # calculate x position
        y = length/2 * math.sin(2 * theta)  # calculate y position

        # Account for center offset
        x += width/2
        y += length/2

        c.goto_xyz_rpy(x, y, altitude, 0, 0, 0, 1/20, isClose=False)  # move

    c.log_info("Figure-Eight Pattern Complete")


def move():
    """
    Move UAV in a figure-eight pattern at given height and width for given
    repititions and altitude levels
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    c = MavrospyController(rate)  # create mavrospy controller instance

    min_height = 1.0  # min height to fly at
    max_height = 3.0  # max height to fly at
    width = 3.0  # width of the figure-eight pattern
    length = 3.0  # length of the figure-eight pattern
    levels = 3  # number of different altitudes to complete figure-eight pattern
    repetitions = 3  # number of times to repeat the figure-eight pattern at each altitude

    # Create list of different altitudes to fly from min to max height and number of levels
    altitudes = [min_height + (max_height - min_height) * l / levels for l in range(1, levels+1)]

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

    # Go to center of figure-eight pattern
    c.goto_xyz_rpy(width/2, length/2, altitudes[0], 0, 0, 0)

    # Go to first point on figure-eight pattern
    c.goto_xyz_rpy(width, length/2, altitudes[0], 0, 0, 0)

    # Fly figure-eight pattern at all altitudes
    for altitude in altitudes:
        c.log_info(f"Pattern Altitude: {altitude}ft")
        for r in range(repetitions):  # repeat figure-eight pattern
            fly_figure_eight(c, width, length, altitude)

    # Go back home
    c.goto_xyz_rpy(0, 0, altitudes[0], 0, 0, 0)

    # Land
    c.log_info("Landing")
    c.land()


if __name__ == "__main__":
    move()
