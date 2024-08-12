#! /usr/bin/env python3

import rospy
import math
from control_node import MavrospyController


def fly_circle(c, radius, repetitions, altitude, resolution=180):
    """
    Fly in a circle pattern facing only in forward direction
    """
    for r in range(repetitions):
        # Generate circle points
        for i in range(resolution):
            theta = 2 * math.pi * i / resolution  # angle from center to perimeter
            x = radius * math.cos(theta)  # calculate x position
            y = radius * math.sin(theta)  # calculate y position

            # Account for center offset
            x += radius
            y += radius

            # If first or last position, wait until it reaches/isClose goal position
            if i == 1 or i == resolution-1:
                c.goto_xyz_rpy(x, y, altitude, 0, 0, 0)
            else:  # just send command regardless of current position
                c.goto_xyz_rpy(x, y, altitude, 0, 0, 0, 1/20, isClose=False)

        c.log_info("Circle Pattern Complete")

def move():
    """
    Move UAV in a circle pattern at given height and width for given
    repititions and altitude levels
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    c = MavrospyController(rate)  # create mavrospy controller instance

    max_height = 6.0  # max height to fly at
    max_width = 10.0  # width of the circle pattern
    levels = 3  # number of different altitude to complete square pattern
    repetitions = 3   # number of times to repeat the square pattern at each altitude
    radius = max_width / 2 #radius of circle pattern

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

    # Go to center of circle
    c.goto_xyz_rpy(radius, radius, altitudes[0], 0, 0, 0)

    # Fly circle pattern at all altitudes
    for altitude in altitudes:
        fly_circle(c, radius, repetitions, altitude)

    # Go back home
    c.goto_xyz_rpy(0, 0, max_height/levels, 0, 0, 0)

    # Land
    c.log_info("Landing")
    c.land()


if __name__ == "__main__":
    move()
