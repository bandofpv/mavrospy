#!/usr/bin/env python3

import rospy
import math
from control_node import MavrospyController


def fly_figure_eight(c, width, length, altitude, resolution=360):
    # Generate figure-eight points
    for i in range(resolution):
        theta = 2 * math.pi * i / resolution
        x = width/2 * math.cos(theta)
        y = length/2 * math.sin(2 * theta)

        # Account for center offset
        x += width/2
        y += length/2

        next_theta = 2 * math.pi * (i + 1) / resolution
        next_x = width/2 * math.cos(next_theta)
        next_y = length/2 * math.sin(2 * next_theta)

        # Account for center offset
        next_x += width/2
        next_y += length/2

        yaw = math.atan2(next_y - y, next_x - x) - math.pi/2

        c.goto_xyz_rpy(x, y, altitude, 0, 0, yaw, 1/20, isClose=False)  # move

    c.log_info("Figure-Eight Pattern Complete")


def move():
    rate = 20
    c = MavrospyController(rate)

    min_height = 1.0
    max_height = 6.0
    width = 10.0
    length = 5.0
    levels = 3
    repetitions = 3

    # Create list of different altitudes to fly from min to max height and number of levels
    altitudes = [min_height + (max_height - min_height) * l / levels for l in range(1, levels+1)]

    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 1, False, False)

    c.log_info(f"Takeoff: {altitudes[0]}ft")
    c.takeoff(altitudes[0])

    # Go to center of figure-eight pattern
    c.goto_xyz_rpy(width/2, length/2, altitudes[0], 0, 0, 0)

    # Go to first point on figure-eight pattern
    c.goto_xyz_rpy(width, length/2, altitudes[0], 0, 0, 0)

    for altitude in altitudes:
        c.log_info(f"Pattern Altitude: {altitude}ft")
        for r in range(repetitions):  # repeat figure-eight pattern
            fly_figure_eight(c, width, length, altitude)

    c.goto_xyz_rpy(0, 0, altitudes[0], 0, 0, 0)
    c.log_info("Landing")
    c.land()


if __name__ == "__main__":
    move()
