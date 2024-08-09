#! /usr/bin/env python3

import rospy
import math
from control_node import MavrospyController  # Assuming control_node.py is in the same directory


def fly_circle(c, radius, repetitions, altitude, resolution=180):
    """
    Fly in a circle pattern facing direction of motion
    """
    for r in range(repetitions):
        # Generate circle points
        for i in range(resolution):
            theta = 2 * math.pi * i / resolution
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)
            yaw = math.atan2(y, x)  # Yaw to face forward along the path

            x += radius
            y += radius

            # Command drone to go to each point
            if i == 1 or i == resolution-1:
                c.goto_xyz_rpy(x, y, altitude, 0, 0, yaw)
            else:
                c.goto_xyz_rpy(x, y, altitude, 0, 0, yaw, 1/20, isClose=False)

def move():
    """
    Move UAV in a circle pattern at given height and width for given
    repititions and altitude levels
    """
    rate = 20
    c = MavrospyController(rate)

    max_height = 6.0  # Max height to fly at
    max_width = 10.0  # Width of the circle pattern
    levels = 3  # Number of different altitude to complete square pattern
    repetitions = 3   # Number of times to repeat the square pattern at each altitude
    radius = max_width / 2 # Radius of circle pattern

    altitudes = []

    for l in range(1, levels + 1):
        altitudes.append((max_height * l) / levels)

    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        # Before entering OFFBOARD mode, you must have already started streaming setpoints
        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 1, False, False)

    c.log_info(f"Takeoff: {altitudes[0]}m")
    c.takeoff(altitudes[0])

    # Go to center of circle
    c.goto_xyz_rpy(radius, radius, altitudes[0], 0, 0, 0)

    # Fly circle pattern
    for altitude in altitudes:
        fly_circle(c, radius, repetitions, altitude) 

    # Go back home
    c.goto_xyz_rpy(0, 0, max_height/levels, 0, 0, 0)

    c.log_info("Landing")
    c.land()


if __name__ == "__main__":
    move()
