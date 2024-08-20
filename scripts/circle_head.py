#! /usr/bin/env python3

import rospy
import math
from control_node import MavrospyController


def fly_circle(c, radius, altitude, resolution=180):
    """
    Fly in a circle pattern facing direction of motion
    """
    # Generate circle points
    for i in range(resolution):
        theta = 2 * math.pi * i / resolution  # angle from center to perimeter
        x = radius * math.cos(theta)  # calculate x position
        y = radius * math.sin(theta)  # calculate y position

        # Account for center offset
        x += radius
        y += radius

        # calculate next point on circle
        next_theta = 2 * math.pi * (i + 1) / resolution
        next_x = radius * math.cos(next_theta)
        next_y = radius * math.sin(next_theta)

        # Account for center offset
        next_x += radius
        next_y += radius

        yaw = math.atan2(next_y - y, next_x - x) - math.pi/2  # calculate yaw to face forward along the path

        c.goto_xyz_rpy(x, y, altitude, 0, 0, yaw, 1/20, isClose=False)  # move

    c.log_info("Circle Pattern Complete")


def move():
    """
    Move UAV in a circle pattern at given height and width for given
    repititions and altitude levels
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    c = MavrospyController(rate)  # create mavrospy controller instance

    min_height = 1.0  # min height to fly at
    max_height = 3.0  # max height to fly at
    width = 3.0  # width of the circle pattern
    radius = width / 2  # radius of circle pattern
    levels = 3  # number of different altitude to complete circle pattern
    repetitions = 3  # number of times to repeat the circle pattern at each altitude

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

    # Go to center of circle
    c.goto_xyz_rpy(radius, radius, altitudes[0], 0, 0, 0)

    # Go to first point on circle
    c.goto_xyz_rpy(radius*2, radius, altitudes[0], 0, 0, 0)

    # Fly circle pattern at all altitudes
    for altitude in altitudes:
        c.log_info(f"Pattern Altitude: {altitude}ft")
        for r in range(repetitions):  # repeat circle pattern
            fly_circle(c, radius, altitude)

    # Go back home
    c.goto_xyz_rpy(0, 0, altitudes[0], 0, 0, 0)

    # Land
    c.log_info("Landing")
    c.land()


if __name__ == "__main__":
    move()
