#! /usr/bin/env python3

import rospy
import math
from control_node import MavrospyController


def fly_spiral(c, min_radius, max_radius, min_height, max_height, final_radius, resolution=180):
    """
    Fly in a spiral pattern facing only in forward direction
    """
    # Calculate step size for radius and height
    radius_step = (max_radius - min_radius) / resolution
    height_step = (max_height - min_height) / resolution

    # Generate spiral points
    for i in range(resolution):
        theta = 2 * math.pi * i / resolution  # angle from center to perimeter
        radius = min_radius + i * radius_step  # calculate radius
        altitude = min_height + i * height_step  # calculate altitude

        x = radius * math.cos(theta)  # calculate x position
        y = radius * math.sin(theta)  # calculate y position

        # Account for center offset
        x += final_radius
        y += final_radius

        c.goto_xyz_rpy(x, y, altitude, 0, 0, 0, 1/20, isClose=False)  # move

    c.log_info("Spiral Pattern Complete")


def move():
    """
    Move UAV in a spiral pattern at given height and width
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    c = MavrospyController(rate)  # create mavrospy controller instance

    min_height = 1.0  # minimum height to fly at
    max_height = 3.0  # maximum height to fly at
    min_width = 0.0  # minimum width of spiral pattern
    max_width = 3.0  # maximum width of spiral pattern
    min_radius = min_width / 2  # minimum radius of spiral pattern
    max_radius = max_width / 2  # maximum radius of spiral pattern
    resolution = 180  # number of steps for smoothness
    levels = 10  # number different altitudes to complete spiral pattern

    # Create list of different altitudes to fly from min to max height and number of levels
    altitudes = [min_height + (max_height - min_height) * l / (levels-1) for l in range(levels)]
    radii = [min_radius + (max_radius - min_radius) * l / (levels-1) for l in range(levels)]

    # Wait until drone is in OFFBOARD mode
    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        # Before entering OFFBOARD mode, you must have already started streaming setpoints
        c.goto_xyz_rpy(0, 0, min_height, 0, 0, 0, 1, False, False)

    # Takeoff at lowest altitude in altitudes list
    c.log_info(f"Takeoff: {altitudes[0]}ft")
    c.takeoff(altitudes[0])

    # Go to center of spiral pattern
    c.slow_goto_xyz_rpy(max_radius, max_radius, altitudes[0], 0, 0, 0)

    # Fly spiral pattern at all altitudes and radii
    for i in range(len(altitudes)-1):
        fly_spiral(c, radii[i], radii[i+1], altitudes[i], altitudes[i+1], max_radius, resolution)

    # Go back home
    c.slow_goto_xyz_rpy(0, 0, altitudes[0], 0, 0, 0, height=True)

    # Land
    c.log_info("Landing")
    c.land()

if __name__ == "__main__":
    move()
