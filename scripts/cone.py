#! /usr/bin/env python3

import rospy
import math
from control_node import MavrospyController

def fly_spiral(c, min_radius, max_radius, min_height, max_height, resolution=180):
    radius_step = (max_radius - min_radius) / resolution
    height_step = (max_height - min_height) / resolution

    for i in range(resolution):
        theta = 2 * math.pi * i / resolution
        radius = min_radius + i * radius_step
        altitude = min_height + i * height_step
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        c.goto_xyz_rpy(x, y, altitude, 0, 0, 0, 1/20, isClose=False)
    c.log_info("Spiral Pattern Complete")

def move():
    rate = 20
    c = MavrospyController(rate)

    min_height = 1.0    # minimum height
    max_height = 10.0   # maximum height
    min_radius = 5.0    # minimum radius
    max_radius = 5.0   # maximum radius
    resolution = 360    # number of steps for smoothness
    levels = 10  # number different altitudes to complete spiral pattern

    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        c.goto_xyz_rpy(0, 0, min_height, 0, 0, 0, 1, False, False)

    c.log_info(f"Takeoff: {min_height}m")
    c.takeoff(min_height)

    # Create list of different altitudes to fly from min to max height and number of levels
    step = (max_height - min_height) / (levels - 1)
    altitudes = [min_height + i * step for i in range(levels)]

    for i in range(len(altitudes)-1):
        fly_spiral(c, min_radius, max_radius, altitudes[i], altitudes[i+1], resolution)

    c.goto_xyz_rpy(0, 0, min_height, 0, 0, 0)
    c.log_info("Landing")
    c.land()

if __name__ == "__main__":
    move()

