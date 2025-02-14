#!/usr/bin/env python3

import math
import rospy
import random
import numpy as np
from control_node import MavrospyController

def fly_dynamic_circle(c, base_radius, base_altitude, yaw_amplitude, altitude_amplitude, radius_amplitude, resolution=180):
    """
    Fly a dynamic circle pattern with user-controlled parameters for yaw, altitude, and radius variations.
    """
    c.log_info(f"Resolution: {resolution}")

    # Randomize frequency parameters
    yaw_frequency = random.uniform(0.01, 0.03)
    altitude_frequency = random.uniform(0.02, 0.05)

    # Generate figure-eight points
    for i in range(resolution):

        theta = 2 * math.pi * i / resolution  # angle from center to perimeter
        x = base_radius * math.cos(theta)  # calculate x position
        y = base_radius * math.sin(theta)  # calculate y position

        # Account for center offset
        x += base_radius
        y += base_radius

        # Calculate dynamic parameters for each point based on time and resolution step
        dynamic_yaw = yaw_amplitude * math.sin(2 * math.pi * yaw_frequency * i)
        dynamic_altitude = base_altitude + altitude_amplitude * math.sin(2 * math.pi * altitude_frequency * i)

        c.goto_xyz_rpy(x, y, dynamic_altitude, 0, 0, dynamic_yaw, 1/20, isClose=False)  # move

    c.log_info("Circle Pattern Complete")

def move():
    """
    Move UAV in a continuously dynamic figure-eight pattern with user-defined parameters.
    """
    rate = 20
    c = MavrospyController(rate)

    # User-defined parameters
    base_radius = 1.5  # base radius of circle
    base_altitude = 2.0  # base altitude for flight
    total_duration = 5*60  # total flight duration in seconds
    yaw_amplitude = 1.0  # rad
    altitude_amplitude = 0.5  # meters
    radius_amplitude = 1.0  # meters

    # Dynamic speed parameters
    min_resolution = 50  # minimum resolution of circle
    max_resolution = 150  # maximum resolution of circle
    T = 10  # number of steps per cycle
    k = 3.0 / T  # controls the rate of decay/growth

    # Wait until drone is in OFFBOARD mode
    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break
        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 1, False, False)

    # Takeoff
    c.log_info(f"Takeoff at altitude: {base_altitude} meters")
    c.takeoff(base_altitude)

    # Go to center of circle
    c.goto_xyz_rpy(base_radius, base_radius, base_altitude, 0, 0, 0)

    # Go to first point on circle
    c.goto_xyz_rpy(base_radius*2, base_radius, base_altitude, 0, 0, 0)

    start_time = rospy.get_time()  # start time of flight

    # Fly dynamic circle pattern for total duration
    while rospy.get_time() - start_time < total_duration:
        # Exponential resolution decay: 150 --> 60
        for t in range(T + 1):
            resolution = int(round(min_resolution + (max_resolution - min_resolution) * np.exp(-k * t)))
            fly_dynamic_circle(c, base_radius, base_altitude, yaw_amplitude, altitude_amplitude, radius_amplitude, resolution)
            if rospy.get_time() - start_time > total_duration:
                break

        # Exponential resolution growth: 60 --> 150
        for t in range(T + 1):
            resolution = int(round(min_resolution + (max_resolution - min_resolution) * np.exp(-k * (T - t))))
            fly_dynamic_circle(c, base_radius, base_altitude, yaw_amplitude, altitude_amplitude, radius_amplitude, resolution)
            if rospy.get_time() - start_time > total_duration:
                break

    # Land
    c.log_info("Landing")
    c.land()

if __name__ == "__main__":
    move()
