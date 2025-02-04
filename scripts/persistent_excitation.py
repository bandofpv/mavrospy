#!/usr/bin/env python3

import math
import rospy
import random
from control_node import MavrospyController

def fly_dynamic_figure_eight(c, base_width, base_length, base_altitude, yaw_amplitude, altitude_amplitude, size_amplitude, resolution=360):
    """
    Fly a dynamic figure-eight pattern with user-controlled parameters for yaw, altitude, and size variations.
    """
    # Randomize parameters
    resolution = random.randint(180, 360)
    yaw_frequency = random.uniform(0.01, 0.03)
    altitude_frequency = random.uniform(0.02, 0.05)
    width = random.uniform(base_width - 2*size_amplitude, base_width)
    length = random.uniform(base_length - 2*size_amplitude, base_length)

    # Generate figure-eight points
    for i in range(resolution):
        if i % 180 == 0:
            # Randomize parameters every 180 degrees
            width = random.uniform(base_width - 2*size_amplitude, base_width)
            length = random.uniform(base_length - 2*size_amplitude, base_length)

        theta = 2 * math.pi * i / resolution
        x = width / 2 * math.cos(theta)
        y = length / 2 * math.sin(2 * theta)

        # Account for center offset
        x += base_width / 2
        y += base_length / 2

        # Calculate dynamic parameters for each point based on time and resolution step
        dynamic_yaw = yaw_amplitude * math.sin(2 * math.pi * yaw_frequency * i)
        dynamic_altitude = base_altitude + altitude_amplitude * math.sin(2 * math.pi * altitude_frequency * i)

        # Move to the calculated position with dynamic yaw and altitude
        c.goto_xyz_rpy(x, y, dynamic_altitude, 0, 0, dynamic_yaw, 1/20, isClose=False)

    c.log_info("Dynamic Figure-Eight Pattern Iteration Complete")

def move():
    """
    Move UAV in a continuously dynamic figure-eight pattern with user-defined parameters.
    """
    rate = 20
    c = MavrospyController(rate)

    # User-defined parameters
    base_altitude = 2.0  # Base altitude for flight
    base_width = 4.0  # Base width of figure-eight
    base_length = 2.5  # Base length of figure-eight
    total_duration = 5*60  # Total flight duration in seconds
    yaw_amplitude = 1.0  # rad
    altitude_amplitude = 0.5  # meters
    size_amplitude = 1.0  # meters

    # Wait until drone is in OFFBOARD mode
    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break
        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 1, False, False)

    # Takeoff
    c.log_info(f"Takeoff at altitude: {base_altitude} meters")
    c.takeoff(base_altitude)

    # Go to starting position
    c.slow_goto_xyz_rpy(base_width/2, base_length/2, base_altitude, 0, 0, 0)

    start_time = rospy.get_time()

    # Fly dynamic figure-eight pattern for total duration
    while rospy.get_time() - start_time < total_duration:
        fly_dynamic_figure_eight(c, base_width, base_length, base_altitude, yaw_amplitude, altitude_amplitude, size_amplitude)

    # Return to center and land
    c.slow_goto_xyz_rpy(base_width/2, base_length/2, base_altitude, 0, 0, 0, height=True)
    c.log_info("Landing")
    c.land()

if __name__ == "__main__":
    move()
