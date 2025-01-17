#! /usr/bin/env python3

import time
import rospy
import random
from control_node import MavrospyController


def move():
    """
    Move UAV in random directions to generate persistent excitation
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = 20
    c = MavrospyController(rate)  # create mavrospy controller instance

    # Set center position
    center_position = [1.5, 1.5, 1.5]  # X, Y, Z in meters

    # Movement bounds
    position_bounds = 1  # ±1m in X and Y
    altitude_bounds = 0.5  # ±0.5m from start altitude
    orientation_bounds = 1  # ±1 radians

    # Flight time
    flight_time = 5*60  # seconds

    # Wait until drone is in OFFBOARD mode
    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        # Before entering OFFBOARD mode, you must have already started streaming setpoints
        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 1, False, False)

    # Takeoff
    c.log_info(f"Takeoff: {center_position[2]}m")
    c.takeoff(center_position[2])

    # Go to center
    c.log_info("Going to center...")
    c.slow_goto_xyz_rpy(center_position[0], center_position[1], center_position[2], 0, 0, 0)

    # Start timer
    start_time = time.time()

    c.log_info("Start persistent excitation")

    while not rospy.is_shutdown() and time.time() - start_time < flight_time:
        # Generate random target position and orientation
        target_x = center_position[0] + random.uniform(-position_bounds, position_bounds)
        target_y = center_position[1] + random.uniform(-position_bounds, position_bounds)
        target_z = center_position[2] + random.uniform(-altitude_bounds, altitude_bounds)
        target_yaw = random.uniform(-orientation_bounds, orientation_bounds)

        # Go to target position and orientation
        c.goto_xyz_rpy(target_x, target_y, target_z, 0, 0, target_yaw, timeout=1, isClose=False)

    # Land
    c.log_info("Landing")
    c.land()


if __name__ == "__main__":
    move()
