#! /usr/bin/env python3

import rospy
import math
from control_node import MavController  # Assuming control_node.py is in the same directory


def fly_circle(c, radius=5, altitude=3, resolution=36):
    # Generate circle points
    for i in range(resolution):
        theta = 2 * math.pi * i / resolution
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        yaw = math.atan2(y, x)  # Yaw to face forward along the path

        # Command drone to go to each point
        c.goto_xyz_rpy(x, y, altitude, 0, 0, yaw)

if __name__ == "__main__":
    c = MavController(20)  # Initialize the MavController
    alt = 3.0

    while not rospy.is_shutdown():
        if c.current_state.mode == "OFFBOARD":
            c.log_info("OFFBOARD enabled")
            break

        # Before entering OFFBOARD mode, you must have already started streaming setpoints
        c.goto_xyz_rpy(0, 0, 0, 0, 0, 0, 0, False, False)

    rospy.loginfo("Takeoff " + str(alt))
    c.takeoff(alt, 8)
    c.goto_xyz_rpy(0, 0, alt, 0, 0, 0, 3)

    fly_circle(c)

    c.log_info("Landing")
    c.land()
