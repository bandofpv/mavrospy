#! /usr/bin/env python

import rospy
from command import send_cmd
from control_node import MavController


def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    c = MavController()
    # rospy.sleep(1)
    alt = 2.0

    while not rospy.is_shutdown() and c.current_state.mode != "OFFBOARD":
        if c.current_state.mode == "OFFBOARD":
            rospy.loginfo("OFFBOARD enabled")
            break

        c.goto_xyz_rpy(0.0, 0.0, 0.0, 0, 0, 0)

        rate.sleep()

    rospy.loginfo("Takeoff " + str(alt))
    c.takeoff(alt)
    # rospy.sleep(3)
    send_cmd(c.takeoff(alt), 3.0)

    # c.goto_xyz_rpy(0, 0, alt, 0, 0, 0)
    # rospy.sleep(3)

    rospy.loginfo("Waypoint 1: position control")
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, -1 * c.pi_2)
    # rospy.sleep(2)
    send_cmd(c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, -1 * c.pi_2), 2.0)
    c.goto_xyz_rpy(0.4, 0.0, alt, 0, 0, -1 * c.pi_2)
    # rospy.sleep(3)
    send_cmd(c.goto_xyz_rpy(0.4, 0.0, alt, 0, 0, -1 * c.pi_2), 3.0)
    rospy.loginfo("Waypoint 2: position control")
    c.goto_xyz_rpy(0.4, 0.0, alt, 0, 0, 0)
    # rospy.sleep(2)
    send_cmd(c.goto_xyz_rpy(0.4, 0.0, alt, 0, 0, 0), 2.0)
    c.goto_xyz_rpy(0.4, 0.4, alt, 0, 0, 0)
    # rospy.sleep(3)
    send_cmd(c.goto_xyz_rpy(0.4, 0.4, alt, 0, 0, 0), 3.0)
    rospy.loginfo("Waypoint 3: position control")
    c.goto_xyz_rpy(0.4, 0.4, alt, 0, 0, c.pi_2)
    # rospy.sleep(2)
    send_cmd(c.goto_xyz_rpy(0.4, 0.4, alt, 0, 0, c.pi_2), 2.0)
    c.goto_xyz_rpy(0.0, 0.4, alt, 0, 0, c.pi_2)
    # rospy.sleep(3)
    send_cmd(c.goto_xyz_rpy(0.0, 0.4, alt, 0, 0, c.pi_2), 3.0)
    rospy.loginfo("Waypoint 4: position control")
    c.goto_xyz_rpy(0.0, 0.4, alt, 0, 0, 2 * c.pi_2)
    # rospy.sleep(2)
    send_cmd(c.goto_xyz_rpy(0.0, 0.4, alt, 0, 0, 2 * c.pi_2), 2.0)
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 2 * c.pi_2)
    # rospy.sleep(3)
    send_cmd(c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 2 * c.pi_2), 3.0)
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 3 * c.pi_2)
    # rospy.sleep(1)
    send_cmd(c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 3 * c.pi_2), 1.0)
    c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 4 * c.pi_2)
    rospy.sleep(2)
    send_cmd(c.goto_xyz_rpy(0.0, 0.0, alt, 0, 0, 4 * c.pi_2), 2.0)

    rospy.loginfo("Landing")
    c.land()
    send_cmd(c.land(), 10.0)

    rospy.loginfo("Disarming")
    c.disarm()


if __name__ == "__main__":
    simple_demo()

