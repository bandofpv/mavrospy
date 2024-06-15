#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("test_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arm_service = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_service = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if (rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    # offb_set_mode = SetModeRequest()
    # offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while not rospy.is_shutdown() and current_state.mode != "OFFBOARD":
        if current_state.mode == "OFFBOARD":
            rospy.loginfo("OFFBOARD enabled")
            break

        local_pos_pub.publish(pose)

        last_req = rospy.Time.now()

        rate.sleep()

    while not rospy.is_shutdown() and current_state.mode == "OFFBOARD":
        if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if arm_service.call(arm_cmd).success:
                rospy.loginfo("Vehicle armed")

        local_pos_pub.publish(pose)

        last_req = rospy.Time.now()

        rate.sleep()

    rospy.loginfo("OFFBOARD disabled")
