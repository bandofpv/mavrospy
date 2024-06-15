import rospy


def send_cmd(command, duration, rate=rospy.Rate(20)):
    start_time = rospy.Time.now()

    while (rospy.Time.now() - start_time) < rospy.Duration(duration):
        exec(command)
        rate.sleep()

