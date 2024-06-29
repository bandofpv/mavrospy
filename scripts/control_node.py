import tf
import math
import rospy
from pymavlink import mavutil
from mavros_msgs.msg import State, ExtendedState
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandHome


class MavController:
    """
    Controller class to help interface with mavros
    """

    def __init__(self, send_rate):
        # initialize our control node
        rospy.init_node("mav_control_node")

        # create subscribers
        rospy.Subscriber("mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self.extended_state_callback)

        # create publishers
        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

        # create services
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.home_service = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)

        # initialize ROS messages
        self.pose = Pose()
        self.current_state = State()
        self.timestamp = rospy.Time()
        self.current_extended_state = ExtendedState()

        # initialize constants
        self.pi_2 = math.pi / 2.0
        self.freq = send_rate
        self.rate = rospy.Rate(send_rate)

        self.lat = 389853504  # Hopper Hall 389853504
        self.lon = -764857648  # Hopper Hall -764857648
        self.alt = 36810  # Hopper Hall 36810

        rospy.loginfo("MavController Initiated")

    def state_callback(self, data):
        self.current_state = data

    def extended_state_callback(self, data):
        self.current_extended_state = data

    def pose_callback(self, data):
        self.timestamp = data.header.stamp  # timestamp pose message
        self.pose = data.pose

    def pause(self):
        """
        Pause at given rate
        """
        try:
            self.rate.sleep()
        except rospy.ROSException as e:
            rospy.logerr(e)

    def set_home_position(self):
        """
        WIP
        """
        try:
            resp = self.home_service(latitude=self.lat, longitude=self.lon, altitude=self.alt)
            if not resp.success:
                rospy.logerr("Failed to set home position")
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def arm(self, arm_status, timeout=5):
        """
        Arm the throttle.
        """
        info = "arm" if arm_status else "disarm"

        try:
            for i in range(timeout * self.freq):  # loop for given timeout
                try:
                    resp = self.arm_service(arm_status)  # set arm status
                    if not resp.success:
                        rospy.logerr(f"Failed to send {info} command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

                if self.current_state.armed == arm_status:  # break when arm status is set
                    rospy.loginfo(f"{info.capitalize()}ed Throttle")
                    return True

                self.pause()

            rospy.logerr(f"Failed to {info} throttle in {timeout} seconds")
        except rospy.ROSException as e:
            rospy.logerr(e)

    def goto(self, pose):
        """
        Set the given pose as a next set point by sending a SET_POSITION_TARGET_LOCAL_NED message. The copter must be in
        OFFBOARD mode for this to work.
        """
        # TODO: FIX make seperate buffer??? Not needed if 1 sec goto 0 before arm... test
        if self.current_state.mode != "OFFBOARD":  # require OFFBOARD mode
            return False

        # initialize ROS PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        try:
            self.cmd_pos_pub.publish(pose_stamped)  # publish PoseStamed message
        except rospy.ROSException as e:
            rospy.logerr(e)

    def goto_xyz_rpy(self, x, y, z, roll, pitch, yaw, timeout, loop=True):
        """
        Sets the given pose as a next set point for given timeout (seconds).
        """
        # initialize Pose message
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # TODO: why +pi_2?? test without might be bugging my takeoff
        # convert euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw + self.pi_2)

        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        if loop:
            for i in range(timeout * self.freq):  # loop for given timeout
                self.goto(pose)  # go to given pose
                self.pause()
        else:
            self.goto(pose)  # go to given pose
            self.pause()

    def set_vel(self, vx, vy, vz, avx, avy, avz, timeout):
        """
        Send command velocities. Must be in OFFBOARD mode. Assumes angular velocities are zero by default.

        WIP
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        for i in range(timeout * self.freq):
            self.cmd_vel_pub.publish(cmd_vel)
            self.pause()

    def takeoff(self, height, timeout):
        """
        Arm the throttle and takeoff to a few feet

        WIP
        """
        self.arm(True)  # arm throttle

        # Takeoff

        self.goto_xyz_rpy(0, 0, height, 0, 0, 0, timeout)

        # explicit_quat = [self.pose.orientation.x, self.pose.orientation.y,
        #                  self.pose.orientation.z, self.pose.orientation.w]
        # roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)

        rospy.loginfo("Taking Off")
        # self.goto_xyz_rpy(self.pose.position.x, self.pose.position.y, height, roll, pitch, yaw, timeout)

    def test_takeoff(self, height, timeout):
        """
        WIP
        """
        self.arm(True)



    def land(self):
        """
        Set mode to AUTO.LAND for immediate descent and disarm when on ground.
        """
        rospy.loginfo("Changing Mode: AUTO.LAND")

        while True:
            try:
                resp = self.mode_service(custom_mode="AUTO.LAND")  # set mode to AUTO.LAND
                if not resp.mode_sent:
                    rospy.logerr("Failed to Change Mode: AUTO.LAND")
            except rospy.ServiceException as e:
                rospy.logerr(e)

            # break when on the ground
            if self.current_extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                break
            self.pause()

        rospy.loginfo("Landed")

        self.arm(False)  # disarm throttle
        rospy.loginfo("Disarmed")

# TODO: relative ned AND vel (after sitl with current setup)
