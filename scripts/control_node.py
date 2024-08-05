import tf
import math
import rospy
from pymavlink import mavutil
from mavros_msgs.msg import State, ExtendedState
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


class MavrospyController:
    """
    Controller class to help interface with mavros
    """

    def __init__(self, send_rate):
        # initialize our control node
        rospy.init_node("mavros_node")

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

        self.log_info("MavController Initiated")

    def state_callback(self, data):
        self.current_state = data

    def extended_state_callback(self, data):
        self.current_extended_state = data

    def pose_callback(self, data):
        self.timestamp = data.header.stamp  # timestamp pose message
        self.pose = data.pose

    def log_info(self, info):
        if not rospy.is_shutdown():
            rospy.loginfo(info)

    def log_error(self, error):
        if not rospy.is_shutdown():
            rospy.logerr(error)

    def pause(self):
        """
        Pause at given rate
        """
        try:
            self.rate.sleep()
        except rospy.ROSException as e:
            self.log_error(e)

    from mavros_msgs.srv import CommandLong

    def set_gps_global_origin(self):
        # rospy.wait_for_service('/mavros/cmd/command')
        try:
            command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
            response = command_service(
                broadcast=False,
                command=48,    # MAV_CMD_SET_GPS_GLOBAL_ORIGIN
                confirmation=0,
                param1=0,      # Target system ID, 0 for the current system
                param2=0,
                param3=0,
                param4=0,
                param5=self.lat,
                param6=self.long,
                param7=self.alt
            )
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return False

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
                        self.log_error(f"Failed to send {info} command")
                except rospy.ServiceException as e:
                    self.log_error(e)

                if self.current_state.armed == arm_status:  # break when arm status is set
                    self.log_info(f"{info.capitalize()}ed Throttle")
                    return True

                self.pause()

            self.log_error(f"Failed to {info} throttle in {timeout} seconds")
        except rospy.ROSException as e:
            self.log_error(e)

    def goto(self, pose):
        """
        Set the given pose as a next set point by sending a SET_POSITION_TARGET_LOCAL_NED message. The copter must be in OFFBOARD mode for this to work.
        """
        # initialize ROS PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        try:
            self.cmd_pos_pub.publish(pose_stamped)  # publish PoseStamed message
        except rospy.ROSException as e:
            self.log_error(e)

    def goto_xyz_rpy(self, x, y, z, roll, pitch, yaw, timeout=30, loop=True, checkMode=True):
        """
        Sets the given pose as a next set point for given timeout (seconds).
        """
        # must be in OFFBOARD mode
        if checkMode and self.current_state.mode != "OFFBOARD":
            self.log_error("Cannot execute goto: not in OFFBOARD mode")
            return False

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

        # check if UAV is close to target setpoint
        def is_close(target, current, euler=False, tolerance=0.23):
            # if its a euler angle, change current to be between 0 and 2 pi
            if euler:
                while current > 2 * math.pi:
                    current -= 2 * math.pi
                while current < 0:
                    current += 2 * math.pi

            return abs(target - current) < tolerance

        if loop:
            for i in range(timeout * self.freq):
                self.goto(pose)
                self.pause()

                euler = tf.transformations.euler_from_quaternion(
                    [self.pose.orientation.x,
                    self.pose.orientation.y,
                    self.pose.orientation.z,
                    self.pose.orientation.w])

                # print(yaw + self.pi_2, yaw + self.pi_2 - math.pi, abs(euler[2]))

                if (is_close(x, self.pose.position.x) and
                    is_close(y, self.pose.position.y) and
                    is_close(z, self.pose.position.z) and
                    is_close(yaw + self.pi_2, euler[2], True)):
                    self.log_info("Reached target position")
                    break
        else:
            self.goto(pose)  # go to given pose
            self.pause()

    def takeoff(self, height, timeout=30):
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

        self.log_info("Taking Off")
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
        self.log_info("Changing Mode: AUTO.LAND")

        while not rospy.is_shutdown():
            try:
                resp = self.mode_service(custom_mode="AUTO.LAND")  # set mode to AUTO.LAND
                if not resp.mode_sent:
                    self.log_error("Failed to Change Mode: AUTO.LAND")
            except rospy.ServiceException as e:
                self.log_error(e)

            # break when on the ground
            if self.current_extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                break
            self.pause()

        self.log_info("Landed")

        self.arm(False)  # disarm throttle
        self.log_info("Disarmed")

# TODO: relative ned AND vel (after sitl with current setup)
