import tf
import math
import rospy
from pymavlink import mavutil
from mavros_msgs.msg import State, ExtendedState
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

from geographic_msgs.msg import GeoPoint, GeoPointStamped
from mavros_msgs.srv import CommandLong

class MavrospyController:
    """
    Controller class to help interface with mavros
    """

    def __init__(self, send_rate):
        # initialize our control node
        rospy.init_node("mavrospy_node")

        # create subscribers
        rospy.Subscriber("mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self.extended_state_callback)

        # create publishers
        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.gp_origin_pub = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=1)

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

        self.log_info("MavrospyController Initiated")

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

    def set_ekf_origin(self, origin: GeoPoint) -> None:
        """Set the EKF origin.

        This is required for navigation on a vehicle with one of the provided localizers.

        Args:
            origin: The EKF origin to set.
        """
        origin_stamped = GeoPointStamped()
        origin_stamped.header.stamp = rospy.get_rostime()
        origin_stamped.position = origin
        self.gp_origin_pub.publish(origin_stamped)


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

    def goto_xyz_rpy(self, x, y, z, roll, pitch, yaw, timeout=30, isClose=True, checkMode=True):
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

        # convert euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw + self.pi_2)

        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        # TODO: play around with tolerance and set tolerance lower or quats... understand quats

        # check if UAV is close to target setpoint
        def is_close(target, current, tolerance, quat=False):
            """
            Recommended tolerances: point x & y --> 0.2
                                    point z --> 0.5
                                    quaternion x, y, z, & w --> 0.1
            """
            # if comparing quaternions
            if quat:
                return abs(target - current) < tolerance or abs(target + current) < tolerance

            return abs(target - current) < tolerance

        if isClose:
            for i in range(timeout * self.freq):
                self.goto(pose)
                self.pause()

                if (is_close(x, self.pose.position.x, 0.2) and
                    is_close(y, self.pose.position.y, 0.2) and
                    is_close(z, self.pose.position.z, 0.5) and
                    is_close(quaternion[0], self.pose.orientation.x, 0.1, True) and
                    is_close(quaternion[1], self.pose.orientation.y, 0.1, True) and
                    is_close(quaternion[2], self.pose.orientation.z, 0.1, True) and
                    is_close(quaternion[3], self.pose.orientation.w, 0.1, True)):
                    self.log_info("Reached target position")
                    break
        else:
            for i in range(int(timeout * self.freq)):
                self.goto(pose)
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
