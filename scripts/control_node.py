import tf
import math
import rospy
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State, ExtendedState
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandHome, CommandHomeRequest

class MavrospyController:
    """
    Controller class to help interface with mavros
    """
    def __init__(self, frequency):
        # Initialize our control node
        rospy.init_node("mavrospy_node")

        # Create subscribers
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self.extended_state_callback)
        rospy.Subscriber('/mavros/global_position/gp_origin', GeoPointStamped, self.origin_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)

        # Create publishers
        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.origin_pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=10)

        # Create services
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_home_service = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)

        # Initialize ROS messages
        self.pose = Pose()
        self.current_state = State()
        self.timestamp = rospy.Time()
        self.current_extended_state = ExtendedState()

        # Initialize constants
        self.pi_2 = math.pi / 2.0
        self.freq = frequency
        self.rate = rospy.Rate(frequency)
        self.target_lat = 38.9853504  # Latitude of the Hopper Hall
        self.target_lon = -76.4857648  # Longitude of the Hopper Hall
        self.target_alt = 36.810  # Altitude of the Hopper Hall

        # Status variables
        self.origin_set = False
        self.current_gps = None

        # Set global origin and home position
        self.set_global_origin()
        self.wait_for_gps_fix()
        self.set_home_position()

        self.log_info("MavrospyController Initiated")

    def state_callback(self, data):
        """
        Callback for PX4 state data
        """
        self.current_state = data

    def extended_state_callback(self, data):
        """
        Callback for PX4 extended state data
        """
        self.current_extended_state = data

    def pose_callback(self, data):
        """
        Callback for PX4 pose data
        """
        self.timestamp = data.header.stamp  # timestamp pose message
        self.pose = data.pose

    def origin_callback(self, data):
        """
        Callback for global origin data
        """
        # Check if global origin is set properly
        if data.position.latitude == self.target_lat and data.position.longitude == self.target_lon:
            self.origin_set = True
        else:
            rospy.logwarn("Global origin mismatch: Expected (%.6f, %.6f) but got (%.6f, %.6f)",
                          self.target_lat, self.target_lon, data.position.latitude, data.position.longitude)

    def gps_callback(self, data):
        """
        Callback for GPS data
        """
        self.current_gps = data

    def log_info(self, info):
        """
        Log messaging
        """
        if not rospy.is_shutdown():  # only log info when mavrospy node is running
            rospy.loginfo(info)

    def log_error(self, error):
        """
        Error messaging
        """
        if not rospy.is_shutdown():  # only log errors when mavropsy node is running
            rospy.logerr(error)

    def pause(self):
        """
        Pause at given rate
        """
        try:
            self.rate.sleep()
        except rospy.ROSInterruptException as e:
            self.log_error(e)

    def service_request(self, service, request):
        """
        Attempt to service request
        """
        try:
            resp = service(request)  # service request

            # Check for response success
            success = None
            for field_name in ['success', 'result', 'status', 'mode_sent']:  # Common names to check
                if hasattr(resp, field_name):
                    success = getattr(resp, field_name)
                    break

            if success is None:  # check if success field was found
                self.log_error("Could not determine if the service call was successful")
            elif not success:
                self.log_error(f"Failed to set {service} service")

        except rospy.ServiceException as e:
            self.log_error(e)

    def set_global_origin(self):
        """
        Set the global origin
        """
        def send_origin_msg():
            # Setup GeoPointStamped message
            origin_msg = GeoPointStamped()
            origin_msg.header.stamp = rospy.Time.now()
            origin_msg.header.frame_id = "map"  # Use "map" as the coordinate reference

            # Assign lat, long, and alt to the message
            origin_msg.position.latitude = self.target_lat
            origin_msg.position.longitude = self.target_lon
            origin_msg.position.altitude = self.target_alt

            # Publish GeoPointStamped message
            try:
                self.origin_pub.publish(origin_msg)
            except rospy.ROSException as e:
                self.log_error(e)

        self.log_info("Setting global origin...")

        # Keep sending global origin message until it is set properly
        while not rospy.is_shutdown():
            send_origin_msg()
            if self.origin_set:
                self.log_info("Global origin set.")
                break
            self.pause()

    def wait_for_gps_fix(self):
        """
        Wait for GPS fix
        """
        self.log_info("Waiting for GPS fix...")

        while not rospy.is_shutdown():
            if self.current_gps.status.status == NavSatStatus.STATUS_FIX:
                self.log_info("GPS fix acquired.")
                break
            self.pause()

    def set_home_position(self):
        """
        Set the home position
        """
        rospy.loginfo("Setting home position...")

        # Set the current GPS position as home
        self.service_request(self.set_home_service, CommandHomeRequest(current_gps=True, latitude=0, longitude=0, altitude=0))
        rospy.loginfo("Home position set.")

    def arm(self, status, timeout=5):
        """
        Arm the throttle.
        """
        info = "arm" if status else "disarm"

        try:
            for i in range(timeout * self.freq):  # loop until timeout
                self.service_request(self.arm_service, status)  # attempt to set arm status

                if self.current_state.armed == status:  # break when arm status is set
                    self.log_info(f"{info.capitalize()}ed Throttle")
                    return True

                self.pause()

            self.log_error(f"Failed to {info} throttle in {timeout} seconds")
        except rospy.ROSException as e:
            self.log_error(e)

    def check_offboard(self):
        """
        Check if PX4 is in OFFBOARD mode
        """
        if self.current_state.mode != "OFFBOARD":
            self.log_error("Not in OFFBOARD mode")
            return False

    def is_close(self, target, current, tolerance, quat=False):
        """
        Chek if UAV is close to target set point

        Recommended tolerances: point x & y --> 0.2
                                point z --> 0.5
                                quaternion x, y, z, & w --> 0.1
        """
        if quat:  # if comparing quaternions
            return abs(target - current) < tolerance or abs(target + current) < tolerance

        return abs(target - current) < tolerance


    def goto(self, pose):
        """
        Set the given pose as a next set point by sending a SET_POSITION_TARGET_LOCAL_NED message.
        The copter must be in OFFBOARD mode for this to work.
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

        isClose: check if UAV is close to target set point
        checkMode: check if UAV is in OFFBOARD mode
        """
        # Check if in OFFBOARD mode
        if checkMode:
            self.check_offboard()

        # initialize Pose message
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z

        # convert euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw + self.pi_2)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion[0], quaternion[1], quaternion[2], quaternion[3]

        for i in range(int(timeout * self.freq)):  # loop for given timeout
            self.goto(pose)  # move to set point

            # check if isClose
            if (isClose and
                self.is_close(x, self.pose.position.x, 0.2) and
                self.is_close(y, self.pose.position.y, 0.2) and
                self.is_close(z, self.pose.position.z, 0.5) and
                self.is_close(quaternion[0], self.pose.orientation.x, 0.1, True) and
                self.is_close(quaternion[1], self.pose.orientation.y, 0.1, True) and
                self.is_close(quaternion[2], self.pose.orientation.z, 0.1, True) and
                self.is_close(quaternion[3], self.pose.orientation.w, 0.1, True)):
                    self.log_info("Reached target position")
                    break

            self.pause()

    def takeoff(self, height, timeout=30):
        """
        Arm the throttle and takeoff to given height (feet)
        """
        self.arm(True)  # arm throttle

        self.log_info("Taking Off")
        self.goto_xyz_rpy(0, 0, height, 0, 0, 0, timeout)  # takeoff

    def land(self):
        """
        Set mode to AUTO.LAND for immediate descent and disarm when on ground.
        """
        self.log_info("Changing Mode: AUTO.LAND")

        while not rospy.is_shutdown():
            self.service_request(self.mode_service, SetModeRequest(custom_mode="AUTO.LAND"))  # change mode to AUTO.LAND

            # Break when on the ground
            if self.current_extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                break

            self.pause()

        self.log_info("Landed")

        self.arm(False)  # disarm throttle
