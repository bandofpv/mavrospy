import tf
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL

# from mavros_msgs.msg import OverrideRCIn
# from mavros_msgs.msg import RCIn


class MavController:
    """
    A simple object to help interface with mavros
    """

    def __init__(self):
        rospy.init_node("mav_control_node")

        rospy.Subscriber("mavros/state", State, self.state_callback)

        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        # rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)

        # self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)
        rospy.wait_for_service("/mavros/cmd/set_mode")
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        rospy.wait_for_service("/mavros/cmd/takeoff")
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        rospy.wait_for_service("/mavros/cmd/land")
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        self.current_state = State()
        # self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()
        self.pi_2 = 3.141592654 / 2.0

        # Wait for Flight Controller connection
        while not self.current_state.connected:
            rospy.sleep(0.1)

    # def rc_callback(self, data):
    #     """
    #     Keep track of the current manual RC values
    #     """
    #     self.rc = data

    def state_callback(self, data):
        self.current_state = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def goto(self, pose):
        """
        Set the given pose as a next set point by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in OFFBOARD mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya + self.pi_2)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send command velocities. Must be in OFFBOARD mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=2.0):
        """
        Set to offboard mode, arm the throttle, and takeoff to a few feet
        """
        # Set to offboard mode
        # mode_resp = self.mode_service(custom_mode="OFFBOARD")

        self.arm()

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        # self.goto_xyz_rpy(self.pose.position.x, self.pose.position.y, height, 0, 0, 0)

        # return takeoff_resp
        return takeoff_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        # resp = self.mode_service(custom_mode="AUTO.LAND")
        # self.goto_xyz_rpy(self.pose.position.x, self.pose.position.y, self.pose.position.z, 0, 0, 0)
        # self.disarm()
        resp = self.land_service()

        # return resp
