#!/usr/bin/env python3

##
#
# Send SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION messages
#
##

import rospy
from builtins import object
from pymavlink.dialects.v10 import common as MAV_COMMON

# Global position of the origin
lat = 38.9853504 * 1e7   # Hopper Hall 389853504
lon = -76.4857648 * 1e7   # Hopper Hall -764857648
alt = 36.810 * 1e3  # Hopper Hall 36810

class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)


def send_message(msg, mav):
    """
    Send a mavlink message
    """
    msg.pack(mav)

def set_global_origin(mav):
    """
    Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
    to use local position information without a GPS.
    """
    target_system = mav.srcSystem
    latitude = lat
    longitude = lon
    altitude = alt

    msg = MAV_COMMON.set_gps_global_origin_send(
            target_system,
            latitude,
            longitude,
            altitude)

    send_message(msg, mav)


def set_home(mav):
    current = 0
    roll = 0
    pitch = 0
    yaw = 0

    latitude = lat
    longitude = lon
    altitude = alt

    msg = MAV_COMMON.mav_cmd_do_set_home_send(
        current,
        roll,
        pitch,
        yaw,
        latitude,
        longitude,
        altitude
    )

    send_message(msg, mav)


def set_home_position(mav, pub):
    """
    Send a mavlink SET_HOME_POSITION message, which should allow
    us to use local position information without a GPS
    """
    target_system = mav.srcSystem
    #target_system = 0  # broadcast to everyone

    lattitude = lat
    longitude = lon
    altitude = alt
    
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = MAV_COMMON.MAVLink_set_home_position_message(
            target_system,
            lattitude,
            longitude,
            altitude,
            x,
            y,
            z,
            q,
            approach_x,
            approach_y,
            approach_z)

    send_message(msg, mav)

if __name__=="__main__":
    try:
        # Set up mavlink instance
        f = fifo()
        mav = MAV_COMMON.MAVLink(f, srcSystem=1, srcComponent=1)

        for _ in range(2):
            rospy.sleep(1)
            set_global_origin(mav)
            # set_home_position(mav, mavlink_pub)
            set_home(mav)
    except rospy.ROSInterruptException:
        pass

