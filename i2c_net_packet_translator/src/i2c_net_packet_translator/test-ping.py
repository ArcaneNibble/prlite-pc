#!/usr/bin/env python

import roslib
roslib.load_manifest('i2c_net_packet_translator')
import rospy
from i2c_net_packets.srv import *

rospy.wait_for_service('i2c_net_ping')
func = rospy.ServiceProxy('i2c_net_ping', i2c_ping)
resp = func(0x04)
print resp
