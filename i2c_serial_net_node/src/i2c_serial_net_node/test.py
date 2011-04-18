#!/usr/bin/env python

import roslib
roslib.load_manifest('i2c_serial_net_node')
import rospy
from i2c_serial_net_node.srv import *
import binascii

rospy.wait_for_service('i2c_in_synch')
s = rospy.ServiceProxy('i2c_in_synch', i2c_sync_request)
resp = s('\x04\xfe\x00\xfe', 16)
print binascii.hexlify(resp.resp)
