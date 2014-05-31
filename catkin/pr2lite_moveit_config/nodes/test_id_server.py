#!/usr/bin/env python

"""
    Relax all servos by disabling the torque for each.
"""
import roslib
roslib.load_manifest('pr2lite_moveit_config')
import rospy, time
from pr2lite_moveit_config.srv import *

if __name__=='__main__':
   rospy.init_node('test_id_server')
   rospy.wait_for_service("search_id")
   search_svc = rospy.ServiceProxy("search_id", SearchID)

   print search_svc("wheel-cnt", "front left")
   print search_svc("wheel-cnt", "front right") 
   print search_svc("wheel-cnt", "back right")
   print search_svc("wheel-cnt", "back left")

