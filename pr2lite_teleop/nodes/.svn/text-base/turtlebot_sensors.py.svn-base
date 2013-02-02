#!/usr/bin/env python

"""
    turtlebot_sensors.py - Version 1.0 2012-09-24
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('pi_teleop')
import rospy
from math import radians, isnan

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from turtlebot_nodes.msg import TurtlebotSensorState

class TurtleBotSensors():
    def __init__(self):
        rospy.init_node('turtlebot_sensors    ', anonymous=False)
        
        rospy.Subscriber("turtlebot_node/sensor_state", TurtlebotSensorState, self.turtlebot_sensors)
        
    def turtlebot_sensors(self, msg):
        rospy.loginfo(msg)

        
if __name__ == '__main__':
    try:
        TurtleBotSensors()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    
