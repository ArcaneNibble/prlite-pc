#!/usr/bin/env python

"""
    detect_obstacle_laser.py - Version 1.0 2012-09-24
    
    Detect an obstacle ahead of the robot based on a laser scan
    
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

import roslib; roslib.load_manifest('pi_rosbridge')
import rospy
from math import radians, isnan

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

class DetectObstacleLaser():
    def __init__(self):
        rospy.init_node('detect_obstacle_laser', anonymous=False)
    
        self.sector_width = radians(rospy.get_param("~sector_width", 60))
        
        self.obstacle_pub = rospy.Publisher("base_laser_obstacle", Float64)
        
        rospy.Subscriber("scan", LaserScan, self.process_scan)
         
    def process_scan(self, msg):
        min_distance = 100.0
        n_samples = len(msg.ranges)
        samples_per_rad = n_samples / (msg.angle_max - msg.angle_min)
        start_index = int(n_samples / 2 - (self.sector_width / 2 * samples_per_rad))
        stop_index =  int(n_samples / 2 + (self.sector_width / 2 * samples_per_rad))
        for i in range(start_index, stop_index):
            distance = msg.ranges[i]
            if not isnan(distance) and distance < min_distance and distance > msg.range_min and distance < msg.range_max:
                min_distance = distance

        self.obstacle_pub.publish(min_distance)
        
if __name__ == '__main__':
    try:
        DetectObstacleLaser()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    
