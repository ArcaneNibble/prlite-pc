#!/usr/bin/env python

"""
    A node to generate fake laser data for experimenting with RViz and the Navigation stack.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

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
from sensor_msgs.msg import LaserScan
from math import sin
import random

class fake_laser():
    """ Laser scan sensor interface. """

    def __init__(self):
        rospy.init_node('fake_laser')

        # parameters
        self.rate = rospy.get_param("~rate", 1.0)
        self.frame_id = rospy.get_param("~frame", "base_laser")
        self.range_min = rospy.get_param("~range_min", 0.2)
        self.range_max = rospy.get_param("~range_max", 5.5)
        self.angle_min = rospy.get_param("~angle_min", -1.57)
        self.angle_max = rospy.get_param("~angle_max", 1.57)
        self.num_readings = rospy.get_param("~num_readings", 100)
        self.fixed_reading = rospy.get_param("~fixed_reading", 2.0)
        self.scan_time = rospy.get_param("~scan_time", 0.1)
        
        self.angle_increment = (self.angle_max - self.angle_min) / self.num_readings
        self.time_increment = self.scan_time / (self.num_readings)

        self.scanPub = rospy.Publisher('scan', LaserScan)

        rospy.loginfo("Started fake laser node publishing to /scan topic.")

        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            ranges = list()
            # Generate the fake data.
            for i in range(self.num_readings):
                ranges.append(self.fixed_reading * sin(i) * random.uniform(0, 1))
                
            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()     
            scan.header.frame_id = self.frame_id
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_increment
            scan.time_increment = self.time_increment
            scan.range_min = self.range_min
            scan.range_max = self.range_max
            scan.ranges = ranges    
            self.scanPub.publish(scan)
            r.sleep()

if __name__ == '__main__':
    try:
        fake_laser()
    except rospy.ROSInterruptException:
        pass
