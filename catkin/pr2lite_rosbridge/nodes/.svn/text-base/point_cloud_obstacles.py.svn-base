#!/usr/bin/env python

"""
    point_cloud_obstacles.py - Version 1.0 2012-06-01
    
    Publish locations of nearby obstacles based on point cloud data
    
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
from sensor_msgs.msg import PointCloud2, LaserScan
import point_cloud2

class PointCloudObstacles():
    def __init__(self):
        rospy.init_node("point_cloud_obstacles")
        
        
        # The dimensions (in meters) of the box in which we will search
        # for obstacles. These are given in camera coordinates
        # where x is left/right,y is up/down and z is depth (forward/backward)
        self.min_x = rospy.get_param("~min_x", -1.0)
        self.max_x = rospy.get_param("~max_x", 1.0)
        self.min_y = rospy.get_param("~min_y", -0.3)
        self.max_y = rospy.get_param("~max_y", 1.0)
        self.max_z = rospy.get_param("~max_z", 2.0)

        rospy.Subscriber('point_cloud', PointCloud2, self.update_obstacles)
        
        # Wait for the pointcloud topic to become available
        rospy.wait_for_message('point_cloud', PointCloud2)
        
    def update_obstacles(self, msg):
        # Initialize the centroid coordinates point count
        x = y = z = n = 0
        
        # Read in the x, y, z coordinates of all points in the cloud
        for point in point_cloud2.read_points(msg, skip_nans=True):
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            
            # Keep only those points within our designated boundaries and sum them up
            if -pt_y > self.min_y and -pt_y < self.max_y and  pt_x < self.max_x and pt_x > self.min_x and pt_z < self.max_z:
                x += pt_x
                y += pt_y
                z += pt_z
                n += 1
        
        # If we have points, compute the centroid coordinates
        if n:    
            x /= n 
            y /= n 
            z /= n
                        
            # Check our movement thresholds
            if (abs(z - self.goal_z) > self.z_threshold) or (abs(x) > self.x_threshold):     
                # Compute the linear and angular components of the movement
                linear_speed = (z - self.goal_z) * self.z_scale
                angular_speed = -x * self.x_scale
                
                # Make sure we meet our min/max specifications
                linear_speed = copysign(max(self.min_linear_speed, 
                                            min(self.max_linear_speed, abs(linear_speed))), linear_speed)
                angular_speed = copysign(max(self.min_angular_speed, 
                                             min(self.max_angular_speed, abs(angular_speed))), angular_speed)
    
                move_cmd.linear.x = linear_speed
                move_cmd.angular.z = angular_speed
                        
        # Publish the movement command
        self.cmd_vel_pub.publish(move_cmd)

        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)     
                   
if __name__ == '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")
