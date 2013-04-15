#!/usr/bin/env python

"""
    pi_joy.py - Version 1.0 2012-09-24
    
    Joystick node for the Logitech Wireless Rumblepad and Pi Robot
    
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
from math import radians, pow

from dynamixel_controllers.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Joy, JointState

class PiJoystick():
    def __init__(self):
        rospy.init_node('pi_joystick', anonymous=False)
        
        rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(rate)

        self.use_servos = rospy.get_param("~use_servos", True)
        self.left_handed = rospy.get_param("~left_handed", False)
        
        # The namespace and joints parameter needs to be set by the servo controller
        namespace = rospy.get_namespace()
        self.joints = rospy.get_param(namespace + '/joints', '')
    
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.15)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.75)
        self.speed_increment = rospy.get_param("~speed_increment", 0.025)
        
        # This parameters makes it easier to drive the robot straight ahead
        self.angular_deadzone = rospy.get_param("~angular_deadzone", 0.5)
        
        self.default_joint_speed = rospy.get_param("~default_joint_speed", 0.3)
        self.default_head_pan_speed = rospy.get_param("~default_head_pan_speed", 0.45)
        self.default_head_tilt_speed = rospy.get_param("~default_head_tilt_speed", 0.3)
        
        self.max_head_pan_position = rospy.get_param("~max_head_pan_position", radians(160))
        self.max_head_tilt_position = rospy.get_param("~max_head_tilt_position", radians(90))
        
        # This parameter makes it easier to pan the head without also tilting it
        self.tilt_deadzone = rospy.get_param("~tilt_deadzone", 0.2)

        if self.left_handed:
            self.base_axis = 0
            self.servo_axis = 2
        else:
            self.base_axis = 2
            self.servo_axis = 0

        # A flag to prevent the operator from running into things
        self.obstacle_detected = False
        
        # A flag to turn off publishing when the robot is already stopped so that
        # other input methods can still work.
        self.robot_is_stopped = False
        
        if self.use_servos:
            self.init_servos()
        
            self.servo_speed['head_pan_joint'](self.default_head_pan_speed)
            self.servo_speed['head_tilt_joint'](self.default_head_tilt_speed)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)
        self.cmd_vel = Twist()
        
        rospy.Subscriber("joy", Joy, self.process_joystick_input)
        rospy.Subscriber("base_laser_obstacle", Bool, self.obstacle_laser)
        
        rospy.loginfo("Ready")
        
        while not rospy.is_shutdown():
            if self.obstacle_detected and self.cmd_vel.linear.x > 0:
                self.cmd_vel.linear.x = 0

            if not self.robot_is_stopped:
                self.cmd_vel_pub.publish(self.cmd_vel)
                
            if self.cmd_vel.linear.x == 0 and self.cmd_vel.angular.z == 0:
                self.robot_is_stopped = True
            else:
                self.robot_is_stopped = False
                
            r.sleep()
        
    def process_joystick_input(self, msg):
        # Adjusting the max speed
        self.max_linear_speed += msg.axes[5] * self.speed_increment
        self.max_angular_speed += msg.axes[5] * self.speed_increment

        self.max_linear_speed = max(0, self.max_linear_speed)
        self.max_angular_speed = max(0, self.max_angular_speed)
        
        # First move the base
        vz = msg.axes[self.base_axis]
        vx = msg.axes[self.base_axis + 1]
        
        if abs(vz) < self.angular_deadzone:
            vz = 0
        
        vx = sign(vx) * (pow(2, abs(vx)) - 1) * self.max_linear_speed
        vz = sign(vz) * (pow(2, abs(vz)) - 1) * self.max_angular_speed
            
        self.cmd_vel.linear.x = vx
        self.cmd_vel.angular.z = vz
        
        if self.use_servos:
            # Now move the servos
            pan = msg.axes[self.servo_axis]
            tilt = msg.axes[self.servo_axis + 1]
            
            #pan = (pow(2, 4 * pan) - 1) * self.max_head_pan_position
            #tilt = (pow(2, tilt) - 1) * self.max_head_tilt_position

            pan = pan * self.max_head_pan_position
            tilt = tilt * self.max_head_tilt_position
            
            if abs(tilt) < self.tilt_deadzone:
                tilt = 0
            
            self.servo_position['head_pan_joint'].publish(pan)
            self.servo_position['head_tilt_joint'].publish(tilt)
        
        
    def obstacle_laser(self, msg):
        self.obstacle_detected = msg.data
        
    def init_servos(self):
        # Create dictionaries to hold the speed, position and torque controllers
        self.servo_speed = dict()
        self.servo_position = dict()
        self.torque_enable = dict()

        # Connect to the set_speed and torque_enable services and
        # define a position publisher for each servo
        rospy.loginfo("Waiting for joint controllers services...")
        
        for controller in sorted(self.joints):
            # The position controllers
            self.servo_position[controller] = rospy.Publisher('/' + controller + '/command', Float64)
                
            try:
                # The set_speed services
                set_speed_service = '/' + controller + '/set_speed'
                rospy.loginfo(set_speed_service)

                rospy.wait_for_service(set_speed_service)  
                self.servo_speed[controller] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent=True)
                
                # Initialize the servo speed to the default_joint_speed
                self.servo_speed[controller](self.default_joint_speed)
                    
                # Torque enable/disable control for each servo
                torque_service = '/' + controller + '/torque_enable'
                rospy.wait_for_service(torque_service) 
                self.torque_enable[controller] = rospy.ServiceProxy(torque_service, TorqueEnable)
                    
                # Start each servo in the disabled state so we can move them by hand
                self.torque_enable[controller](False)
            except:
                rospy.logerr("Can't contact servo services!")
        
def sign(x):
    if x < 0:
        return -1
    if x > 0: 
        return 1
    
    return 0
        
if __name__ == '__main__':
    try:
        PiJoystick()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    
    
