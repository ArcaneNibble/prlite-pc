#!/usr/bin/env python

"""
dynamixel_joint_state_publisher.py - Version 1.0 2010-12-28
Publish the dynamixel_controller joint states on the /joint_states topic
Created for the Pi Robot Project: http://www.pirobot.org
Copyright (c) 2010 Patrick Goebel. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('pi_dynamixel_controller')
import rospy

#from sensor_msgs.msg import JointState as JointStatePR2
from sensor_msgs.msg import JointState
#from dynamixel_msgs.msg import JointState as JointStateDynamixel

class JointStatePR2Message():
    def __init__(self):
        #self.jspr2msg = JointStatePR2() 
        self.jspr2msg = JointState()
        self.jspr2msg.name = [ "left_shoulder_tilt_joint", "left_lin_act_cyl_joint", "left_upper_arm_hinge_joint","left_linear_actuator_joint", "right_shoulder_tilt_joint", "right_lin_act_cyl_joint", "right_upper_arm_hinge_joint","right_linear_actuator_joint", "torso_lift_joint"]
        self.jspr2msg.position = [0.0 for name in self.jspr2msg.name]
        self.jspr2msg.velocity = [0.0 for name in self.jspr2msg.name]
        #self.jspr2msg.effort = []
        rospy.loginfo("done init")
         
class Fake_LA_pos():
    def __init__(self):
        rospy.loginfo("calling JointStatePR2Message")
        self.msg = JointStatePR2Message()
        rospy.loginfo("calling init_node")
        rospy.init_node('fake_linact_pos', anonymous=True)
        # Start controller state subscribers
        rospy.loginfo("sub")
        rospy.Subscriber('/joint_states', JointState, self.arm_linact_state_handler)    
        rospy.loginfo("sub LA states")
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState)
        rospy.loginfo("pub LA states")
        while not rospy.is_shutdown():
            self.publish_LA_states()
            # With 23 servos, we go as fast as possible
            rospy.sleep(0.9)
          
    def arm_linact_state_handler(self, jsmsg):
        rospy.loginfo("arm linact state hander")
        do_copy = 0
        for joint_name in jsmsg.name:
          if joint_name == "left_linear_actuator_joint":
            do_copy = do_copy + 1
          if joint_name == "right_linear_actuator_joint":
            do_copy = do_copy + 1
          if joint_name == "torso_lift_joint":
            do_copy = do_copy + 1
        if do_copy >= 1:
          rospy.loginfo("copying joint state message")
          indx = 0
          for joint_name in jsmsg.name:
            indx2 = 0
            for joint_name2 in self.msg.jspr2msg.name:
              if joint_name == joint_name2:
                self.msg.jspr2msg.position[indx2] = jsmsg.position[indx]
                self.msg.jspr2msg.velocity[indx2] = jsmsg.velocity[indx]
              indx2 = indx2 + 1
            indx = indx + 1
     
    def publish_LA_states(self):
        rospy.loginfo("pub LA states")
        self.msg.jspr2msg.header.stamp = rospy.Time.now()
        rospy.loginfo("pub LA states 2")
        #self.msg.header.stamp = rospy.Time.now()
        self.joint_states_pub.publish(self.msg.jspr2msg)
        
if __name__ == '__main__':
    try:
        s = Fake_LA_pos()
        rospy.spin()
    except rospy.ROSInterruptException: pass
