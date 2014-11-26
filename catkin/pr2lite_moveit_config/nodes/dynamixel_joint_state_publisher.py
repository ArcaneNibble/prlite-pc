#!/usr/bin/env python

"""
    dynamixel_joint_state_publisher.py - Version 1.0 2010-12-28
    
    Publish the dynamixel_controller joint states on the /joint_states topic
    
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

#import roslib; roslib.load_manifest('pi_dynamixel_controller')
import roslib; roslib.load_manifest('pr2lite_moveit_config')
import rospy

from sensor_msgs.msg import JointState as JointStatePR2
from dynamixel_msgs.msg import JointState as JointStateDynamixel

class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class JointStatePublisher():
    def __init__(self):
        rospy.init_node('dynamixel_joint_state_publisher', anonymous=True)
        
        dynamixel_namespace = rospy.get_namespace()
        rate = rospy.get_param('~rate', 10)
        r = rospy.Rate(rate)
        
        self.joints = list()
        
        # Get all parameter names
        parameters = rospy.get_param_names()
        
        for parameter in parameters:
            # Look for the parameters that name the joints.
            if parameter.find(dynamixel_namespace) != -1 and parameter.find("joint_name") != -1:
              self.joints.append(rospy.get_param(parameter))

        self.servos = list()
        self.controllers = list()
        self.joint_states = dict({})
        
        for joint in self.joints:
            # Remove "_joint" from the end of the joint name to get the controller names.
            servo = joint.split("_joint")[0]
            self.joint_states[joint] = JointStateMessage(joint, 0.0, 0.0, 0.0)
            #  ARD
            self.controllers.append(dynamixel_namespace + servo + "_controller")
            # self.controllers.append(joint)
            rospy.loginfo("Dynamixel Joint State Publisher " + joint)
                           
        # Start controller state subscribers
        [rospy.Subscriber(c + '/state', JointStateDynamixel, self.controller_state_handler) for c in self.controllers]
     
        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointStatePR2)
       
        rospy.loginfo("Starting Dynamixel Joint State Publisher at " + str(rate) + "Hz")
       
        while not rospy.is_shutdown():
            self.publish_joint_states()
            # With 23 servos, we go as fast as possible
            r.sleep()
           
    def controller_state_handler(self, msg):
        js = JointStateMessage(msg.name, msg.current_pos, msg.velocity, msg.load)
        self.joint_states[msg.name] = js
       
    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointStatePR2()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
       
        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            if  joint.name == 'right_elbow_flex_joint':
              # Shoulder mimic jnt to elbow joint compensation for gravity
              # code must match follow_controller_complex.py
              jnt_fudge = 0
              if joint.position > -.42 and joint.position < .38:
                jnt_fudge = -.02
                if joint.position > -.34 and joint.position < .26:
                  jnt_fudge = -.04
                  if joint.position > -.26 and joint.position < .14:
                    jnt_fudge = -.06
                    if joint.position > -.18 and joint.position < .02:
                      jnt_fudge = -.08

                # jnt_fudge = .15 * (.5 - joint.position)
                # jnt_fudge = -.08
              # jnt_fudge = -.1
              rospy.loginfo('elbow compensation = ' + str(jnt_fudge))
            else:
              jnt_fudge = 0
            jnt_fudge = 0
            fudge_value = rospy.get_param('~fudge_factor/' + joint.name + '/value', 0.0)
            j_pos = joint.position - fudge_value - jnt_fudge
            # rospy.loginfo("fudge " + str(joint.name) + ': ' + str(j_pos) + ' = ' + str(joint.position) + ' - ' + str(fudge_value))
            msg.position.append(j_pos)
            msg.velocity.append(joint.velocity)
            msg.effort.append(joint.effort)
           
        msg.header.stamp = rospy.Time.now()
        self.joint_states_pub.publish(msg)
        
if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

