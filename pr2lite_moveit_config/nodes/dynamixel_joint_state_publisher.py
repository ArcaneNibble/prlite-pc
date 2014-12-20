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
from pr2lite_moveit_config.srv import ReturnJointStates


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
        self.jnt_fudge = 0
        
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
       

    def shoulder_state(self, shoulder_joint):
        rospy.wait_for_service("return_joint_states")
        joint_names = [shoulder_joint]
        error = 1
        while (error == 1):
          try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
          except rospy.ServiceException, e:
            print "error when calling return_joint_states: %s"%e
            return -1000.0
          for (ind, joint_name) in enumerate(joint_names):
            if(not resp.found[ind]):
                # print "joint %s not found!"%joint_name
                # rospy.sleep(.1)
                return -1000.0
            else:
                 error = 0
        # return (resp.position, resp.velocity, resp.effort)
        return resp.position[ind]

    # we fudge the position to match reality during the follow traj execution
    # and a matching fudge in the joint position publisher.
    # The joint position is monitored by moveit.
    def compute_jnt_fudge(self, joint_name, joint_pos):
          self.jnt_fudge = 0
          # we either fudge here or in the follow_trajectory publisher
          # this is better so moveit and rviz also looks good
          if joint_name == 'right_elbow_flex_joint':
            # Shoulder mimic jnt to elbow joint compensation for gravity
            shoulder_tilt = self.shoulder_state("right_shoulder_tilt_joint")
            if joint_pos > -.6 and joint_pos <= 0:
              self.jnt_fudge = -.3 * (1 + joint_pos / .6)
              # if shoulder_tilt > .4 and shoulder_tilt < 1.2:
              self.jnt_fudge =  self.jnt_fudge + .5 * (shoulder_tilt - .5) * (1 + joint_pos / .6)
              # else:
              #   print "bad shoulder joint value"
            elif joint_pos > 0 and joint_pos < .6:
              self.jnt_fudge = -.3 * (1 - joint_pos / .6)
              # if shoulder_tilt > .4 and shoulder_tilt < 1.2:
              self.jnt_fudge =  self.jnt_fudge + .5 * (shoulder_tilt - .5) * (1 - joint_pos / .6)
              # else:
              #   print "bad shoulder joint value"
            elif joint_pos > 2.1:
              # 2.4 on model is 2.7 on servo
              self.jnt_fudge = (2.1 - joint_pos) / 2

    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointStatePR2()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
       
        for joint in self.joint_states.values():
            msg.name.append(joint.name)

            fudge_value = rospy.get_param('~fudge_factor/' + joint.name + '/value', 0.0)
            jp = joint.position - fudge_value
            self.compute_jnt_fudge(joint.name, jp)
            j_pos = joint.position - fudge_value - self.jnt_fudge
            if joint.name == 'right_elbow_flex_joint':
              rospy.loginfo("dyno fudge " + str(joint.name) + ': ' + str(j_pos) + ' = ' + str(joint.position) + ' - ' + str(fudge_value) + ' - ' + str(self.jnt_fudge))
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

