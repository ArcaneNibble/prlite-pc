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

    def compute_elbow_fudge(self, elbow, shoulder):
          jnt_fudge = 0
          # Shoulder mimic jnt to elbow joint compensation for gravity
          elbow_gravity_factor = .1
          shoulder_elbow_tilt_factor = 0
          # parameters
          straight_elbow_threshold = .6
          shoulder_threshold = .5
          # from pr2lite urdf: shoulder lower="0.42" upper="1.2"
          shoulder_min = .42   # arm up
          shoulder_max   = 1.2 # arm down
          # 2.4 on model is 2.7 on servo
          bent_elbow_threshold = 2.1 

          if elbow > (-1 * straight_elbow_threshold) and elbow <= 0:
              jnt_fudge = -1 * elbow_gravity_factor * (1 + elbow / straight_elbow_threshold)
              # add back fudge based on shoulder tilt
              jnt_fudge =  jnt_fudge + shoulder_elbow_tilt_factor * (shoulder - shoulder_min)*(1 + elbow / straight_elbow_threshold)
          elif elbow > 0 and elbow < straight_elbow_threshold:
              jnt_fudge = -.1 * elbow_gravity_factor * (1 - elbow / straight_elbow_threshold)
              jnt_fudge =  jnt_fudge + shoulder_elbow_tilt_factor * (shoulder - shoulder_min) / (shoulder_max - shoulder_min)
          elif elbow > bent_elbow_threshold:
              # 2.4 on model is 2.7 on servo
              jnt_fudge =  (bent_elbow_threshold - elbow) / 2
          return jnt_fudge

    # we fudge the position to match reality during the follow traj execution
    # and a matching fudge in the dynamixel joint position publisher.
    # The joint position is monitored by moveit.
    def compute_jnt_fudge(self, joint_name, joint_pos):
          self.jnt_fudge = 0
          if joint_name == 'right_elbow_flex_joint':
            shoulder_tilt = self.shoulder_state("right_shoulder_tilt_joint")
            # right_shoulder_tilt is [1] with right arm conroller
            self.jnt_fudge = self.compute_elbow_fudge(joint_pos, shoulder_tilt)
          elif joint_name == 'left_elbow_flex_joint':
            shoulder_tilt = self.shoulder_state("left_shoulder_tilt_joint")
            # left_shoulder_tilt is [1] with left arm controller
            self.jnt_fudge = self.compute_elbow_fudge(joint_pos, shoulder_tilt)

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
            # if joint.name == 'right_elbow_flex_joint':
            #   rospy.loginfo("dyno fudge " + str(joint.name) + ': ' + str(j_pos) + ' = ' + str(joint.position) + ' - ' + str(fudge_value) + ' - ' + str(self.jnt_fudge))
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

