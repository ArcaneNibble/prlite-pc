#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Anh Tran

# Based on the wubble_head_action.py script by Anh Tran. Modifications made by Patrick Goebel
# for the Pi Robot Project.

import roslib; roslib.load_manifest('pr2lite_moveit_config')
import actionlib
import rospy, time
import tf2_ros
from std_msgs.msg import Float64
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import PointStamped
from pr2lite_moveit_config.srv import ReturnJointStates

import math

class PointHeadNode():
    def __init__(self):
        # Initialize new node
        # rospy.init_node('point_head_node', anonymous=True)
       

        # servo_namespace = rospy.get_namespace()
        servo_namespace = ""
        self.head_pan_controller_topic = rospy.get_param('head_pan_controller_topic', servo_namespace + 'head_pan_controller/command')
        self.head_tilt_controller_topic = rospy.get_param('head_tilt_controller_topic', servo_namespace + 'head_tilt_controller/command')
        self.head_pan_speed_param = rospy.get_param('head_pan_speed_param', 'head_pan_controller/max_speed')
        self.head_tilt_speed_param = rospy.get_param('head_tilt_speed_param', 'head_tilt_controller/max_speed')

        rate = rospy.get_param('~rate', 1)
        r = rospy.Rate(rate)
       
        rospy.set_param(self.head_pan_speed_param, 50)
        rospy.set_param(self.head_tilt_speed_param, 50)
       
        # Initialize the target point
        self.target_point = PointHeadGoal()
        self.last_target_point = PointHeadGoal()
       

        self.base_frame = 'base_link'
        # Initialize publisher for the pan servo
        self.head_pan_frame = 'head_pan_link'
        # self.head_pan_frame = 'kinect_depth_optical_frame'
        # self.head_pan_frame = 'kinect_link'
        self.head_pan_pub = rospy.Publisher(self.head_pan_controller_topic, Float64)
       
        # Initialize publisher for the tilt servo
        self.head_tilt_frame = 'head_pan_link'
        self.head_tilt_pub = rospy.Publisher(self.head_tilt_controller_topic, Float64)

        # Initialize tf listener
        self.buf = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.buf)
       
        # Reset the head position to neutral
        rospy.sleep(1)
        self.reset_head_position()
        # Subscribe to the target_point topic
        self.server = actionlib.SimpleActionServer('/head_traj_controller/point_head_action', PointHeadAction, execute_cb=self.update_target_point, auto_start=False)
        self.server.start()


    # http://www.ros.org/wiki/pr2_controllers/Tutorials/Getting%20the%20current%20joint%20angles
    def get_head_states(self):
      joint_names = ['head_pan_joint', 'head_tilt_joint']
      rospy.wait_for_service("return_joint_states")
      try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
      except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
      for (ind, joint_name) in enumerate(joint_names):
        if joint_name == "head_pan_joint":
          head_pan_pos = resp.position[ind]
        elif joint_name == "head_tilt_joint":
          head_tilt_pos = resp.position[ind]
        elif(not resp.found[ind]):
          print "joint %s not found!"%joint_name
      return (head_pan_pos, head_tilt_pos)

    def update_target_point(self, msg):
        self.target_point = msg
        rospy.loginfo("got message");
        if self.target_point == self.last_target_point:
            rospy.loginfo("continue");
            self.server.set_succeeded()
            return
        rospy.loginfo("transtargpnt");
        target_angles = self.transform_target_point(self.target_point.target)
        if target_angles[0] == -1000 and target_angles[1] == -1000:
            rospy.loginfo("tf transform failed");
            self.server.set_succeeded()
            return
        self.head_tilt_pub.publish(target_angles[1])
        rospy.loginfo("publishing");
        self.head_pan_pub.publish(target_angles[0])
        self.head_tilt_pub.publish(target_angles[1])

        self.last_target_point = self.target_point
        rospy.loginfo("Setting Target Point:\n" + str(self.target_point))
        self.server.set_succeeded()

    def reset_head_position(self):
        self.head_pan_pub.publish(0.0)
        self.head_tilt_pub.publish(0.0)
        rospy.sleep(1)

    def transform_target_point(self, target):
        # based on pr2_head_action.cpp
        # http://docs.ros.org/diamondback/api/pr2_head_action/html/pr2__head__action_8cpp_source.html
        pan_ref_frame = self.head_pan_frame
        tilt_ref_frame = self.head_tilt_frame
       
        # Wait for tf info (time-out in 5 seconds)
        rospy.loginfo( "pan tf: ")
        try:
          pan_trans = self.buf.lookup_transform(pan_ref_frame, self.base_frame, 
                                      rospy.Time(0), timeout=rospy.Duration(4))
          print pan_trans
          rospy.loginfo( pan_trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
          print e 
          return [-1000, -1000]

        rospy.loginfo( "tilt tf: ")
        try:
          tilt_trans = self.buf.lookup_transform(tilt_ref_frame, 
                 self.base_frame, rospy.Time(0), timeout=rospy.Duration(4))
          print tilt_trans
          rospy.loginfo( tilt_trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
          print e
          return [-1000, -1000]

        rospy.loginfo( "target tf: ")
        try:
          target_trans = self.buf.lookup_transform(self.base_frame, 
             target.header.frame_id, rospy.Time(0), timeout=rospy.Duration(4))
          print target_trans
          rospy.loginfo( target_trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
          print e
          return [-1000, -1000]

        rospy.loginfo( "ref frames: " + pan_ref_frame + ", " + tilt_ref_frame)
        rospy.loginfo( "target point " + str(target.header.frame_id) + " x " + str( target.point.x) + " y " + str(target.point.y) + " z " + str(target.point.z))

        # atan2 (opposite, adjacent)
        # Transform target point to pan reference frame & retrieve the pan angle
        pan_angle = math.atan2( 
                         target_trans.transform.translation.y + target.point.y, 
                         target_trans.transform.translation.x + target.point.x)
        rospy.loginfo( "pan angle " + str( pan_angle))

        pan_to_tilt_len = .09
        tilt_angle = -1 * math.atan2( 
	   (target_trans.transform.translation.z + target.point.z + tilt_trans.transform.translation.z - pan_to_tilt_len),
	   math.sqrt(math.pow(target.point.x, 2) + math.pow(target.point.y, 2)))
        rospy.loginfo( "tilt angle " + str( tilt_angle))
        return [pan_angle, tilt_angle]


if __name__ == '__main__':
    try:
        rospy.init_node("point_head")
        point_head = PointHeadNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


