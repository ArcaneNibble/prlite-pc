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
# Author: Antons Rebguns
#

NAME = 'hokuyo_laser_action'

import roslib; roslib.load_manifest('prlite_hokuyo')
import rospy
from actionlib import SimpleActionServer
from dynamixel_controllers.srv import SetSpeed
from std_msgs.msg import Float64
from pr2_controllers_msgs.msg import JointControllerState
from pr2_msgs.msg import LaserScannerSignal
# from ax12_controller_core.srv import SetSpeed
# from wubble_actions.msg import *
from laser_assembler.srv import *

class HokuyoLaserActionServer():
    def __init__(self):
        # Initialize constants
        self.error_threshold = 0.0175 # Report success if error reaches below threshold
        self.signal = 1
       
        # Initialize new node
        rospy.init_node(NAME, anonymous=True)
       
        controller_name = rospy.get_param('~controller')
       
        # Initialize publisher & subscriber for tilt
        self.laser_tilt = JointControllerState()
        self.laser_tilt_pub = rospy.Publisher(controller_name + '/command', Float64)
        #self.laser_signal_pub = rospy.Publisher('laser_scanner_signal', LaserScannerSignal)
        self.cloud = rospy.Publisher('neck_scan/shadow_filtered', LaserScannerSignal)
        self.joint_speed_srv = rospy.ServiceProxy(controller_name + '/set_speed', SetSpeed, persistent=True)
       
        rospy.Subscriber(controller_name + '/state', JointControllerState, self.process_tilt_state)
        rospy.wait_for_message(controller_name + '/state', JointControllerState)

        rospy.wait_for_service("assemble_scans")
       
        # Initialize tilt action server
        self.result = HokuyoLaserTiltResult()
        self.feedback = HokuyoLaserTiltFeedback()
        self.feedback.tilt_position = self.laser_tilt.process_value
        self.server = SimpleActionServer('hokuyo_laser_tilt_action', HokuyoLaserTiltAction, self.execute_callback)
       
        rospy.loginfo("%s: Ready to accept goals", NAME)

    def process_tilt_state(self, tilt_data):
        self.laser_tilt = tilt_data

    def reset_tilt_position(self, offset=0.0):
        self.laser_tilt_pub.publish(offset)
        rospy.sleep(0.5)

    def execute_callback(self, goal):
        r = rospy.Rate(100)
        self.joint_speed_srv(2.0)
        self.reset_tilt_position(goal.offset)
        delta = goal.amplitude - goal.offset
        target_speed = delta / goal.duration
        timeout_threshold = rospy.Duration(5.0) + rospy.Duration.from_sec(goal.duration)
        self.joint_speed_srv(target_speed)

       
        print "delta = %f, target_speed = %f" % (delta, target_speed)
       
        self.result.success = True
        self.result.tilt_position = self.laser_tilt.process_value
        rospy.loginfo("%s: Executing h laser tilt %s time(s)", NAME, goal.tilt_cycles)
       
        # Tilt laser goal.tilt_cycles amount of times.
        for i in range(1, goal.tilt_cycles + 1):
            self.feedback.progress = i
	    rospy.loginfo("b")
           
            # Issue 2 commands for each cycle
            for j in range(2):
                if j % 2 == 0:
                    target_tilt = goal.offset + goal.amplitude     # Upper tilt limit
                    self.signal = 0
                else:
                    target_tilt = goal.offset     # Lower tilt limit
                    self.signal = 1
                   
                rospy.loginfo("c")
                # Publish target command to controller
                self.laser_tilt_pub.publish(target_tilt)
                start_time = rospy.Time.now()
                current_time = start_time
               
                while abs(target_tilt - self.laser_tilt.process_value) > self.error_threshold:
                    rospy.loginfo("d")
                    delta = abs(target_tilt - self.laser_tilt.process_value)
                    time_left = goal.duration - (rospy.Time.now() - start_time).to_sec()
                    target_speed = delta / time_left
                    #self.joint_speed_srv(target_speed)
                   
                    # Cancel exe if another goal was received (i.e. preempt requested)
                    if self.server.is_preempt_requested():
                        rospy.loginfo("%s: Aborted: Action Preempted", NAME)
                        self.result.success = False
                        self.server.set_preempted()
                        return
                       
                    # Publish current head position as feedback
                    self.feedback.tilt_position = self.laser_tilt.process_value
                    self.server.publish_feedback(self.feedback)
                   
                    # Abort if timeout
                    current_time = rospy.Time.now()
                   
                    if (current_time - start_time > timeout_threshold):
                        rospy.loginfo("%s: Aborted: Action Timeout", NAME)
                        self.result.success = False
                        self.server.set_aborted()
                        return
                       
                    r.sleep()
                   
                # signal = LaserScannerSignal()
                # signal.header.stamp = current_time
                # signal.signal = self.signal
                # self.laser_signal_pub.publish(signal)
                try:
                    assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
                    resp = assemble_scans(start_time, rospy.get_rostime())
                    rospy.loginfo("Got cloud with %u points",len(resp.cloud.points));
                    self.cloud.publish(resp.cloud)
                    #rospy.sleep(0.5)
                except rospy.ServiceException, e:
                    rospy.loginfo("Service call failed: %s",e);
        if self.result.success:
            rospy.loginfo("%s: Goal Completed", NAME)
            self.result.tilt_position = self.laser_tilt.process_value
            self.server.set_succeeded(self.result)

if __name__ == '__main__':
    try:
        h = HokuyoLaserActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

