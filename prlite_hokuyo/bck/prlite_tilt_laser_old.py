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


PKG = 'prlite_hokuyo'
NAME = 'prlite_tilt_laser'

import roslib; roslib.load_manifest(PKG)

import rospy
from actionlib import SimpleActionClient
from wubble_actions.msg import *
from laser_assembler.srv import *
from pr2_msgs.msg import LaserScannerSignal
from sensor_msgs.msg import *

# from wubble_actions.msg import HokuyoLaserTiltAction
# from wubble_actions.msg import HokuyoLaserTiltGoal
# from prlite_hokuyo.msg import *
import tf

start_tm = rospy.Time(0,0)

def callback(signal):
   global start_tm
   rospy.loginfo("signal callback")
   try:
      rospy.wait_for_service("assemble_scans")
      cloudpub = rospy.Publisher('cloudpub', PointCloud)
      assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
      #curtm = rospy.get_rostime()
      # resp = assemble_scans(signal.header.stamp, curtm)
      resp = assemble_scans(start_tm, signal.header.stamp)
      rospy.loginfo("start %s cur %s", start_tm, signal.header.stamp)
      start_tm = signal.header.stamp
      rospy.loginfo("Got cloud with %u points" % len(resp.cloud.points))
      cloudpub.publish(resp.cloud)
   except rospy.ServiceException, e:
      rospy.loginfo("Service call failed: %s",e);

def handle_laser_tilt(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

def tilt(n=100):
    # Creates a goal to send to the action server.
    goal = HokuyoLaserTiltGoal()
    goal.tilt_cycles = n
    # goal.amplitude = 1.5
    goal.amplitude = 1.5
    # goal.offset = 0.0
    goal.offset = -0.90
    goal.duration = 7.0
    
    print 'Cycles: %d, amplitude: %f, offset: %f, duration: %f' % (goal.tilt_cycles, goal.amplitude, goal.offset, goal.duration)
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    
    laser_signal_sub = rospy.Subscriber('laser_scanner_signal', LaserScannerSignal, callback)
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    laser_tilt.feedback = HokuyoLaserTiltFeedback()
    rospy.Subscriber('laser_tilt',
                     turtlesim.msg.Pose,
                     handle_laser_tilt,
                     turtlename)
    
    # Return result
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        client = SimpleActionClient('hokuyo_laser_tilt_action', HokuyoLaserTiltAction)
        client.wait_for_server()
        
        result = tilt(5);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: " + str(result.tilt_position)
            
    except rospy.ROSInterruptException:
        pass

