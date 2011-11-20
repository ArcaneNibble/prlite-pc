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


PKG = 'wubble2_robot'
NAME = 'tilt_neck_laser'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionClient
from wubble_actions.msg import *

def tilt(n=1):
    # Creates a goal to send to the action server.
    goal = HokuyoLaserTiltGoal()
    goal.tilt_cycles = n
    goal.amplitude = 1.5
    goal.offset = 0.0
    goal.duration = 7.0
    
    print 'Cycles: %d, amplitude: %f, offset: %f, duration: %f' % (goal.tilt_cycles, goal.amplitude, goal.offset, goal.duration)
    
    # Sends the goal to the action server.
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    
    # Return result
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        client = SimpleActionClient('hokuyo_laser_tilt_action', HokuyoLaserTiltAction)
        client.wait_for_server()
        
        print "Tilt 10 cycles"
        result = tilt(1000000);
        if result.success == False:
            print "Action failed"
        else:
            print "Result: " + str(result.tilt_position)
            
    except rospy.ROSInterruptException:
        pass

