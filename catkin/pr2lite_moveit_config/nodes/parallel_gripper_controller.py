#!/usr/bin/env python

"""
  parallel_gripper_controller.py - controls a gripper built of two servos
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; 
#roslib.load_manifest('arbotix_controllers')
roslib.load_manifest('pr2lite_moveit_config')
import rospy
import actionlib
from pr2_controllers_msgs.msg import (Pr2GripperCommandGoal, Pr2GripperCommand,
                                      Pr2GripperCommandAction)
import thread


from pr2_controllers_msgs.msg import (Pr2GripperCommandGoal, Pr2GripperCommand,
                                      Pr2GripperCommandAction)
from std_msgs.msg import Float64
from math import asin

class ParallelGripperController:
    """ A simple controller that operates two opposing servos to
        open/close to a particular size opening. """
    def __init__(self):
        rospy.init_node("parallel_gripper_controller")

        # trapezoid model: base width connecting each gripper's rotation point
            #              + length of gripper fingers to computation point
            #              = compute angles based on a desired width at comp. point
        self.pad_width = rospy.get_param("~pad_width", 0.01)
        self.finger_length = rospy.get_param("~finger_length", 0.1016)
        self.min_opening = rospy.get_param("~min", 0.0)
        self.max_opening = rospy.get_param("~max", 2*self.finger_length)

        self.side = rospy.get_param("~side", "left")
        self.center_l = rospy.get_param("~center_left", 0.0)
        self.center_r = rospy.get_param("~center_right", 0.0)
        self.invert_l = rospy.get_param("~invert_left", False)
        self.invert_r = rospy.get_param("~invert_right", False)

        # publishers
        self.l_pub = rospy.Publisher(self.side + "_gripper_left_controller/command", Float64)
        self.r_pub = rospy.Publisher(self.side + "_gripper_right_controller/command", Float64)

        # subscribe to command and then spin
        # Pr2GripperCommand(position, max_effort)
        # rospy.Subscriber("~command", Float64, self.commandCb)
        if self.side == "right":
            side_c = "r"
        if self.side == "left":
            side_c = "l"
        # rospy.Subscriber(side_c+"_gripper_controller/gripper_action", Pr2GripperCommand, self.commandCb)
        #self.server = actionlib.SimpleActionServer(side_c+"_gripper_controller/gripper_action", Pr2GripperCommandAction, execute_cb=self.commandCb, auto_start=False)
        self.server = actionlib.SimpleActionServer("gripper_controller/gripper_action", Pr2GripperCommandAction, execute_cb=self.commandCb, auto_start=False)
        self.server.start()
        

    def commandCb(self, msg):
        """ Take an input command of width to open gripper. """
        # check limits
        if msg.command.position > self.max_opening or msg.command.position < self.min_opening:
            rospy.logerr("Command exceeds limits.")
            return
        # compute angles
        angle = asin((msg.command.position - self.pad_width)/(2*self.finger_length))
        print "Gripper angle ", angle
        if self.invert_l:
            l = -angle + self.center_l
        else:
            l = angle + self.center_l
        if self.invert_r:
            r = -angle + self.center_r
        else:
            r = angle + self.center_r
        # publish msgs
        lmsg = Float64(l)
        rmsg = Float64(r)
        self.l_pub.publish(lmsg)
        self.r_pub.publish(rmsg)
        rospy.loginfo(self.side + " Gripper %f %f" % (l, r))
        self.server.set_succeeded()
        #vs. self.server.set_aborted(result)

if __name__=="__main__": 
    try:
        ParallelGripperController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hasta la Vista...")
