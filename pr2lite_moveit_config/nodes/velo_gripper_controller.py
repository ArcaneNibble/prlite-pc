#!/usr/bin/env python

"""
parallel_gripper_controller.py - controls a gripper built of two servos
Copyright (c) 2011 Vanadium Labs LLC. All right reserved.

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
from pr2_controllers_msgs.msg import(Pr2GripperCommandGoal, Pr2GripperCommand, Pr2GripperCommandAction)
from std_msgs.msg import Float64
from math import asin

class VeloGripperController:
    """ A simple controller that operates two opposing servos to open/close to a particular size opening. """
    def __init__(self):
        rospy.init_node("velo_gripper_controller")
        self.speed = rospy.get_param("~speed", 0.5)
        self.torque = rospy.get_param("~torque", 500)
        self.min_opening = rospy.get_param("~min", 0.0)
        self.max_opening = rospy.get_param("~max", 2.0)

        self.controller = 'velo_controller'

        if self.torque > 1023:
           self.torque = 1023
        if self.torque < 0:
           self.torque = 0
        #torque_service = '/' + self.controller + '/torque_enable'
        torque_service = '/' + self.controller + '/set_torque_limit'
        # torque_service = '/' + self.controller + '/set_compliance_punch' # min torque
        # torque_service = '/' + self.controller + '/set_compliance_limit' # max torque
        # limit, punch 0-1023
        rospy.wait_for_service(torque_service)
        #self.my_torque_service = rospy.ServiceProxy
        rospy.wait_for_service(torque_service)
        #self.my_torque_service = rospy.ServiceProxy (torque_service, TorqueEnable)
        self.my_torque_service = rospy.ServiceProxy (torque_service, SetTorqueLimit)
        speed_service = '/' + self.controller + '/set_speed'
        rospy.wait_for_service(speed_service)
        # speed_services.append(rospy.ServiceProxy (speed_service, SetSpeed))
        self.my_speed_service = rospy.ServiceProxy (speed_service, SetSpeed)
        # position in radians
        pub_topic = '/' + self.controller + '/command'
        self.velo_pub = rospy.Publisher(pub_topic, Float64)

        # subscribe to command and then spin
        # Pr2GripperCommand(position, max_effort)
        self.server = actionlib.SimpleActionServer(side_c +"_gripper_controller/gripper_action", Pr2GripperCommandAction, execute_cb=self.commandCb, auto_start=False)
        self.server.start()

    def commandCb(self, msg):
        """ Take an input command of width to open gripper.  """
        # check limits
        if msg.command.position > self.max_opening or msg.command.position < self.min_opening:
            rospy.logerr("Command exceeds limits.")
            return
        print "Gripper angle ", angle

        # AX12 Position 0 (min) to 300 degrees (max) = 5.236 radians
        # -2.618 radians to 2.618
        # velopos = 2.6 * (msg.command.position)
        velopos = 1 * (msg.command.position) - .9

        # publish msgs
        velomsg = Float64(velopos)
        self.velo_pub.publish(velomsg)
        rospy.loginfo(self.side + " Gripper %f" % (velopos))
        self.server.set_succeeded()
if __name__=="__main__":
    try:
        VeloGripperController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Hasta la Vista...")
