#!/usr/bin/env python

"""
  reset_arm_server.py - a simple service to reset the arm
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holders nor the names of its
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

import roslib; roslib.load_manifest('pr2lite_arm_navigation')
import rospy, actionlib
from control_msgs.msg import *
from simple_arm_actions.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_msgs.msg import *


class ResetArmServer:

    def __init__(self):
        rospy.init_node("reset_arm_server")
       
        dynamixel_namespace = rospy.get_namespace()
#        if dynamixel_namespace == '/':
#            dynamixel_namespace = rospy.get_param('~dynamixel_namespace', '/dynamixel_controller')
#        self.joints = rospy.get_param(dynamixel_namespace + "/arm_joints")
        self.joints = rospy.get_param('arm_joints')
 
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client.wait_for_server()

        self.server = actionlib.SimpleActionServer("reset_arm", ResetArmAction, execute_cb=self.actionCb, auto_start=False)
        self.server.start()

        rospy.spin()

    def actionCb(self, req):
        goal = FollowJointTrajectoryGoal()
        msg = JointTrajectory()
        msg.joint_names = self.joints
        msg.points = list()
        point = JointTrajectoryPoint()
        point.positions = [ 0.0 for servo in msg.joint_names ]
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(2.0)
        msg.points.append(point)
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.01)
        goal.trajectory = msg

        self.client.send_goal(goal)
        self.client.wait_for_result()
        self.server.set_succeeded( ResetArmResult() )  


if __name__ == '__main__':
    try:
        ResetArmServer()
    except rospy.ROSInterruptException:
        rospy.loginfo("And that's all folks...")

