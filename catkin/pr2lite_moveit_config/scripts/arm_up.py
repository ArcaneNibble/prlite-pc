#!/usr/bin/env python

"""
follow_controller.py - controller for a kinematic chain
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


import roslib; roslib.load_manifest('pr2lite_moveit_config')
import actionlib
import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from diagnostic_msgs.msg import *
from std_msgs.msg import Float64
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetSpeed
from collections import deque

#!/usr/bin/env python

# Quick example code to show how to execute a simple trajectory on the PR-2 armin python
# Nick Eckenstein

import sys
import string
import actionlib
import pr2_controllers_msgs.msg
import trajectory_msgs.msg

from sensor_msgs.msg import JointState
from actionlib import simple_action_client
# from pr2_controllers_msgs.msg import JointTrajectoryAction
# from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64
from std_msgs.msg import Int64 
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal

class CKBotReporter():
    def __init__(self,*arg,**kw):
        #initializing ROS
        print 'init minipr2'
        rospy.init_node('minipr2')

        # Creates joint trajectory action client and waits for the server
        print 'Waiting for Joint Server'
        #l_client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
        #FollowJointTrajectoryAction,
        r_client = actionlib.SimpleActionClient('arm_controllerR/follow_joint_trajectory', FollowJointTrajectoryAction)
        #l_client.wait_for_server()
        r_client.wait_for_server()

        # Creates the goal object to pass to the server
        # goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        # goal = control_msgs.FollowJointTrajectoryActionResult()
        # goal = FollowJointTrajectoryAction()
        goal = FollowJointTrajectoryGoal()
        
# arm_controllerR: {type: follow_controller, joints: [right_shoulder_pan_joint, right_shoulder_tilt_joint, right_elbow_pan_joint, right_elbow_flex_joint, right_wrist_flex_joint, right_wrist_roll_joint, torso_lift_joint], action_name: arm_controllerR/follow_joint_trajectory, onboard: False },

        # Populates trajectory with joint names.
        goal.trajectory.joint_names.append("right_shoulder_pan_joint")
        goal.trajectory.joint_names.append("right_shoulder_tilt_joint")
        goal.trajectory.joint_names.append("right_elbow_flex_joint")
        goal.trajectory.joint_names.append("right_elbow_pan_joint")
        goal.trajectory.joint_names.append("right_wrist_flex_joint")
        goal.trajectory.joint_names.append("right_wrist_roll_joint")
        goal.trajectory.joint_names.append("torso_lift_joint")
        
        # First trajectory point
        # Positions
        ind = 0
        #print goal.trajectory.points
        point1 = trajectory_msgs.msg.JointTrajectoryPoint()
        goal.trajectory.points = [point1]
        point1.positions = [0,0.5,0,0,0,0,0.295]
        point1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        
        #To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = rospy.Duration(1.0)
        goal.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(1.0)        
        r_client.send_goal(goal)
        #rospy.spin()
        self.goal = goal

        #rospy.spin()



if __name__=='__main__':
    try:
        #Creates ckbot reporter
        rep = CKBotReporter()
        # rep.run()
    except rospy.ROSInterruptException: pass
    

