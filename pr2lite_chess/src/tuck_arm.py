#!/usr/bin/env python

""" 
  Copyright (c) 2011 Michael E. Ferguson. All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import roslib; roslib.load_manifest('pr2lite_chess')
import rospy
import actionlib
import sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import *


left_servos = [ 'left_shoulder_pan_joint', 'left_shoulder_tilt_joint', 'left_upper_arm_hinge_joint', 'left_elbow_pan_joint', 'left_elbow_flex_joint', 'left_wrist_flex_joint', 'left_wrist_roll_joint']
left_to_side = [-1.6157930965728755,0.35822709988129486, -0.35822709988129486, 0, -1.3294500161692546, 0, 0]
left_forward = [-0.5368932757599745,0.35822709988129486, -0.35822709988129486, 0, -1.3294500161692546, 0, 0]
left_tucked = [-1.6157930965728755, 0.9481697047772568, -0.9481697047772568, 0, -1.3396765547496274, 0,0]
left_startIKpos = [ -0.04601942363656924, 0.35822709988129486, -0.35822709988129486, 1.5851134808151626,  0.02045307717010969, -1.4675082870772636, 0.5675728915176873]

servos = ['right_shoulder_pan_joint', 'right_shoulder_tilt_joint', 'right_upper_arm_hinge_joint', 'right_elbow_pan_joint', 'right_elbow_flex_joint', 'right_wrist_flex_joint', 'right_wrist_roll_joint']
to_side = [1.6157930965728755,0.35822709988129486, -0.35822709988129486, 0, -1.3294500161692546, 0, 0]
forward = [0.5368932757599745,0.35822709988129486, -0.35822709988129486, 0, -1.35, 0, 0]
tucked = [1.6157930965728755, 0.9481697047772568, -0.9481697047772568, 0, -1.3396765547496274, 0,0]
startIKpos = [ 0.04601942363656924, 0.35822709988129486, -0.35822709988129486, -1.5851134808151626,  0.02045307717010969, 1.4675082870772636, 0.5675728915176873]

class tuck_arm:
    
    def __init__(self, client=None):
        rospy.loginfo('tuck_arm init')
        if client != None:
            self._client = client
            self._client.wait_for_server()
        else:
            self._client = actionlib.SimpleActionClient('arm_controllerR/follow_joint_trajectory', FollowJointTrajectoryAction)
            self._client.wait_for_server()
            self._left_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            self._left_client.wait_for_server()

    def tuck(self):
        return
        rospy.loginfo('tuck_arm tuck')
        # prepare a joint trajectory
        msg = JointTrajectory()
        msg.joint_names = servos
        msg.points = list()
    
        point = JointTrajectoryPoint()
        point.positions = forward
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(5.0)
        msg.points.append(point)
        point = JointTrajectoryPoint()
        point.positions = to_side
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(8.0)
        msg.points.append(point)
        point = JointTrajectoryPoint()
        point.positions = tucked
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(11.0)
        msg.points.append(point)

        # call action
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self._client.send_goal(goal)
        
    def untuck(self):
        rospy.loginfo('tuck_arm untuck')
        # prepare a joint trajectory
        msg = JointTrajectory()
        msg.joint_names = servos
        msg.points = list()
        
        # point = JointTrajectoryPoint()
        # point.positions = to_side
        # point.velocities = [ 0.0 for servo in msg.joint_names ]
        # point.time_from_start = rospy.Duration(3.0)
        # msg.points.append(point)

        point = JointTrajectoryPoint()
        point.positions = forward
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(point)

        # call action
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self._client.send_goal(goal)

    def IKpose(self):
        rospy.loginfo('IKpose')
        # prepare a joint trajectory
        msg = JointTrajectory()
        msg.joint_names = servos
        msg.points = list()

        point = JointTrajectoryPoint()
        point.positions = startIKpos
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(point)

        # call action
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self._client.send_goal(goal)

    def left_IKpose(self):
        rospy.loginfo('IKpose')
        # prepare a joint trajectory
        msg = JointTrajectory()
        msg.joint_names = left_servos
        msg.points = list()

        point = JointTrajectoryPoint()
        point.positions = left_startIKpos
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(point)

        # call action
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self._client.send_goal(goal)


    def left_tuck(self):
        rospy.loginfo('left_tuck_arm tuck')
        # prepare a joint trajectory
        msg = JointTrajectory()
        msg.joint_names = left_servos
        msg.points = list()
    
        point = JointTrajectoryPoint()
        point.positions = left_forward
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(5.0)
        msg.points.append(point)
        point = JointTrajectoryPoint()
        point.positions = left_to_side
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(8.0)
        msg.points.append(point)
        point = JointTrajectoryPoint()
        point.positions = left_tucked
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(11.0)
        msg.points.append(point)

        # call action
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self._left_client.send_goal(goal)
        
    def left_untuck(self):
        rospy.loginfo('left_tuck_arm untuck')
        # prepare a joint trajectory
        msg = JointTrajectory()
        msg.joint_names = left_servos
        msg.points = list()
        
        #point = JointTrajectoryPoint()
        #point.positions = left_to_side
        #point.velocities = [ 0.0 for servo in msg.joint_names ]
        #point.time_from_start = rospy.Duration(3.0)
        #msg.points.append(point)

        point = JointTrajectoryPoint()
        point.positions = left_forward
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(point)

        # call action
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = msg
        self._left_client.send_goal(goal)
    
if __name__=="__main__":
    rospy.init_node("tuck_arm")
    tuck = tuck_arm()
    
    # tucking or untucking?
    if len(sys.argv) > 1 and sys.argv[1].find("u") > -1:
        tuck.untuck()
    elif len(sys.argv) > 1 and sys.argv[1].find("T") > -1:
        tuck.left_tuck()
    elif len(sys.argv) > 1 and sys.argv[1].find("U") > -1:
        tuck.left_untuck()
    else:
        tuck.tuck()
    
