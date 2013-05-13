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
from actionlib_msgs.msg import *
import sys
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *


from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import *
from std_msgs.msg import Float64


class HeadEngine:   # a crazy name, but matches our convention
    
    def __init__(self, client=None):
        self.head_pan_pub = rospy.Publisher('/head_pan_controller/command', Float64)
        self.head_tilt_pub = rospy.Publisher('/head_tilt_controller/command', Float64)
        rospy.sleep(2)
      
        self.joints = ["head_pan_joint", "head_tilt_joint"] 
        self.last = [None, None] 
        self.temps = [0.0, 0.0]
        self.previous_tilt = None
        self.previous_pan = None
        self.iter = 0
        rospy.loginfo('head_util')

        if client != None:
            self._client = client
        else:
            # self._client = actionlib.SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            self._client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
           # /head_traj_controller/point_head_action/goal 

        # /head_traj_controller/point_head_action
        rospy.loginfo('wait for head_traj_controller')
        self._client.wait_for_server()
        rospy.loginfo('head_traj_controller')

        # rospy.Subscriber('joint_states', JointState, self.stateCb)
        # rospy.Subscriber('diagnostics', DiagnosticArray, self.diagnosticCb)
        # setup home positions
        # while self.last[0] == None:
            # pass
        # self.home_pan = 0.0
        # self.home_tilt = 1.1 #1.05 #0.9561 #self.last[1]
    
    def stateCb(self, msg):
        """ Callback for JointState message. """
        try:
            indexes = [msg.name.index(name) for name in self.joints]
        except ValueError as val:
            rospy.logerr('/joint_stats.name is invalid.')
            return
        self.last = [ msg.position[k] for k in indexes ]

    def diagnosticCb(self, msg):
        """ ... """
        for entry in msg.status:
            name = entry.name.replace("Joint ", "")
            if name in self.joints:
                i = self.joints.index(name)
                for kv in entry.values:
                    if kv.key == "Temperature":
                        self.temps[i] = kv.value
        
    #######################################################
    # look at person/board
    def look_at_player(self):
        g = PointHeadGoal()
        g.target.header.frame_id = 'head_tilt_link'
        g.target.point.x = 1.0
        g.target.point.y = 0.0
        g.target.point.z = 0.0
        g.min_duration = rospy.Duration(1.0)

        self._client.send_goal(g)
        self._client.wait_for_result()

        if self._client.get_state() == GoalStatus.SUCCEEDED:
          print "Succeeded"
        else:
          print "Failed"

    def look_at_board(self):
        self.head_pan_pub.publish(0.0)
        self.head_tilt_pub.publish(1.17)
        rospy.sleep(2)
        return
        g = PointHeadGoal()
        g.target.header.frame_id = 'base_link'
        g.target.point.x = 0.34
        g.target.point.y = 0.0
        g.target.point.z = 0.0
        g.min_duration = rospy.Duration(1.0)
        self._client.send_goal(g)
        self._client.wait_for_result()

        if self._client.get_state() == GoalStatus.SUCCEEDED:
          print "Succeeded"
        else:
          print "Failed"

    def wiggle_head(self):
        # self.iter = (self.iter+1)%5
        g = PointHeadGoal()
        g.target.header.frame_id = 'head_tilt_link'
        # g.target.point.x = (self.iter-2)*0.05
        g.target.point.x = 0.1
        # g.target.point.y = 0.1
        g.target.point.y = 0.0
        g.target.point.z = 0.0
        g.min_duration = rospy.Duration(1.0)
        self._client.send_goal(g)
        self._client.wait_for_result()
        g.target.header.frame_id = 'base_link'
        g.target.point.x = 0.1
        self._client.send_goal(g)
        self._client.wait_for_result()
        if self._client.get_state() == GoalStatus.SUCCEEDED:
          print "Succeeded"
        else:
          print "Failed"

if __name__=="__main__":
    rospy.init_node("head_util_test")
    h = HeadEngine()
    
    # h.look_at_player()
    rospy.sleep(5.0)
    h.look_at_board()
    rospy.sleep(5.0)
    
    for i in range(10):
        # h.wiggle_head()
        rospy.sleep(2.0)


