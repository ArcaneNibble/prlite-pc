#!/usr/bin/env python

"""
  follow_controller.py - controller for a kinematic chain
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

import roslib; roslib.load_manifest('pr2lite_arm_navigation')
import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from diagnostic_msgs.msg import *
from std_msgs.msg import Float64

class FollowController:
    """ A controller for joint chains, exposing a FollowJointTrajectory action. """

    def __init__(self):

        name = rospy.get_name()
        rospy.loginfo("get_node name " + name )
        self.name = name
        rospy.loginfo("FollowController " + name)
        rospy.loginfo("init")

        # parameters: rates and joints
        self.rate = rospy.get_param('~controllers/'+ name +'/rate',50.0)

        self.joints = list()
        self.fudge_factor = list()
        # Get all parameter names
        parameters = rospy.get_param_names()
        for parameter in parameters:
            # Look for the parameters that name the joints.
            if parameter.find("joint_name") != -1:
              self.joints.append(rospy.get_param(parameter))

        #self.joints = rospy.get_param('~controllers/'+name+'/joints')
        #self.index = rospy.get_param('~controllers/'+name+'/index', len(device.controllers))
        self.controllers = list()
        self.fudge_factor = list()
        for joint in self.joints:
            #self.device.servos[joint].controller = self
            # Remove "_joint" from the end of the joint name to get the controller names.
            servo_nm = joint.split("_joint")[0]
            self.controllers.append(servo_nm + "_controller")

        # output for joint states publisher
        self.joint_names = []
        self.joint_positions = []
        self.joint_velocities = []

        # action server
        name = rospy.get_param('~controllers/'+name+'/action_name','follow_joint_trajectory')
        self.server = actionlib.SimpleActionServer(name, FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)
        rospy.loginfo("Started FollowController ("+name+"). Joints: " + str(self.joints))
        # Start controller state subscribers
        self.pub = list()
        for c in self.controllers:
          self.pub.append(rospy.Publisher(c + '/command', Float64))
          rospy.loginfo("Starting " + c + "/command")
        self.server.start()


    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved.")
        traj = goal.trajectory

        if set(self.joints) != set(traj.joint_names):
            msg = "Trajectory joint names does not match action controlled joints." + str(traj.joint_names)
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        if not traj.points:
            msg = "Trajectory empty."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return

        try:
            indexes = [traj.joint_names.index(joint) for joint in self.joints]
        except ValueError as val:
            msg = "Trajectory invalid."
            rospy.logerr(msg)
            self.server.set_aborted(text=msg)
            return    

        # carry out trajectory
        time = rospy.Time.now()
        start = traj.header.stamp
        r = rospy.Rate(self.rate)
        #last = [ self.device.servos[joint].angle for joint in self.joints ]
        for point in traj.points:
            while rospy.Time.now() + rospy.Duration(0.01) < start:
                rospy.sleep(0.01)

            #positions = [ self.device.servos[self.joints[k]].setControl(point.positions[indexes[k]]) for k in range(len(indexes)) ]
            t = ((start + point.time_from_start) - rospy.Time.now()).to_sec()
            if t < 1/30.0:
                continue
            rospy.logdebug(self.name + ": Sending Point," + str(positions) + " " + str(t))

            positions = [ self.pub[self.joints[k]].publish(point.positions[indexes[k]] + rospy.get_param('~fudge_factor/' + self.joints[k] + '/value', 0.0)) for k in range(len(indexes)) ]

#           desired = [ point.positions[k] for k in indexes ]
#           endtime = start + point.time_from_start
#           while rospy.Time.now() + rospy.Duration(0.01) < endtime:
#               err = [ (d-c) for d,c in zip(desired,last) ]
#               velocity = [ abs(x / (self.rate * (endtime - rospy.Time.now()).to_sec())) for x in err ]
#               rospy.logdebug(err)
#               for i in range(len(self.joints)):
#                   if err[i] > 0.01 or err[i] < -0.01:
#                       cmd = err[i] 
#                       top = velocity[i]
#                       if cmd > top:
#                           cmd = top
#                       elif cmd < -top:
#                           cmd = -top
#                       last[i] += cmd
#                       self.device.servos[self.joints[i]].dirty = True
#                       self.device.servos[self.joints[i]].relaxed = False
#                       self.device.servos[self.joints[i]].desired = last[i]
#                   else:
#                       velocity[i] = 0
#               r.sleep()
        rospy.loginfo(self.name + ": Done.")
        self.server.set_succeeded()


if __name__ == '__main__':
  rospy.loginfo("FollowController init_node " )
  rospy.init_node("follow_controller")
  server = FollowController()
  rospy.loginfo("spin")
  rospy.spin()
  rospy.loginfo("post spin")
