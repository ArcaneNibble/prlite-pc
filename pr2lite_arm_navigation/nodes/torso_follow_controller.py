#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2lite_arm_navigation')
roslib.load_manifest('pr2_position_scripts')
import actionlib
import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryActionGoal
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')



from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryControllerState, JointTrajectoryActionGoal,\
  SingleJointPositionAction, SingleJointPositionGoal, Pr2GripperCommandAction,\
  Pr2GripperCommandGoal, JointTrajectoryGoal

from diagnostic_msgs.msg import *
from std_msgs.msg import Float64
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetSpeed


class TorsoFollowTrajController:
    """ A controller for joint chains, exposing a FollowJointTrajectory action. """
    def __init__(self):

        # action server for FollowController
        self.active = 0
        self.name = "torso_controller/position_joint_action"
        self.server = actionlib.SimpleActionServer(self.name, SingleJointPositionAction, execute_cb=self.actionCb, auto_start=False)
        self.torso_pub = rospy.Publisher('torso_lift_controller/command', Float64)
        self.set_torso_pos = rospy.Publisher('net_485net_set_torso_pos', Float64)
        self.current_pos = 0.0
        self.velocity = 0.0508
        rospy.loginfo("Started TorsoFollowTrajController")
        self.server.start()

    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved %f" % goal.position)
        time = rospy.Time.now()
        self.active = 1
        desired_pos_in_meters = goal.position

        starttime = rospy.Time.now() 
        endtime = rospy.Time.now() + rospy.Duration(abs((desired_pos_in_meters - self.current_pos) / self.velocity))

        self.torso_pub.publish(goal.position)
        while rospy.Time.now() < endtime:
          if self.current_pos > goal.position:
            est_pos = self.current_pos - self.velocity * (rospy.Time.now().to_sec() - starttime.to_sec())
          else:
            est_pos = self.current_pos + self.velocity * (rospy.Time.now().to_sec() - starttime.to_sec())
          self.set_torso_pos.publish(est_pos)
          print "torso %f" % est_pos
          rospy.sleep(.05)
        self.current_pos = goal.position
        print "torso %f" % self.current_pos
        self.set_torso_pos.publish(self.current_pos)
        self.server.set_succeeded()
  
if __name__ == '__main__':
  rospy.init_node("TorsoFollowTrajController")
  server = TorsoFollowTrajController()
  rospy.spin()
