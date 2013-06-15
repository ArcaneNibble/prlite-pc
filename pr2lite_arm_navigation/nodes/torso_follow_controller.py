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
#from joint_states_listener.srv import ReturnJointStates
from joint_states_listener import ReturnJointStates
import time
import sys
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
        desired_pos_in_meters = goal.position

        self.torso_pub.publish(goal.position)
        self.current_pos = torso_state()
        while abs(self.current_pos[0] - desired_pos_in_meters) > .006:
          rospy.sleep(.05)
          self.current_pos = torso_state()
        self.server.set_succeeded()

def torso_state():
    rospy.wait_for_service("return_joint_states")
    joint_names = ["torso_lift_joint"]
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    # return (resp.position, resp.velocity, resp.effort)
    return (resp.position)
  
if __name__ == '__main__':
  rospy.init_node("TorsoFollowTrajController")
  server = TorsoFollowTrajController()
  rospy.spin()
