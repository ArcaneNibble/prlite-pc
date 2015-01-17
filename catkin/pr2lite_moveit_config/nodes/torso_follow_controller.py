#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2lite_moveit_config')
import sys
import math
roslib.load_manifest('pr2lite_moveit_config')
import actionlib
import rospy
from sensor_msgs.msg import JointState
# from control_msgs.msg import FollowJointTrajectoryAction
# from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import *
from actionlib_msgs.msg import *
# from pr2_controllers_msgs.msg import *
#from joint_states_listener.srv import ReturnJointStates
from joint_states_listener import ReturnJointStates
import time
import sys
import roslib
import tf
import tf2_ros

roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('control_msgs')



# from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryControllerState, JointTrajectoryActionGoal,\
#   SingleJointPositionAction, SingleJointPositionGoal, Pr2GripperCommandAction,\
#   Pr2GripperCommandGoal, JointTrajectoryGoal

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
        self.torso_pub = rospy.Publisher('torso_lift_controller/command', Float64, queue_size=10)
        self.set_torso_pos = rospy.Publisher('net_485net_set_torso_pos', Float64, queue_size=10)
        self.current_pos = 0.0
        self.velocity = 0.0508
        rospy.loginfo("Started TorsoFollowTrajController")
        # self.current_pos = torso_state()
        # self.handle_camera(self.current_pos)
        rospy.loginfo("init camera in TorsoFollowTrajController (%f)", self.current_pos)
        self.server.start()

    def handle_camera(self, curpos):
      try:
         br = tf.TransformBroadcaster()
         # br.sendTransform((0, 0, curpos), 
         br.sendTransform((0, 0, 0), 
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         # "/base_link", "kinect_depth_optical_frame")
                         "/base_link", "kinect_link")
      except rospy.ServiceException, e:
         rospy.loginfo("Service call failed: %s",e);

    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved %f" % goal.position)
        desired_pos_in_meters = goal.position

        self.torso_pub.publish(goal.position)
        self.current_pos = torso_state()
        # while math.fabs(self.current_pos[0] - desired_pos_in_meters) > .006:
        i = 0
        while math.fabs(self.current_pos - desired_pos_in_meters) > .006 and i < 14:
          rospy.sleep(.05)
          i = i + 1
          # self.current_pos = torso_state()
          # handle_camera(self.current_pos)
        self.server.set_succeeded()

def torso_state():
    rospy.wait_for_service("return_joint_states")
    joint_names = ["torso_lift_joint"]
    error = 1
    while (error == 1):
      try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
      except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
      for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
            rospy.sleep(.1)
        else:
             error = 0
    # return (resp.position, resp.velocity, resp.effort)
    return resp.position[ind]
  
if __name__ == '__main__':
  rospy.init_node("TorsoFollowTrajController")
  server = TorsoFollowTrajController()
  rospy.spin()
