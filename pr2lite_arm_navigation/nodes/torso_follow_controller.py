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
        #self.server = actionlib.SimpleActionServer(self.name, FollowJointTrajectoryAction, execute_cb=self.actionCb, auto_start=False)
        self.server = actionlib.SimpleActionServer(self.name, SingleJointPositionAction, execute_cb=self.actionCb, auto_start=False)
        # /joint_states
        rospy.Subscriber('joint_states', JointState, self.getJointState)
        # self.torso_pub = rospy.Publisher('torso_lift_joint/command', Float64)
        self.torso_pub = rospy.Publisher('torso_lift_controller/command', Float64)
        self.current_pos = -1
        rospy.loginfo("Started TorsoFollowTrajController")
        self.server.start()

    def actionCb(self, goal):
        rospy.loginfo(self.name + ": Action goal recieved.")
        # float64 position
        # duration min_duration
        # float64 max_velocity
        # time = rospy.Time.now()
        # get current position
        self.active = 1
        while self.current_pos == -1:
          rospy.sleep(.05)
        endtime = rospy.Time.now() + rospy.Duration(6.0)
        # endtime = rospy.Time.now() + goal.min_duration
        # if goal.position != 0:
        desired_pos_in_meters = goal.position
        # else:
          # pr2lite does not support variable torso lift velocity.  
          # Convert to position-based.
          #desired_pos_in_meters = cur_pos + (goal.min_duration * goal.max_velocity)
          # desired_pos_in_meters = goal.min_duration.to_sec() * goal.max_velocity
          # desired_pos_in_meters = desired_pos_in_meters + cur_pos
        # torso should be linact_type_raw with height from 0 to 1000. 
        # new_pos = int(1000 * desired_pos_in_meters / 3.28084) 

        # torso should be dynamixel_type_raw with height in meters
        # self.torso_pub.publish(desired_position_in_meters)
        self.torso_pub.publish(goal.position)
        while abs(self.current_pos - goal.position) > .006 and rospy.Time.now() < endtime:
          rospy.sleep(.05)
        self.active = -1
        if abs(self.current_pos - goal.position) < .006:
          self.server.set_succeeded()
        else:
          self.server.set_aborted()
  
    # we could have one controller shared among the arms, head, and torso to reduce 
    # the overhead of the callback. For now, only process joint states to get current_pos 
    # once we receive a torso trajectory message.
    def getJointState(self, msg):
      if self.active == 1:
        for index, joint_state_name in enumerate(msg.name):
          if joint_state_name == "torso_lift_joint":
            self.current_pos = msg.position[index]
      else:
        self.current_pos = -1

if __name__ == '__main__':
  rospy.init_node("TorsoFollowTrajController")
  server = TorsoFollowTrajController()
  rospy.spin()
