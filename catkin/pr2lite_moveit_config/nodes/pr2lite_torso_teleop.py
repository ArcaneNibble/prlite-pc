#!/usr/bin/python
#
#  send tele-operation commands to pilot from an HKS racing controller
#
#   Copyright (C) 2011 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'pr2lite_moveit_config'

# standard Python packages
import sys
import math

import roslib 
roslib.load_manifest(PKG_NAME)
from std_msgs.msg import Float64
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetSpeed, SetTorqueLimit
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, conversions
#from moveit_msgs.msg import RobotTrajectory, Grasp
from moveit_msgs.msg import *
from pr2lite_moveit_config.srv import ReturnJointStates
from sound_utilities import SpeechEngine
#from parallel_gripper_controller import ParallelGripperController


# In ROS Electric, the Joy message gained a header and moved to
# sensor_msgs.  Use that if it's available.
try:
    from sensor_msgs.msg import Joy
except ImportError:
    from joy.msg import Joy

def clamp(minimum, value, maximum):
    "constrain value to the range [minimum .. maximum]"
    return max(minimum, min(value, maximum))

class JoyNode():
    "Pr2Lite joystick tele-operation node."

    def __init__(self):
        "JoyNode constructor"
        print "joystick constructor"

        # PRLite Joystick controls

#       # Modes -> Front buttons
        self.right_arm_mode = 5   # front bottom right
        self.left_arm_mode = 4    # front bottom left
        self.body_mode = 6        # front top right (head mode)
        # self.head_mode = 7             # front top left
        self.no_mode = 0        # no mode set
        self.prev_mode = 0        # no mode set

#
#       # BUTTON mapping
        self.torso_up_btn = 0       # triangle (head mode)
        self.torso_down_btn = 2     # X (head mode)

        self.gripper_close_btn = 3  # square
        self.gripper_open_btn = 1   # circle

        self.arm_tuck_btn   = 9     # start
        self.arm_toggle_btn = 8     # select

#   
#       # digital Joystick Axes (cross)
        self.shoulder_tilt_crs = 5   # cross: 1 = up, -1 = down
        self.shoulder_pan_crs = 4    # cross: 1 = right, -1 = left

#
#       # analog joysticks Axes; mode dependent
#
        # joystick 
        self.elbow_pan_joy = 0   # left Joy: 1 = right, -1 = left
        self.elbow_flex_joy = 1  # left Joy: 1 = up, -1 = down
        self.wrist_flex_joy = 2  # right joy: 1 = up, 2 = down
        self.wrist_roll_joy = 3  # right joy: 1 = counter, 2 = clock

        self.head_tilt_joy = 1   # left Joy: 1 = up, -1 = down (hd mode)
        self.head_pan_joy = 0    # left Joy: 1 = right -1 = left  (hd mode)

        # Arm Poses
        self.pose_move_aside = 0
        self.pose_tuck   = 1   
        self.pose_untuck = 2   
        self.pose_stretch = 3
        self.pose_decr = 8  # select
        self.pose_incr = 9  # start

        self.cur_pos = 0
        self.new_pos = 0

        # Head Mode
        self.head_joy        = 1   
        self.head_look_up    = 2   
        self.head_look_down  = 3   
        self.head_shake_no   = 4   
        self.head_nod_yes    = 5   
        self.head_left_hand  = 6   
        self.head_right_hand = 7   
        self.head_pos = self.head_joy
        self.phg = PointHeadGoal()

        # controllers
        velo_controller = 'velo_gripper_controller/gripper_action'
        velo_torque_service = '/velo_gripper_ax12_controller/set_torque_limit'
        velo_speed_service = '/velo_gripper_ax12_controller/set_speed'
        left_gripper_controller = '/gripper_controller/gripper_action'
        head_controller = '/head_traj_controller/point_head_action'
        # base_controller = '/base_controller/command'
        left_wrist_flex_controller = '/left_wrist_flex_controller/command'
        right_wrist_flex_controller = '/right_wrist_flex_controller/command'
        left_wrist_roll_controller = '/left_wrist_roll_controller/command'
        right_wrist_roll_controller = '/right_wrist_roll_controller/command'
        left_elbow_pan_controller = '/left_elbow_pan_controller/command'
        right_elbow_pan_controller = '/right_elbow_pan_controller/command'
        left_elbow_flex_controller = '/left_elbow_flex_controller/command'
        right_elbow_flex_controller = '/right_elbow_flex_controller/command'
        left_shoulder_pan_controller = '/left_shoulder_pan_controller/command'
        right_shoulder_pan_controller = '/right_shoulder_pan_controller/command'
        left_shoulder_tilt_controller = '/left_shoulder_tilt_controller/command'
        right_shoulder_tilt_controller = '/right_shoulder_tilt_controller/command'
        torso_lift_controller = '/torso_lift_controller/command'

        # initialize ROS topics 
        rospy.init_node('pr2lite_torso_teleop')

        self.torque = 500
        if self.torque > 1023:
           self.torque = 1023
        if self.torque < 0:
           self.torque = 0
        print "wait for velo torque service"
        rospy.wait_for_service(velo_torque_service)
        self.velo_torque_service = rospy.ServiceProxy( velo_torque_service, SetTorqueLimit)
        self.speed = 0.5
        print "wait for velo speed service"
        rospy.wait_for_service(velo_speed_service)
        self.velo_speed_service = rospy.ServiceProxy(
                  velo_speed_service, SetSpeed)
        self.velo_gripper_client = actionlib.SimpleActionClient(
                  velo_controller, Pr2GripperCommandAction)
        print "wait for velo server"
        self.velo_gripper_client.wait_for_server()
        # position in radians
        # self.velo_gripper_pub = rospy.Publisher(velo_controller, Float64)
        self.max_velo_gripper_pos = .125
        self.velo_gripper_pos = self.max_velo_gripper_pos
        self.max_left_gripper_pos = .125
        self.left_gripper_pos = self.max_left_gripper_pos
        self.right_wrist_flex_client = rospy.Publisher(right_wrist_flex_controller, Float64)
        self.left_wrist_flex_client = rospy.Publisher(left_wrist_flex_controller, Float64)
        self.right_wrist_roll_client = rospy.Publisher(right_wrist_roll_controller, Float64)
        self.left_wrist_roll_client = rospy.Publisher(left_wrist_roll_controller, Float64)
        self.left_elbow_pan_client = rospy.Publisher(left_elbow_pan_controller, Float64)
        self.right_elbow_pan_client = rospy.Publisher(right_elbow_pan_controller, Float64)
        self.left_elbow_flex_client = rospy.Publisher(left_elbow_flex_controller, Float64)
        self.right_elbow_flex_client = rospy.Publisher(right_elbow_flex_controller, Float64)
        self.left_shoulder_pan_client = rospy.Publisher(left_shoulder_pan_controller, Float64)
        self.right_shoulder_pan_client = rospy.Publisher(right_shoulder_pan_controller, Float64)
        self.left_shoulder_tilt_client = rospy.Publisher(left_shoulder_tilt_controller, Float64)
        self.right_shoulder_tilt_client = rospy.Publisher(right_shoulder_tilt_controller, Float64)
        self.torso_lift_client = rospy.Publisher(torso_lift_controller, Float64)

        print "wait for left gripper server"
        self.left_gripper_client = actionlib.SimpleActionClient(
                  left_gripper_controller, Pr2GripperCommandAction)
        self.left_gripper_client.wait_for_server()

        print "wait for head controller"
        self.head_client = actionlib.SimpleActionClient(
                  head_controller, PointHeadAction)
        self.head_client.wait_for_server()

        # self.cmd_vel = rospy.Publisher(base_controller, Twist)

        print "subscribe to joystick"
        self.joy = rospy.Subscriber('joy', Joy, self.joyCallback)
        print "joystick ready"


    def get_joint_state(self,name):
        joint_names = [name]
        rospy.wait_for_service("return_joint_states")
        try:
          s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
          resp = s(joint_names)
        except rospy.ServiceException, e:
          print "error when calling return_joint_states: %s"%e
          sys.exit(1)
        for (ind, joint_name) in enumerate(joint_names):
          if joint_name == name:
            pos = resp.position[ind]
          elif(not resp.found[ind]):
            print "joint %s not found!"%joint_name
        return pos

    def joyCallback(self, joy):
        "invoked every time a joystick message arrives"
        #rospy.logdebug('joystick input:\n' + str(joy))
        rospy.loginfo('joystick input:\n' + str(joy))
        print "joystick input:" 
        print joy

	vel = Twist()
	vel.angular.z = 0;
	vel.linear.x = 0;
	vel.linear.y = 0;
        if joy.buttons[self.right_arm_mode]:
            if self.prev_mode != self.right_arm_mode:
              self.prev_mode = self.right_arm_mode
            self.new_pos = 0
            if joy.buttons[self.pose_incr]:
              self.cur_pos = self.cur_pos + 1
              if self.cur_pos > self.pose_stretch:
                self.cur_pos = self.pose_move_aside
              self.new_pos = 5
            if joy.buttons[self.pose_decr]:
              self.cur_pos = self.cur_pos - 1
              if self.cur_pos < self.pose_move_aside:
                self.cur_pos = self.pose_stretch
              self.new_pos = 5
            print "new pos " + str(self.new_pos)
            # TODO: have head follow arm?
            if self.new_pos and self.cur_pos == self.pose_move_aside:
              print "move_aside_right_arm"
              self.torso_lift_client.publish(0)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_flex_joint' + '/value', 0.0)
              self.right_elbow_flex_client.publish(-1.57 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_pan_joint' + '/value', 0.0)
              self.right_elbow_pan_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_shoulder_pan_joint' + '/value', 0.0)
              self.right_shoulder_pan_client.publish(1.57 + fudge_value)
              self.right_shoulder_tilt_client.publish(1.213)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_flex_joint' + '/value', 0.0)
              self.right_wrist_flex_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_roll_joint' + '/value', 0.0)
              self.right_wrist_roll_client.publish(0 + fudge_value)
            elif self.new_pos and self.cur_pos == self.pose_tuck:
              print "tuck_right_arm"
              self.torso_lift_client.publish(0 )
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_flex_joint' + '/value', 0.0)
              self.right_elbow_flex_client.publish(-1.57 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_pan_joint' + '/value', 0.0)
              self.right_elbow_pan_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_shoulder_pan_joint' + '/value', 0.0)
              self.right_shoulder_pan_client.publish(0 + fudge_value)
              self.right_shoulder_tilt_client.publish(1.213 )
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_flex_joint' + '/value', 0.0)
              self.right_wrist_flex_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_roll_joint' + '/value', 0.0)
              self.right_wrist_roll_client.publish(0 + fudge_value)
            elif self.new_pos and self.cur_pos == self.pose_untuck:
              print "untuck_right_arm"
              self.torso_lift_client.publish(0)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_flex_joint' + '/value', 0.0)
              self.right_elbow_flex_client.publish(-1.57 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_pan_joint' + '/value', 0.0)
              self.right_elbow_pan_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_shoulder_pan_joint' + '/value', 0.0)
              self.right_shoulder_pan_client.publish(0 + fudge_value)
              self.right_shoulder_tilt_client.publish(.27)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_flex_joint' + '/value', 0.0)
              self.right_wrist_flex_client.publish(1.57 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_roll_joint' + '/value', 0.0)
              self.right_wrist_roll_client.publish(0 + fudge_value)
            elif self.new_pos and self.cur_pos == self.pose_stretch:
              print "stretch_right_arm"
              self.torso_lift_client.publish(0.3)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_flex_joint' + '/value', 0.0)
              self.right_elbow_flex_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_pan_joint' + '/value', 0.0)
              self.right_elbow_pan_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_shoulder_pan_joint' + '/value', 0.0)
              self.right_shoulder_pan_client.publish(0 + fudge_value)
              self.right_shoulder_tilt_client.publish(1.213)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_flex_joint' + '/value', 0.0)
              self.right_wrist_flex_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_roll_joint' + '/value', 0.0)
              self.right_wrist_roll_client.publish(0 + fudge_value)
            if self.new_pos > 0:
              self.new_pos = self.new_pos - 1
            threshold = 0.1


#	    if math.fabs(joy.axes[self.elbow_pan_joy]) > threshold or math.fabs(joy.axes[self.elbow_flex_joy]) > threshold or math.fabs(joy.axes[self.wrist_flex_joy]) > threshold or math.fabs(joy.axes[self.wrist_roll_joy]) > threshold or math.fabs(joy.axes[self.shoulder_pan_joy]) > threshold or math.fabs(joy.axes[self.shoulder_tilt_joy]) > threshold or math.fabs(joy.axes[self.wrist_flex_joy]) > threshold or math.fabs(joy.axes[self.wrist_roll_joy]) > threshold:
#              print "joystick right arm control"
#            else:
#              return
            joint_delta = .15
            if math.fabs(joy.axes[self.wrist_roll_joy]) > threshold:
              right_wrist_roll_pos = self.get_joint_state('right_wrist_roll_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_roll_joint' + '/value', 0.0)

              right_wrist_roll_pos = right_wrist_roll_pos - joy.axes[self.wrist_roll_joy]*joint_delta + fudge_value
              self.right_wrist_roll_client.publish(right_wrist_roll_pos)
            if math.fabs(joy.axes[self.wrist_flex_joy]) > threshold:
              right_wrist_flex_pos = self.get_joint_state('right_wrist_flex_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_wrist_flex_joint' + '/value', 0.0)
              right_wrist_flex_pos = right_wrist_flex_pos - joy.axes[self.wrist_flex_joy]*joint_delta + fudge_value
              self.right_wrist_flex_client.publish(right_wrist_flex_pos)
            if math.fabs(joy.axes[self.elbow_pan_joy]) > threshold:
              right_elbow_pan_pos = self.get_joint_state('right_elbow_pan_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_pan_joint' + '/value', 0.0)
              right_elbow_pan_pos = right_elbow_pan_pos - joy.axes[self.elbow_pan_joy]*joint_delta + fudge_value
              self.right_elbow_pan_client.publish(right_elbow_pan_pos)
            if math.fabs(joy.axes[self.elbow_flex_joy]) > threshold:
              right_elbow_flex_pos = self.get_joint_state('right_elbow_flex_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_elbow_flex_joint' + '/value', 0.0)
              print "right_elbow_flex old pos " + str(right_elbow_flex_pos)
              print "right_elbow_flex fudge value " + str(fudge_value)
              right_elbow_flex_pos = right_elbow_flex_pos - joy.axes[self.elbow_flex_joy]*joint_delta + fudge_value
              self.right_elbow_flex_client.publish(right_elbow_flex_pos)
              print "right_elbow_flex new pos " + str(right_elbow_flex_pos)

            if math.fabs(joy.axes[self.shoulder_tilt_crs]) > threshold:
              right_shoulder_tilt_pos = self.get_joint_state('right_shoulder_tilt_joint')
              right_elbow_pan_pos = right_elbow_pan_pos - joy.axes[self.elbow_pan_joy]*joint_delta
              right_shoulder_tilt_pos = right_shoulder_tilt_pos - joy.axes[self.shoulder_tilt_crs]*joint_delta 
            if math.fabs(joy.axes[self.shoulder_pan_crs]) > threshold:
              right_shoulder_pan_pos = self.get_joint_state('right_shoulder_pan_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'right_shoulder_pan_joint' + '/value', 0.0)
              right_shoulder_pan_pos = right_shoulder_pan_pos - joy.axes[self.shoulder_pan_crs]*joint_delta + fudge_value
              self.right_shoulder_pan_client.publish(right_shoulder_pan_pos)

            if math.fabs(joy.buttons[self.torso_up_btn]) > threshold:
              torso_pos = self.get_joint_state('torso_lift_joint')
              if torso_pos < 2.95:
                torso_pos = torso_pos + .05
              self.torso_lift_client.publish(torso_pos)
            if math.fabs(joy.buttons[self.torso_down_btn]) > threshold:
              torso_pos = self.get_joint_state('torso_lift_joint')
              if torso_pos > .05:
                torso_pos = torso_pos - .05
              self.torso_lift_client.publish(torso_pos)
            if math.fabs(joy.buttons[self.gripper_open_btn]) > threshold:
              velo_delta = .005
              self.velo_gripper_pos = self.velo_gripper_pos + velo_delta
              if self.velo_gripper_pos > self.max_velo_gripper_pos:
                self.velo_gripper_pos = self.max_velo_gripper_pos
              print "velo gripper " + str(self.velo_gripper_pos)
              goal = Pr2GripperCommandGoal()
              goal.command.position = self.velo_gripper_pos
              goal.command.max_effort = 1000
              self.velo_gripper_client.send_goal(goal)
              self.velo_gripper_client.wait_for_result(rospy.Duration(1.0))
            if math.fabs(joy.buttons[self.gripper_close_btn]) > threshold:
              velo_delta = .005
              self.velo_gripper_pos = self.velo_gripper_pos - velo_delta
              if self.velo_gripper_pos < 0:
                self.velo_gripper_pos = 0
              print "velo gripper " + str(self.velo_gripper_pos)
              goal = Pr2GripperCommandGoal()
              goal.command.position = self.velo_gripper_pos
              goal.command.max_effort = 1000
              self.velo_gripper_client.send_goal(goal)
              self.velo_gripper_client.wait_for_result(rospy.Duration(1.0))

        elif joy.buttons[self.left_arm_mode]:
                
            if self.prev_mode != self.left_arm_mode:
              self.prev_mode = self.left_arm_mode
            self.new_pos = 0
            if joy.buttons[self.pose_incr]:
              self.cur_pos = self.cur_pos + 1
              if self.cur_pos > self.pose_stretch:
                self.cur_pos = self.pose_move_aside
              self.new_pos = 5
            if joy.buttons[self.pose_decr]:
              self.cur_pos = self.cur_pos - 1
              if self.cur_pos < self.pose_move_aside:
                self.cur_pos = self.pose_stretch
              self.new_pos = 5
            print "new pos " + str(self.new_pos)
            # TODO: have head follow arm?
            if self.new_pos and self.cur_pos == self.pose_move_aside:
              print "move_aside_left_arm"
              self.torso_lift_client.publish(0)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_flex_joint' + '/value', 0.0)
              self.left_elbow_flex_client.publish(-1.57 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_pan_joint' + '/value', 0.0)
              self.left_elbow_pan_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_shoulder_pan_joint' + '/value', 0.0)
              self.left_shoulder_pan_client.publish(-1.57 + fudge_value)
              self.left_shoulder_tilt_client.publish(1.213)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_flex_joint' + '/value', 0.0)
              self.left_wrist_flex_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_roll_joint' + '/value', 0.0)
              self.left_wrist_roll_client.publish(0 + fudge_value)
            elif self.new_pos and self.cur_pos == self.pose_tuck:
              print "tuck_left_arm"
              self.torso_lift_client.publish(0 )
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_flex_joint' + '/value', 0.0)
              self.left_elbow_flex_client.publish(-1.57 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_pan_joint' + '/value', 0.0)
              self.left_elbow_pan_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_shoulder_pan_joint' + '/value', 0.0)
              self.left_shoulder_pan_client.publish(0 + fudge_value)
              self.left_shoulder_tilt_client.publish(1.213 )
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_flex_joint' + '/value', 0.0)
              self.left_wrist_flex_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_roll_joint' + '/value', 0.0)
              self.left_wrist_roll_client.publish(0 + fudge_value)
            elif self.new_pos and self.cur_pos == self.pose_untuck:
              print "untuck_left_arm"
              self.torso_lift_client.publish(0)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_flex_joint' + '/value', 0.0)
              self.left_elbow_flex_client.publish(-1.57 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_pan_joint' + '/value', 0.0)
              self.left_elbow_pan_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_shoulder_pan_joint' + '/value', 0.0)
              self.left_shoulder_pan_client.publish(0 + fudge_value)
              self.left_shoulder_tilt_client.publish(.27)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_flex_joint' + '/value', 0.0)
              self.left_wrist_flex_client.publish(1.57 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_roll_joint' + '/value', 0.0)
              self.left_wrist_roll_client.publish(0 + fudge_value)
            elif self.new_pos and self.cur_pos == self.pose_stretch:
              print "stretch_left_arm"
              self.torso_lift_client.publish(0.3)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_flex_joint' + '/value', 0.0)
              self.left_elbow_flex_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_pan_joint' + '/value', 0.0)
              self.left_elbow_pan_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_shoulder_pan_joint' + '/value', 0.0)
              self.left_shoulder_pan_client.publish(0 + fudge_value)
              self.left_shoulder_tilt_client.publish(1.213)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_flex_joint' + '/value', 0.0)
              self.left_wrist_flex_client.publish(0 + fudge_value)
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_roll_joint' + '/value', 0.0)
              self.left_wrist_roll_client.publish(0 + fudge_value)
            if self.new_pos > 0:
              self.new_pos = self.new_pos - 1
            threshold = 0.1


#	    if math.fabs(joy.axes[self.elbow_pan_joy]) > threshold or math.fabs(joy.axes[self.elbow_flex_joy]) > threshold or math.fabs(joy.axes[self.wrist_flex_joy]) > threshold or math.fabs(joy.axes[self.wrist_roll_joy]) > threshold or math.fabs(joy.axes[self.shoulder_pan_joy]) > threshold or math.fabs(joy.axes[self.shoulder_tilt_joy]) > threshold or math.fabs(joy.axes[self.wrist_flex_joy]) > threshold or math.fabs(joy.axes[self.wrist_roll_joy]) > threshold:
#              print "joystick left arm control"
#            else:
#              return
            joint_delta = .15
            if math.fabs(joy.axes[self.wrist_roll_joy]) > threshold:
              left_wrist_roll_pos = self.get_joint_state('left_wrist_roll_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_roll_joint' + '/value', 0.0)

              left_wrist_roll_pos = left_wrist_roll_pos - joy.axes[self.wrist_roll_joy]*joint_delta + fudge_value
              self.left_wrist_roll_client.publish(left_wrist_roll_pos)
            if math.fabs(joy.axes[self.wrist_flex_joy]) > threshold:
              left_wrist_flex_pos = self.get_joint_state('left_wrist_flex_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_wrist_flex_joint' + '/value', 0.0)
              left_wrist_flex_pos = left_wrist_flex_pos - joy.axes[self.wrist_flex_joy]*joint_delta + fudge_value
              self.left_wrist_flex_client.publish(left_wrist_flex_pos)
            if math.fabs(joy.axes[self.elbow_pan_joy]) > threshold:
              left_elbow_pan_pos = self.get_joint_state('left_elbow_pan_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_pan_joint' + '/value', 0.0)
              left_elbow_pan_pos = left_elbow_pan_pos - joy.axes[self.elbow_pan_joy]*joint_delta + fudge_value
              self.left_elbow_pan_client.publish(left_elbow_pan_pos)
            if math.fabs(joy.axes[self.elbow_flex_joy]) > threshold:
              left_elbow_flex_pos = self.get_joint_state('left_elbow_flex_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_elbow_flex_joint' + '/value', 0.0)
              print "left_elbow_flex old pos " + str(left_elbow_flex_pos)
              print "left_elbow_flex fudge value " + str(fudge_value)
              left_elbow_flex_pos = left_elbow_flex_pos - joy.axes[self.elbow_flex_joy]*joint_delta + fudge_value
              self.left_elbow_flex_client.publish(left_elbow_flex_pos)
              print "left_elbow_flex new pos " + str(left_elbow_flex_pos)

            if math.fabs(joy.axes[self.shoulder_tilt_crs]) > threshold:
              left_shoulder_tilt_pos = self.get_joint_state('left_shoulder_tilt_joint')
              left_elbow_pan_pos = left_elbow_pan_pos - joy.axes[self.elbow_pan_joy]*joint_delta
              left_shoulder_tilt_pos = left_shoulder_tilt_pos - joy.axes[self.shoulder_tilt_crs]*joint_delta 
            if math.fabs(joy.axes[self.shoulder_pan_crs]) > threshold:
              left_shoulder_pan_pos = self.get_joint_state('left_shoulder_pan_joint')
              fudge_value = rospy.get_param('~fudge_factor/' + 'left_shoulder_pan_joint' + '/value', 0.0)
              left_shoulder_pan_pos = left_shoulder_pan_pos - joy.axes[self.shoulder_pan_crs]*joint_delta + fudge_value
              self.left_shoulder_pan_client.publish(left_shoulder_pan_pos)

            if math.fabs(joy.buttons[self.torso_up_btn]) > threshold:
              torso_pos = self.get_joint_state('torso_lift_joint')
              if torso_pos < 2.95:
                torso_pos = torso_pos + .05
              self.torso_lift_client.publish(torso_pos)
            if math.fabs(joy.buttons[self.torso_down_btn]) > threshold:
              torso_pos = self.get_joint_state('torso_lift_joint')
              if torso_pos > .05:
                torso_pos = torso_pos - .05
              self.torso_lift_client.publish(torso_pos)
            if joy.buttons[self.gripper_open_btn]:
              left_gripper_delta = .005
              self.left_gripper_pos = self.left_gripper_pos + left_gripper_delta
              if self.left_gripper_pos > self.max_left_gripper_pos:
                self.left_gripper_pos = self.max_left_gripper_pos
              print "left gripper " + str(self.left_gripper_pos)
              goal = Pr2GripperCommandGoal()
              goal.command.position = self.left_gripper_pos
              goal.command.max_effort = 1000
              self.left_gripper_client.send_goal(goal)
              self.left_gripper_client.wait_for_result(rospy.Duration(1.0))
            if joy.buttons[self.gripper_close_btn]:
              left_gripper_delta = .005
              self.left_gripper_pos = self.left_gripper_pos - left_gripper_delta
              if self.left_gripper_pos < 0:
                self.left_gripper_pos = 0
              print "left gripper " + str(self.left_gripper_pos)
              goal = Pr2GripperCommandGoal()
              goal.command.position = self.left_gripper_pos
              goal.command.max_effort = 1000
              self.left_gripper_client.send_goal(goal)
              self.left_gripper_client.wait_for_result(rospy.Duration(1.0))
        elif joy.buttons[self.body_mode]:
            if self.prev_mode != self.body_mode:
              self.prev_mode = self.body_mode

            level_head = 1
            head_mv_tm = .75
            self.new_pos = 0
            if joy.buttons[self.pose_incr]:
              self.head_pos = self.head_pos + 1
              if self.head_pos > self.head_right_hand:
                self.head_pos = self.head_joy
              self.new_pos = 5
            if joy.buttons[self.pose_decr]:
              self.head_pos = self.head_pos - 1
              if self.head_pos < self.head_joy:
                self.head_pos = self.head_right_hand
              self.new_pos = 5
            if self.head_pos == self.head_joy:
              # do every loop, not just with new pos
              print "head_joy"
              head_delta = .15
              cur_head_pan = self.get_joint_state('head_pan_joint')
              cur_head_tilt = self.get_joint_state('head_tilt_joint')
              print "cur head tilt %f !"%cur_head_tilt
              print "cur head pan %f !"%cur_head_pan
              threshold = 0.1
              # if math.fabs(joy.axes[self.head_pan_joy]) > threshold or math.fabs(joy.axes[self.head_tilt]) > threshold:
              if True:
                # 3.14 / 2 = 1.57
                new_head_pan = joy.axes[self.head_pan_joy] - 3.14
                new_head_tilt = joy.axes[self.head_tilt_joy] - 1.57
                self.phg.target.header.frame_id = 'head_pan_link'
                self.phg.pointing_frame = "kinect_depth_optical_frame";
                self.phg.target.point.x = 5* math.sin(new_head_tilt) * math.cos(new_head_pan)
                self.phg.target.point.y = 5* math.sin(new_head_tilt) * math.sin(new_head_pan)
                self.phg.target.point.z = 5* math.cos(new_head_tilt) 
                print "head x %f !"%self.phg.target.point.x
                print "head y %f !"%self.phg.target.point.y
                print "head z %f !"%self.phg.target.point.z
                self.phg.min_duration = rospy.Duration(1.0)
                self.head_client.send_goal(self.phg)
                self.head_client.wait_for_result()
                # rospy.sleep(head_mv_tm)
            elif self.new_pos and self.head_pos == self.head_left_hand:
              print "head_left_hand"
              self.phg.target.header.frame_id = 'left_wrist_roll_link'
              self.phg.target.point.x = 0
              self.phg.target.point.y = 0
              self.phg.target.point.z = 0
              self.phg.pointing_frame = "kinect_depth_optical_frame";
            elif self.new_pos and self.head_pos == self.head_right_hand:
              print "head_right_hand"
              self.phg.target.header.frame_id = 'right_wrist_roll_link'
              self.phg.target.point.x = 0
              self.phg.target.point.y = 0
              self.phg.target.point.z = 0
              self.phg.pointing_frame = "kinect_depth_optical_frame";
            elif self.new_pos and self.head_pos == self.head_look_up:
              print "head_look_up"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame";
              self.phg.target.point.x = 5
              self.phg.target.point.y = 0
              self.phg.target.point.z = level_head
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
            elif self.new_pos and self.head_pos == self.head_look_down:
              print "head_look_down"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame";
              self.phg.target.point.x = 2
              self.phg.target.point.y = 0
              self.phg.target.point.z = 0
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
            elif self.new_pos and self.head_pos == self.head_shake_no:
              print "head_shake_no"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame";
              self.phg.target.point.x = 5
              self.phg.target.point.y = 0
              self.phg.target.point.z = level_head
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = 1
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = -1
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = 1
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = -1
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.y = 0
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
            elif self.new_pos and self.head_pos == self.head_nod_yes:
              print "nod_yes"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame";
              self.phg.target.point.x = 5
              self.phg.target.point.y = 0
              self.phg.target.point.z = level_head
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.x = 2
              self.phg.target.point.z = 0
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.x = 5
              self.phg.target.point.z = level_head
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.x = 2
              self.phg.target.point.z = 0
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
              rospy.sleep(head_mv_tm)
              self.phg.target.point.x = 5
              self.phg.target.point.z = level_head
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()

        if self.head_pos == self.head_left_hand or self.head_pos == self.head_right_hand:
          self.head_client.send_goal(self.phg)
          self.head_client.wait_for_result()

def main():
    joynode = JoyNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
    rospy.loginfo('joystick vehicle controller finished')

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())

