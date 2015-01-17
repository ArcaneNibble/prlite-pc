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
from control_msgs.msg import *
#from pr2_controllers_msgs.msg import *
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
        self.body_mode = 6             # front top right
        self.head_mode = 7             # front top left
        self.prev_mode = 0        # no mode set
#
#       # BUTTON mapping
        self.cur_pos = 0        
        self.pose_incr = 9        # start
        self.pose_decr = 8        # select
#   
#       # digital Joystick Axes
        # self.torso_up_down = 5    # cross: 1 = up, -1 = down
        # self.shoulder_tilt = 5    # cross: 1 = up, -1 = down
        # self.shoulder_pan = 4     # cross: 1 = right -1 = left 

#
#       # analog joysticks Axes; mode dependent
#
        # joystick -head
        self.head_tilt = 1        # left Joy: 1 = up, -1 = down
        self.head_pan = 0         # left Joy: 1 = right -1 = left 

        # joystick - base
        self.move_fwd_bck = 1     # left Joy: 1 = fwd, -1 = back
        self.rot_left_right = 0   # left Joy: 1 = right, -1 = left
        self.move_left_right = 3  # right Joy: 1 = right -1 = left 
        self.rot_right_left = 2   # right Joy: 1 = fwd, -1 = back

        self.follow_mode = 8      # select

        # cross - gripper orientation
        self.elbow_pan = 4           # cross: 1 = right -1 = left 
        self.wrist_flex = 5          # cross: 1 = up, -1 = down
        self.wrist_roll_clock = 0     # triangle
        self.wrist_roll_counterclock = 2   # X
        self.gripper_close = 3       # square
        self.gripper_open = 1        # circle
        # joystick - gripper movement
        self.wrist_up_down = 1       # left Joy: 1 = up, -1 = down
        self.wrist_fwd_bck = 2       # right Joy: 1 = up, -1 = down
        self.wrist_right_left = 3    # right Joy: 1 = right -1 = left 

        # Velo Testing
        # buttons
#       self.speed_up = 1         # "0"
#       self.speed_down = 3       # square
#       self.torque_up = 0        # triangle
#       self.torque_down = 2      # X
 
        # Arm Poses
        self.pose_move_aside = 0
        self.pose_tuck   = 1   
        self.pose_untuck = 2   
        self.pose_stretch = 3

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

        # analog joysticks axes
        self.position = 2         # right joy (up / down)

        # controllers
        velo_controller = 'velo_gripper_controller/gripper_action'
        velo_torque_service = '/velo_gripper_ax12_controller/set_torque_limit'
        velo_speed_service = '/velo_gripper_ax12_controller/set_speed'
        left_gripper_controller = '/gripper_controller/gripper_action'
        head_controller = '/head_traj_controller/point_head_action'
        base_controller = '/base_controller/command'
        left_wrist_flex_controller = '/left_wrist_flex_controller/command'
        right_wrist_flex_controller = '/right_wrist_flex_controller/command'
        left_wrist_roll_controller = '/left_wrist_roll_controller/command'
        right_wrist_roll_controller = '/right_wrist_roll_controller/command'
        left_elbow_pan_controller = '/left_elbow_pan_controller/command'
        right_elbow_pan_controller = '/right_elbow_pan_controller/command'

        # initialize ROS topics 
        rospy.init_node('pr2lite_teleop')

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
                  velo_controller, GripperCommandAction)
        print "wait for velo server"
        self.velo_gripper_client.wait_for_server()
        # position in radians
        # self.velo_gripper_pub = rospy.Publisher(velo_controller, Float64, queue_size=10)
        self.max_velo_gripper_pos = .125
        self.velo_gripper_pos = self.max_velo_gripper_pos
        self.max_left_gripper_pos = .125
        self.left_gripper_pos = self.max_left_gripper_pos
        self.right_wrist_flex_client = rospy.Publisher(right_wrist_flex_controller, Float64, queue_size=10)
        self.left_wrist_flex_client = rospy.Publisher(left_wrist_flex_controller, Float64, queue_size=10)
        self.right_wrist_roll_client = rospy.Publisher(right_wrist_roll_controller, Float64, queue_size=10)
        self.left_wrist_roll_client = rospy.Publisher(left_wrist_roll_controller, Float64, queue_size=10)
        self.left_elbow_pan_client = rospy.Publisher(left_elbow_pan_controller, Float64, queue_size=10)
        self.right_elbow_pan_client = rospy.Publisher(right_elbow_pan_controller, Float64, queue_size=10)

        print "wait for left gripper server"
        self.left_gripper_client = actionlib.SimpleActionClient(
                  left_gripper_controller, Pr2GripperCommandAction)
        self.left_gripper_client.wait_for_server()

        print "wait for head controller"
        self.head_client = actionlib.SimpleActionClient(
                  head_controller, PointHeadAction)
        self.head_client.wait_for_server()

        self.cmd_vel = rospy.Publisher(base_controller, Twist, queue_size=10)

        #TODO: moveit!
        print "left arm and torso moveit commander"
        rospy.loginfo("pr2lite_teleop: left arm and torso moveit commander")
        try:
          self.left_arm_group = MoveGroupCommander("left_arm_and_torso")
        except rospy.ServiceException, e:
          self.left_arm_group = MoveGroupCommander("left_arm_and_torso")
        print "right arm and torso moveit commander"
        rospy.loginfo("pr2lite_teleop: right arm and torso moveit commander")
        try:
          self.right_arm_group = MoveGroupCommander("right_arm_and_torso")
        except rospy.ServiceException, e:
          self.right_arm_group = MoveGroupCommander("right_arm_and_torso")

        rospy.loginfo("pr2lite_teleop: speech engine")
        self.se = SpeechEngine()

        print "subscribe to joystick"
        self.joy = rospy.Subscriber('joy', Joy, self.joyCallback)
        print "joystick ready"
        self.se.say( "joystick ready")


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
              self.se.say( "right arm mode")
            new_pos = False
            if joy.buttons[self.pose_incr]:
              self.cur_pos = self.cur_pos + 1
              if self.cur_pos > self.pose_stretch:
                self.cur_pos = self.pose_move_aside
              new_pos = True
            if joy.buttons[self.pose_decr]:
              self.cur_pos = self.cur_pos - 1
              if self.cur_pos < self.pose_move_aside:
                self.cur_pos = self.pose_stretch
              new_pos = True
            print "new pos " + str(new_pos)
            # TODO: have head follow arm?
            if new_pos and self.cur_pos == self.pose_move_aside:
              print "move_aside_right_arm"
              self.se.say( "move aside right arm")
              self.right_arm_group.allow_replanning(True)
              self.right_arm_group.set_named_target("move_aside_right_arm")
              # planned = self.right_arm_group.plan()
              if self.right_arm_group.go() == False:
                self.se.say( "Error")
            elif new_pos and self.cur_pos == self.pose_tuck:
              print "tuck_right_arm"
              self.se.say( "tuck right arm")
              self.right_arm_group.allow_replanning(True)
              self.right_arm_group.set_named_target("tuck_right_arm")
              # planned = self.right_arm_group.plan()
              if self.right_arm_group.go() == False:
                self.se.say( "Error")
            elif new_pos and self.cur_pos == self.pose_untuck:
              print "untuck_right_arm"
              self.se.say( "untuck right arm")
              self.right_arm_group.allow_replanning(True)
              self.right_arm_group.set_named_target("untuck_right_arm")
              # planned = self.right_arm_group.plan()
              if self.right_arm_group.go() == False:
                self.se.say( "Error")
            elif new_pos and self.cur_pos == self.pose_stretch:
              print "stretch_right_arm"
              self.se.say( "stretch right arm")
              self.right_arm_group.allow_replanning(True)
              self.right_arm_group.set_named_target("stretch_right_arm")
              # planned = self.right_arm_group.plan()
              if self.right_arm_group.go() == False:
                self.se.say( "Error")
            threshold = 0.1
	    if math.fabs(joy.axes[self.wrist_right_left]) > threshold or math.fabs(joy.axes[self.wrist_fwd_bck]) > threshold or math.fabs(joy.axes[self.wrist_up_down]) > threshold:
              return
              self.se.say( "joystick control of right arm")
              print "joystick right arm control"
              currentrf = self.right_arm_group.get_pose_reference_frame()
              self.right_arm_group.set_pose_reference_frame(currentrf)
              self.right_arm_group.allow_replanning(False)
              # Comment out replanning:
              # self.right_arm_group.allow_replanning(True)
              # With replanning, the joystick arm control is frequently called
              # with a new status.   The planning never executes anything 
              # because it is always replanning and never executing.  
              # For now, no replannng and use blocking callbacks.
#following lines core dump
#              ee_link = self.right_arm_group.get_end_effector_link()
#              newpose = self.right_arm_group.get_current_pose(ee_link).pose
#              if self.right_arm_group.has_end_effector_link():
#                print "eef: " +  self.right_arm_group.get_end_effector_link()
#              # newposerpy = self.right_arm_group.get_current_rpy()
#              arm_delta = 0.45
#              newpose.position.x = newpose.position.x-arm_delta * joy.axes[self.wrist_right_left]
#              newpose.position.y = newpose.position.y-arm_delta * joy.axes[self.wrist_fwd_bck]
#              newpose.position.z = newpose.position.z-arm_delta * joy.axes[self.wrist_up_down]
#              # print newpose   
#              print('Setting a new target pose..')
#              self.right_arm_group.set_pose_target(newpose, ee_link)
              self.right_arm_group.set_start_state_to_current_state()
              planned = self.right_arm_group.plan()
              print ('Joint interpolated plan:')
              print planned
              self.right_arm_group.go()
              # self.right_arm_group.execute(planned)
              # scene = PlanningSceneInterface()
            wrist_delta = .15
            if joy.buttons[self.wrist_roll_clock]:
              right_wrist_roll_pos = self.get_joint_state('right_wrist_roll_joint')
              right_wrist_roll_pos = right_wrist_roll_pos - wrist_delta
              self.right_wrist_roll_client.publish(right_wrist_roll_pos)
            if joy.buttons[self.wrist_roll_counterclock]:
              right_wrist_roll_pos = self.get_joint_state('right_wrist_roll_joint')
              right_wrist_roll_pos = right_wrist_roll_pos + wrist_delta
              self.right_wrist_roll_client.publish(right_wrist_roll_pos)
            if joy.axes[self.wrist_flex] != 0:
              right_wrist_flex_pos = self.get_joint_state('right_wrist_flex_joint')
              right_wrist_flex_pos = right_wrist_flex_pos - joy.axes[self.wrist_flex]*wrist_delta
              self.right_wrist_flex_client.publish(right_wrist_flex_pos)
            if joy.buttons[self.elbow_pan] != 0:
              right_elbow_pan_pos = self.get_joint_state('right_elbow_pan_joint')
              right_elbow_pan_pos = right_elbow_pan_pos - joy.axes[self.elbow_pan]*wrist_delta
              self.right_elbow_pan_client.publish(right_elbow_pan_pos)
            if joy.buttons[self.gripper_open]:
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
            if joy.buttons[self.gripper_close]:
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
              self.se.say( "left arm mode")
            new_pos = False
            if joy.buttons[self.pose_incr]:
              self.cur_pos = self.cur_pos + 1
              if self.cur_pos > self.pose_stretch:
                self.cur_pos = self.pose_move_aside
              new_pos = True
            if joy.buttons[self.pose_decr]:
              self.cur_pos = self.cur_pos - 1
              if self.cur_pos < self.pose_move_aside:
                self.cur_pos = self.pose_stretch
              new_pos = True
            print "new pos " + str(new_pos)
            # TODO: have head follow arm?
            if new_pos and self.cur_pos == self.pose_move_aside:
              print "move_aside_left_arm"
              self.se.say( "move aside left arm")
              self.left_arm_group.set_named_target("move_aside_left_arm")
              # planned = self.left_arm_group.plan()
              if self.left_arm_group.go() == False:
                self.se.say( "Error")
            elif new_pos and self.cur_pos == self.pose_tuck:
              print "tuck_left_arm"
              self.se.say( "tuck left arm")
              self.left_arm_group.set_named_target("tuck_left_arm")
              # planned = self.left_arm_group.plan()
              if self.left_arm_group.go() == False:
                self.se.say( "Error")
            elif new_pos and self.cur_pos == self.pose_untuck:
              print "untuck_left_arm"
              self.se.say( "untuck left arm")
              self.left_arm_group.set_named_target("untuck_left_arm")
              # planned = self.left_arm_group.plan()
              if self.left_arm_group.go() == False:
                self.se.say( "Error")
            elif new_pos and self.cur_pos == self.pose_stretch:
              print "stretch_left_arm"
              self.se.say( "stretch left arm")
              self.left_arm_group.set_named_target("stretch_left_arm")
              # planned = self.left_arm_group.plan()
              if self.left_arm_group.go() == False:
                self.se.say( "Error")
                
            # TODO: Joystick Arm control
            #
            if joy.buttons[self.gripper_open]:
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
            if joy.buttons[self.gripper_close]:
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
        elif joy.buttons[self.head_mode]:
            if self.prev_mode != self.head_mode:
              self.prev_mode = self.head_mode
              self.se.say( "head mode")

            level_head = 1
            head_mv_tm = .75
            new_pos = False
            if joy.buttons[self.pose_incr]:
              self.head_pos = self.head_pos + 1
              if self.head_pos > self.head_right_hand:
                self.head_pos = self.head_joy
              new_pos = True
            if joy.buttons[self.pose_decr]:
              self.head_pos = self.head_pos - 1
              if self.head_pos < self.head_joy:
                self.head_pos = self.head_right_hand
              new_pos = True
            if self.head_pos == self.head_joy:
              # do every loop, not just with new pos
              print "head_joy"
              head_delta = .15
              cur_head_pan = self.get_joint_state('head_pan_joint')
              cur_head_tilt = self.get_joint_state('head_tilt_joint')
              print "cur head tilt %f !"%cur_head_tilt
              print "cur head pan %f !"%cur_head_pan
              threshold = 0.1
              # if math.fabs(joy.axes[self.head_pan]) > threshold or math.fabs(joy.axes[self.head_tilt]) > threshold:
              if True:
                # 3.14 / 2 = 1.57
                new_head_pan = joy.axes[self.head_pan] - 3.14
                new_head_tilt = joy.axes[self.head_tilt] - 1.57
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
            elif new_pos and self.head_pos == self.head_left_hand:
              print "head_left_hand"
              self.phg.target.header.frame_id = 'left_wrist_roll_link'
              self.phg.target.point.x = 0
              self.phg.target.point.y = 0
              self.phg.target.point.z = 0
              self.phg.pointing_frame = "kinect_depth_optical_frame";
            elif new_pos and self.head_pos == self.head_right_hand:
              print "head_right_hand"
              self.phg.target.header.frame_id = 'right_wrist_roll_link'
              self.phg.target.point.x = 0
              self.phg.target.point.y = 0
              self.phg.target.point.z = 0
              self.phg.pointing_frame = "kinect_depth_optical_frame";
            elif new_pos and self.head_pos == self.head_look_up:
              print "head_look_up"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame";
              self.phg.target.point.x = 5
              self.phg.target.point.y = 0
              self.phg.target.point.z = level_head
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
            elif new_pos and self.head_pos == self.head_look_down:
              print "head_look_down"
              self.phg.target.header.frame_id = 'base_link'
              self.phg.pointing_frame = "kinect_depth_optical_frame";
              self.phg.target.point.x = 2
              self.phg.target.point.y = 0
              self.phg.target.point.z = 0
              self.phg.min_duration = rospy.Duration(1.0)
              self.head_client.send_goal(self.phg)
              self.head_client.wait_for_result()
            elif new_pos and self.head_pos == self.head_shake_no:
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
            elif new_pos and self.head_pos == self.head_nod_yes:
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

        elif joy.buttons[self.body_mode]:
            if self.prev_mode != self.body_mode:
              self.prev_mode = self.body_mode
              self.se.say( "body mode")
            # TODO: move to tuck mode?
            # TODO: have head look down/forward?
            # TODO: follow_mode
            # a_scale = 0.9
            a_scale = 0.5
            l_scale = 0.3
            threshold = 0.1
	    if math.fabs(joy.axes[self.move_fwd_bck]) > threshold or math.fabs(joy.axes[self.rot_left_right]) > threshold:
	      vel.angular.z = -1.0*a_scale*joy.axes[self.rot_left_right];
	      vel.linear.x = l_scale*joy.axes[self.move_fwd_bck];
	      vel.linear.y = 0;
	    elif math.fabs(joy.axes[self.move_left_right]) > threshold or math.fabs(joy.axes[self.rot_right_left]) > threshold:
	      # for horizontal, roles of linear and angular are switched
	      vel.angular.z = -1.0*a_scale*joy.axes[self.rot_right_left];
	      vel.linear.x = 0;
	      vel.linear.y = l_scale*joy.axes[self.move_left_right];
	    else:
	      vel.linear.x = 0;
	      vel.linear.y = 0;
	      vel.linear.z = 0;
        self.cmd_vel.publish(vel)
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

