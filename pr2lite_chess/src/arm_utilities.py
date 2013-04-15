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
import rospy, actionlib
# from math import sqrt fabs
from math import sqrt
import math 
import pexpect

import actionlib
from pr2_controllers_msgs.msg import (Pr2GripperCommandGoal, Pr2GripperCommand,
                                      Pr2GripperCommandAction)
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


from chess_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped
# from simple_arm_server.msg import *
# from simple_arm_server.srv import * 
from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from sensor_msgs.msg import JointState

from chess_utilities import SQUARE_SIZE, castling_extras
from tuck_arm import *

GRIPPER_OPEN = 0.05
GRIPPER_CLOSE = 0.0075

class ArmPlanner:
    """ Connection to the arm server. """
    
    def __init__(self, client=None, listener=None):
        rospy.loginfo('ArmPlanner Init')
        print "ArmPlanner Init"
        if client==None:
            #rospy.wait_for_service('simple_arm_server/move')
            #self.move = rospy.ServiceProxy('simple_arm_server/move', MoveArm) 
            #ARD: uncomment next line
            rospy.loginfo('ArmPlanner pr2lite_move_right_arm')
            arm = "right"
            self.move_arm_client = actionlib.SimpleActionClient('move_right_arm', MoveArmAction)
            #self.move_arm_client = actionlib.SimpleActionClient('move_right_arm', MoveArmAction)
            #ARD: uncomment next line
            rospy.loginfo('ArmPlanner wait_for_server')
            self.move_arm_client.wait_for_server()
            if arm is "right":
              service = "r_gripper_controller/gripper_action"
            else:
              service = "l_gripper_controller/gripper_action"
            rospy.loginfo('Gripper wait_for_server')
            self.gripper = actionlib.SimpleActionClient(service, Pr2GripperCommandAction)
            self.gripper.wait_for_server()
        else:
            self.move_arm_client = client
      
        self.success = True
        # setup tf for translating poses
        self.listener = listener
        rospy.loginfo('ArmPlanner Init done')

    def poseConstraintToPositionOrientationConstraints(self, pose_constraint):
        position_constraint = PositionConstraint()
        orientation_constraint = OrientationConstraint()
        position_constraint.header = pose_constraint.header
        position_constraint.link_name = pose_constraint.link_name
        print "poseConstr %s" % pose_constraint.link_name
        position_constraint.position = pose_constraint.pose.position
        position_constraint.constraint_region_shape.type = 0
        position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.x)
        position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.y)
        position_constraint.constraint_region_shape.dimensions.append(2*pose_constraint.absolute_position_tolerance.z)
    
        position_constraint.constraint_region_orientation.x = 0.0
        position_constraint.constraint_region_orientation.y = 0.0
        position_constraint.constraint_region_orientation.z = 0.0
        position_constraint.constraint_region_orientation.w = 1.0
    
        position_constraint.weight = 1.0
    
        orientation_constraint.header = pose_constraint.header
        orientation_constraint.link_name = pose_constraint.link_name
        print "poseConstraintToPosi... %s" % pose_constraint.link_name
        orientation_constraint.orientation = pose_constraint.pose.orientation
        orientation_constraint.type = pose_constraint.orientation_constraint_type
        orientation_constraint.absolute_roll_tolerance = pose_constraint.absolute_roll_tolerance
        orientation_constraint.absolute_pitch_tolerance = pose_constraint.absolute_pitch_tolerance
        orientation_constraint.absolute_yaw_tolerance = pose_constraint.absolute_yaw_tolerance
        orientation_constraint.weight = 1.0
        print "poseConstr return" 
        return (position_constraint, orientation_constraint)

    def addGoalConstraintToMoveArmGoal(self, pose_constraint, move_arm_goal):
        position_constraint, orientation_constraint = self.poseConstraintToPositionOrientationConstraints(pose_constraint);
        move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)
        move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

    # called by chess executive
    def execute(self, move, board, nested=False):

        rospy.loginfo('ArmPlanner execute')
        print "ArmPlanner execute"
        """ Execute a move. """

        # untuck arm
        # ARD ARD ARD
        # self.tuck_server.untuck()
        rospy.sleep(3.0)

        # get info about move        
        (col_f, rank_f) = board.toPosition(move[0:2])
        (col_t, rank_t) = board.toPosition(move[2:])
        fr = board.getPiece(col_f, rank_f)
        to = board.getPiece(col_t, rank_t)

        #req = MoveArmRequest()
        #req.header.frame_id = fr.header.frame_id
        goal = MoveArmGoal()
        #goal.header.frame_id = fr.header.frame_id
        # see move_arm/test/regression_test_simple_pose.cpp
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"

        # is this a capture?
        if to != None: 
            off_board = ChessPiece()
            off_board.header.frame_id = fr.header.frame_id
            off_board.pose.position.x = -2 * SQUARE_SIZE
            off_board.pose.position.y = SQUARE_SIZE
            off_board.pose.position.z = fr.pose.position.z
            self.addTransit(goal, to.pose, off_board.pose)
        
        to = ChessPiece()
        to.header.frame_id = fr.header.frame_id
        print "Execute "
        print to.header.frame_id
        print fr.header.frame_id
        to.pose = self.getPose(col_t, rank_t, board, fr.pose.position.z)

        # self.addTransit(goal, fr.pose, to.pose)
        self.addTransit(goal, fr, to)
        self.execute_goal(goal, move, nested, board, to)
        return to.pose
        

    def execute_goal(self, goal, move, nested, board, to):
        # execute
        try:
            #self.success = self.move(req)
            #print self.success
            self.move_arm_client.send_goal(goal)
            self.move_arm_client.wait_for_result()   
            print self.move_arm_client.get_result()
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)

        if move in castling_extras:
            self.execute(castling_extras[move],board)

        if not nested:
            # tuck arm
            # ARD: uncomment tuck_server later
            # self.tuck_server.tuck()
            rospy.sleep(5.0)
        return to.pose


    def addTransit(self, goal, fr, to):
        """ Move a piece from 'fr' to 'to' """

        rospy.loginfo('ArmPlanner addTransit')
        # rospy.wait_for_service("cartesian_planner/plan_cartesian_path")

        #header = std_msgs.msg.Header()
        #header.frame_id = "base_link"
        #header.frame_id = "right_arm_shelf_link"
        #print "addTransit : base_link" 
        rospy.loginfo('addTransit right_arm_shelf_link')
        #header.stamp = rospy.get_rostime()
        
        print "MotionPlanRequest" 
        motion_plan_request = arm_navigation_msgs.msg.MotionPlanRequest()
        motion_plan_request.group_name = "right arm"
        motion_plan_request.num_planning_attempts = 1000;
        motion_plan_request.allowed_planning_time =  rospy.Duration(5.)
        motion_plan_request.planner_id = ""

        joint_state_message = rospy.wait_for_message("/joint_states", JointState)
        print "joint_state_message" 
    
        motion_plan_request.start_state.joint_state = joint_state_message
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        # pose = Pose()
        pose = SimplePoseConstraint()
        pose.header.frame_id = "right_arm_shelf_link";
        #pose.header.frame_id = "base_link";
        pose.link_name = "right_wrist_roll_link"

        print "header frame_id" 
        print pose.header.frame_id
        pose.pose.position.x = fr.pose.position.x
        pose.pose.position.y = fr.pose.position.y
        pose.pose.position.z = 0.15
        # fr.position.z + 0.1
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.absolute_position_tolerance.x = 0.05;
        pose.absolute_position_tolerance.y = 0.05;
        pose.absolute_position_tolerance.z = 0.05;
        pose.absolute_roll_tolerance = 0.2;
        pose.absolute_pitch_tolerance = 0.2;
        pose.absolute_yaw_tolerance = 0.2;

        # hover over piece
        # action = ArmAction()
        # action.type = ArmAction.MOVE_ARM
        # action.move_time = rospy.Duration(2.5)

        # ARD: pose.header.link_name
        self.addGoalConstraintToMoveArmGoal(pose, goal)

        print "send_goal1"
        self.move_arm_client.send_goal(goal)
        print "wait for result"
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        print self.move_arm_client.get_result()

        # goal.motions.append(action)
        # open gripper
        # ARD
        # /parallel_gripper_controller/command
        # gripper_open = 0.042;
        # gripper_closed = 0.024;
        grippergoal = Pr2GripperCommandGoal()
        grippergoal.command.position = 0.08
        grippergoal.command.max_effort = -1 # open fast.
        self.gripper.send_goal(grippergoal)
        self.gripper.wait_for_result()
        print self.move_arm_client.get_result()
        #time.sleep(5)
        rospy.sleep(5)

        # action = ArmAction()
        # action.type = ArmAction.MOVE_GRIPPER
        # action.command = GRIPPER_OPEN
        # action.move_time = rospy.Duration(1.0)
        # goal.motions.append(action)

        # lower gripper
        pose = SimplePoseConstraint()
        # action = MoveArmGoal()
        # action = ArmAction()
        # action.type = ArmAction.MOVE_ARM
        # ARD
        # pose.header.link_name = "/base_link"
        pose.pose.position.x = fr.pose.position.x
        pose.pose.position.y = fr.pose.position.y
        pose.pose.position.z = fr.pose.position.z + 0.03
        if pose.pose.position.z > 0.05:
            pose.pose.position.z = 0.05
        #pose.goal.position.z = 0.05 # 0.035
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.absolute_position_tolerance.x = 0.05;
        pose.absolute_position_tolerance.y = 0.05;
        pose.absolute_position_tolerance.z = 0.05;
        pose.absolute_roll_tolerance = 0.2;
        pose.absolute_pitch_tolerance = 0.2;
        pose.absolute_yaw_tolerance = 0.2;
        # action.move_time = rospy.Duration(1.5)
        # goal.motions.append(action)
        self.addGoalConstraintToMoveArmGoal(pose, goal)
        self.move_arm_client.send_goal(goal)
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        print self.move_arm_client.get_result()

        # close gripper
        grippergoal = Pr2GripperCommandGoal()
        grippergoal.command.position = 0.0
        grippergoal.command.max_effort = 50 # close slowly
        self.gripper.send_goal(grippergoal)
        self.gripper.wait_for_result()
        print self.move_arm_client.get_result()
        # action = ArmAction()
        # action.type = ArmAction.MOVE_GRIPPER
        # action.command = GRIPPER_CLOSE
        # action.move_time = rospy.Duration(3.0)
        # goal.motions.append(action)

        # raise gripper
        # action = ArmAction()
        # action.type = ArmAction.MOVE_ARM
        pose.pose.position.x = fr.pose.position.x
        pose.pose.position.y = fr.pose.position.y
        #action.pose.position.z = fr.pose.position.z + 0.1
        pose.pose.position.z = 0.15
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        # pose.move_time = rospy.Duration(1.0)
        #goal.motions.append(action)
        self.addGoalConstraintToMoveArmGoal(pose, goal)
        self.move_arm_client.send_goal(goal)
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        print self.move_arm_client.get_result()

        # over over goal
        # action = ArmAction()
        # action.type = ArmAction.MOVE_ARM
        pose.header.frame_id = to.header.frame_id
        # pose.header.link_name = to.header.link_name
        pose.pose.position.x = to.pose.position.x
        pose.pose.position.y = to.pose.position.y
        pose.pose.position.z = 0.15
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        # action.move_time = rospy.Duration(2.5)
        # goal.motions.append(action)
	self.addGoalConstraintToMoveArmGoal(pose, goal)
	self.move_arm_client.send_goal(goal)
	finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        print self.move_arm_client.get_result()

        # lower gripper
        # action = ArmAction()
        # action.type = ArmAction.MOVE_ARM
        pose.pose.position.x = to.pose.position.x
        pose.pose.position.y = to.pose.position.y
        pose.pose.position.z = 0.06
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        # action.move_time = rospy.Duration(1.5)
        #goal.motions.append(pose)
        self.addGoalConstraintToMoveArmGoal(pose, goal)
        self.move_arm_client.send_goal(goal)
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        print self.move_arm_client.get_result()
        
        # open gripper
        # action = ArmAction()
        # action.type = ArmAction.MOVE_GRIPPER
        # action.command = GRIPPER_OPEN
        # action.move_time = rospy.Duration(1.0)
        # goal.motions.append(action)
        grippergoal = Pr2GripperCommandGoal()
        grippergoal.command.position = 0.08
        grippergoal.command.max_effort = -1 # open fast.
        self.gripper.send_goal(grippergoal)
        self.gripper.wait_for_result()
        print self.move_arm_client.get_result()
        #time.sleep(5)
        rospy.sleep(5)

        # action = ArmAction()
        # action.type = ArmAction.MOVE_GRIPPER
        # action.command = GRIPPER_OPEN
        # action.move_time = rospy.Duration(1.0)
        # goal.motions.append(action)

        # lower gripper
        #action = SimplePoseConstraint()
        # action = ArmAction()
        # action.type = ArmAction.MOVE_ARM
        pose.pose.position.x = fr.pose.position.x
        pose.pose.position.y = fr.pose.position.y
        pose.pose.position.z = fr.pose.position.z + 0.03
        if pose.pose.position.z > 0.05:
            pose.pose.position.z = 0.05
        #pose.goal.position.z = 0.05 # 0.035
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.absolute_position_tolerance.x = 0.05;
        pose.absolute_position_tolerance.y = 0.05;
        pose.absolute_position_tolerance.z = 0.05;
        pose.absolute_roll_tolerance = 0.2;
        pose.absolute_pitch_tolerance = 0.2;
        pose.absolute_yaw_tolerance = 0.2;
        # action.move_time = rospy.Duration(1.5)
        #goal.motions.append(action)
        self.addGoalConstraintToMoveArmGoal(pose, goal)
        self.move_arm_client.send_goal(goal)
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        print self.move_arm_client.get_result()

        # close gripper
        grippergoal = Pr2GripperCommandGoal()
        grippergoal.command.position = 0.0
        grippergoal.command.max_effort = 50 # close slowly
        self.gripper.send_goal(grippergoal)
        self.gripper.wait_for_result()
        print self.move_arm_client.get_result()
        # action = ArmAction()
        # action.type = ArmAction.MOVE_GRIPPER
        # action.command = GRIPPER_CLOSE
        # action.move_time = rospy.Duration(3.0)
        # goal.motions.append(action)

        # raise gripper
        # action = ArmAction()
        # action.type = ArmAction.MOVE_ARM
        pose.pose.position.x = fr.pose.position.x
        pose.pose.position.y = fr.pose.position.y
        #pose.pose.position.z = fr.position.z + 0.1
        pose.pose.position.z = 0.15
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        # pose.move_time = rospy.Duration(1.0)
        #goal.motions.append(pose)
        self.addGoalConstraintToMoveArmGoal(pose, goal)
        self.move_arm_client.send_goal(goal)
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        print self.move_arm_client.get_result()

        # over over goal
        # action = ArmAction()
        
        # raise gripper
        # action = ArmAction()
        # action.type = ArmAction.MOVE_ARM
        pose.pose.position.x = to.pose.position.x
        pose.pose.position.y = to.pose.position.y
        pose.pose.position.z = 0.15
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        # action.move_time = rospy.Duration(1.0)
        # goal.motions.append(pose)
        self.move_arm_client.send_goal(goal)
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        print self.move_arm_client.get_result()

    def getPose(self, col, rank, board, z=0):
        """ Find the reach required to get to a position """
        rospy.loginfo('ArmPlanner getPose')
        p = Pose()
        if board.side == board.WHITE:
            # p.position.x = (col * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.x = (abs(col-4.5) * SQUARE_SIZE)
            p.position.y = ((rank-1) * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.z = z
        else:
            # p.position.x = ((7-col) * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.x = (abs(4.5-col) * SQUARE_SIZE)
            p.position.y = ((8-rank) * SQUARE_SIZE) + SQUARE_SIZE/2
            p.position.z = z
        return p

    def getReach(self, col, rank, board):
        rospy.loginfo('ArmPlanner getReach')
        """ Find the reach required to get to a position """
        ps = PoseStamped()
        # ps.header.frame_id = "chess_board" # ARD
        ps.header.frame_id = "chess_board_raw"
        ps.pose = self.getPose(board.getColIdx(col), int(rank), board)
        pose = self.listener.transformPose("right_shoulder_pan_link", ps)
        print "get reach : chess_board_raw to right_shoulder_pan_link"  
        x = pose.pose.position.x
        y = pose.pose.position.y
        reach = sqrt( (x*x) + (y*y) ) 
        print reach
        return reach

