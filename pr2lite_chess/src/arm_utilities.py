#!/usr/bin/env python

"""
Copyright (c) 2011 Michael E. Ferguson. All right reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
"""

import roslib; 
roslib.load_manifest('pr2lite_chess')
#roslib.load_manifest('pr2_controllers_msgs')
import rospy, actionlib
# from math import sqrt fabs
from math import sqrt
import math
import pexpect

import actionlib
from pr2_controllers_msgs.msg import (Pr2GripperCommandGoal, Pr2GripperCommand,
                                      Pr2GripperCommandAction)
from turtlebot_block_manipulation.msg import *
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pr2lite_arm_utilities import PR2Arm_Planning 


from chess_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped
# from simple_arm_server.msg import *
# from simple_arm_server.srv import *
from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *
from sensor_msgs.msg import JointState
from pr2_controllers_msgs.msg import *
import roslib


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
            self.listener = listener
            self.pr2_arm = PR2Arm_Planning('right', self.listener)
            #ARD: uncomment next line
            rospy.loginfo('ArmPlanner pr2lite_move_right_arm')
            arm = "right"
            self.move_arm_client = actionlib.SimpleActionClient('pr2lite_move_right_arm', MoveArmAction)
            if arm is "right":
              service = "r_gripper_controller/gripper_action"
            else:
              service = "l_gripper_controller/gripper_action"
            rospy.loginfo('Gripper wait_for_server')
            self.gripper = actionlib.SimpleActionClient(service, Pr2GripperCommandAction)
            self.gripper.wait_for_server()
            # rospy.init_node('single_joint_position', anonymous=True)
            self.tuck_server = tuck_arm()
            self.torso_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
            self.torso_client.wait_for_server()

            self.block_client = actionlib.SimpleActionClient('interactive_manipulation', InteractiveBlockManipulationAction)
            rospy.loginfo('ArmPlanner wait for interactive manip')
            self.block_client.wait_for_server()
            # self.block_client.block_size = 0.03 
        else:
            self.move_arm_client = client
     
        self.success = True
        # setup tf for translating poses
        rospy.loginfo('ArmPlanner Init done')

    # called by chess executive
    def execute(self, move, board, nested=False):

        rospy.loginfo('ArmPlanner execute')
        """ Execute a move. """

        # get info about move
        (col_f, rank_f) = board.toPosition(move[0:2])
        (col_t, rank_t) = board.toPosition(move[2:])
        fr = board.getPiece(col_f, rank_f)
        to = board.getPiece(col_t, rank_t)

        #req = MoveArmRequest()
        #req.header.frame_id = fr.header.frame_id
        # see move_arm/test/regression_test_simple_pose.cpp
        goal = MoveArmGoal()
        goal.planner_service_name = "ompl_planning/plan_kinematic_path"

        # is this a capture?
        if to != None:
            print "execute Capture"
            off_board = ChessPiece()
            off_board.header.frame_id = fr.header.frame_id
            off_board.pose.position.x = -2 * SQUARE_SIZE
            off_board.pose.position.y = SQUARE_SIZE
            off_board.pose.position.z = fr.pose.position.z
            self.addTransit(goal, to.pose, off_board.pose)
       
        to = ChessPiece()
        to.header.frame_id = fr.header.frame_id
        to.pose = self.getPose(col_t, rank_t, board, fr.pose.position.z)

        print "Execute FR"
        print fr
        print "Execute TO"
        print to
        fr.pose = self.getPose(col_f, rank_f, board, fr.pose.position.z)
        fr.header.stamp = rospy.Time.now()
        print "Execute FR2"
        print fr

        # self.addTransit(goal, fr.pose, to.pose)
        self.addTransit(goal, fr, to)
        return to.pose
       
    def move_torso(self, position):
        rospy.loginfo('move_torso %f' % position)
        if position < 0:
          position = 0
        goal = SingleJointPositionGoal()
        goal.position = position
        self.torso_client.send_goal(goal)
        self.torso_client.wait_for_result()

    def begin_game_pos(self):
        # self.tuck_server.left_tuck()
        self.tuck_server.untuck()
        rospy.sleep(5.0)
        self.move_torso(0)

    def picknplaceCB(self, success, result):
        rospy.loginfo('Pick and Place Callback')

    def addTransit(self, goal, fr, to):
        """ Move a piece from 'fr' to 'to' """

        rospy.loginfo('ArmPlanner addTransit')

        start_torso_pos = .30     # raise torso for easier planning
        above_board = .15     # raise torso for easier planning
        self.move_torso(start_torso_pos)
        self.tuck_server.IKpose()
        rospy.sleep(3.0)

        pose = SimplePoseConstraint()
        pose.link_name = "right_wrist_roll_link"

        # fr.header.frame_id = "chess_board_raw"
        self.listener.mutex.acquire()
        fr_tfpose = self.listener.transformPose("base_link", fr)
        self.listener.mutex.release()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = fr_tfpose.pose.position.x
        pose.pose.position.y = fr_tfpose.pose.position.y
        pose.pose.position.z = fr_tfpose.pose.position.z + above_board + start_torso_pos

        block_goal = InteractiveBlockManipulationGoal()
        block_goal.block_size = 0.03 
        block_goal.frame = "chess_board_raw"
        self.block_client.block_size = 0.03 
        self.block_client.frame = "chess_board_raw"
        self.block_client.pickup_pose = fr
        self.block_client.place_pose = to
        self.block_client.send_goal(block_goal, self.picknplaceCB)
        rospy.loginfo('Block goal')


        # q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
        # pose.pose.position.z = 0.15
        q = quaternion_from_euler(1.57, 0.0, 0.0, 'sxyz')

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        print "quat from euler"
        print pose.pose.orientation

        pose.pose.orientation.x = 0.359098912478
        pose.pose.orientation.y = 0.625686613067
        pose.pose.orientation.z = -0.330601968027
        pose.pose.orientation.w = 0.608495334429

        pose.absolute_position_tolerance.x = 0.05;
        pose.absolute_position_tolerance.y = 0.05;
        pose.absolute_position_tolerance.z = 0.05;
        pose.absolute_roll_tolerance = 0.06;
        pose.absolute_pitch_tolerance = 0.06;
        pose.absolute_yaw_tolerance = 0.06;

        # hover over piece
        traj = self.pr2_arm.build_trajectory(pose, None)
        goal = self.pr2_arm.build_follow_trajectory(traj)
        print "move to FROM pos"
        self.pr2_arm.follow_trajectory(goal)
        # self.move_arm_client.send_goal(goal)
        # finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(200.0))
        # print self.move_arm_client.get_result()

        print "open gripper"
        grippergoal = Pr2GripperCommandGoal()
        grippergoal.command.position = 0.08
        grippergoal.command.max_effort = -1 # open fast.
        self.gripper.send_goal(grippergoal)
        self.gripper.wait_for_result()
        rospy.sleep(5)
        rospy.sleep(50)

        # following looks wrong
        lower_to_piece = fr.pose.position.z + 0.03 + above_board
        print "lower torso to piece ", lower_to_piece
        self.move_torso(lower_to_piece)     # raise torso for easier planning

        # close gripper
        print "close gripper"
        grippergoal = Pr2GripperCommandGoal()
        grippergoal.command.position = 0.01
        grippergoal.command.max_effort = 50 # close slowly
        self.gripper.send_goal(grippergoal)
        self.gripper.wait_for_result()
        print self.move_arm_client.get_result()

        print "move to TO pos"
        to_tfpose = self.listener.transformPose("base_link", to)
        pose.header.frame_id = to_tfpose.header.frame_id
        pose.pose.position.x = to_tfpose.pose.position.x
        pose.pose.position.y = to_tfpose.pose.position.y
        pose.pose.position.z = to_tfpose.pose.position.z + above_board + start_torso_pos
        q = quaternion_from_euler(0.0, 1.57, 0.0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        pose.pose.orientation.x = 0.359098912478
        pose.pose.orientation.y = 0.625686613067
        pose.pose.orientation.z = -0.330601968027
        pose.pose.orientation.w = 0.608495334429
        traj = self.pr2_arm.build_trajectory(pose, None)
        goal = self.pr2_arm.build_follow_trajectory(traj)
        self.pr2_arm.follow_trajectory(goal)
        # self.move_arm_client.send_goal(goal)

        print "lower torso"
        self.move_torso(lower_to_piece)     # raise torso for easier planning
       
        print "open gripper"
        grippergoal = Pr2GripperCommandGoal()
        grippergoal.command.position = 0.08
        grippergoal.command.max_effort = -1 # open fast.
        self.gripper.send_goal(grippergoal)
        self.gripper.wait_for_result()
        # print self.move_arm_client.get_result()
        # rospy.sleep(5)

        self.move_torso(start_torso_pos)
        self.tuck_server.untuck()

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
        pose = self.listener.transformPose("right_arm_shelf_link", ps)
        # pose = self.listener.transformPose("right_shoulder_pan_link", ps)
        print "get reach : chess_board_raw to right_shoulder_pan_link"
        x = pose.pose.position.x
        y = pose.pose.position.y
        reach = sqrt( (x*x) + (y*y) )
        print reach
        return reach


def pose_relative_trans(pose, x=0., y=0., z=0.):
    """Return a pose translated relative to a given pose."""
    ps = deepcopy(pose)
    M_trans = tft.translation_matrix([x,y,z])
    q_ps = [ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]
    M_rot = tft.quaternion_matrix(q_ps)
    trans = np.dot(M_rot,M_trans)
    ps.pose.position.x += trans[0][-1]
    ps.pose.position.y += trans[1][-1]
    ps.pose.position.z += trans[2][-1]
    #print ps
    return ps

def pose_relative_rot(pose, r=0., p=0., y=0., degrees=True):
    """Return a pose rotated relative to a given pose."""
    ps = deepcopy(pose) 
    if degrees:
        r = math.radians(r)
        p = math.radians(p)
        y = math.radians(y)
    des_rot_mat = tft.euler_matrix(r,p,y) 
    q_ps = [ps.pose.orientation.x, 
            ps.pose.orientation.y, 
            ps.pose.orientation.z, 
            ps.pose.orientation.w]
    state_rot_mat = tft.quaternion_matrix(q_ps) 
    final_rot_mat = np.dot(state_rot_mat, des_rot_mat) 
    ps.pose.orientation = Quaternion(
                            *tft.quaternion_from_matrix(final_rot_mat))
    return ps

def find_approach(pose, standoff=0., axis='x'):
    """Return a PoseStamped pointed down the z-axis of input pose."""
    ps = deepcopy(pose)
    if axis == 'x':
        ps = pose_relative_rot(ps, p=90)
        ps = pose_relative_trans(ps, -standoff)
    return ps
    
def calc_dist(ps1, ps2):
    """ Return the cartesian distance between the points of 2 poses."""
    p1 = ps1.pose.position
    p2 = ps2.pose.position
    return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)

class PoseUtilsTest():
    def __init__(self):
        rospy.Subscriber('pose_in', PoseStamped, self.cb)
        self.recd_pub = rospy.Publisher('pose_utils_test_received', PoseStamped)
        self.trans_pub = rospy.Publisher('pose_utils_test_trans', PoseStamped)
        self.rot_pub = rospy.Publisher('pose_utils_test_rot', PoseStamped)
        self.ps = PoseStamped()
        self.ps.header.frame_id = '/torso_lift_link'
        self.ps.header.stamp = rospy.Time.now()
        self.ps.pose.position.x = 5
        self.ps.pose.position.y = 2
        self.ps.pose.position.z = 3

    def cb(self, ps):
        self.ps.pose.orientation = Quaternion(*tft.random_quaternion())
        print ps
        rospy.sleep(0.5)
        self.recd_pub.publish(ps)
        
        #trans_pose = pose_relative_trans(ps, 0.5, 0.5, 0.2)
        #self.trans_pub.publish(trans_pose)
        #rospy.loginfo("Pose Utils Test: Pose Translated: \n\r %s" %trans_pose)

        ps_rot = pose_relative_rot(ps, 90, 30 , 45)
        self.rot_pub.publish(ps_rot)
        rospy.loginfo("Pose Utils Test: Pose Rotated: \n\r %s" %ps_rot)

if __name__=='__main__':
    rospy.init_node('pose_utils_test')
    put = PoseUtilsTest()
    while not rospy.is_shutdown():
        put.cb(put.ps)
        rospy.sleep(5)
