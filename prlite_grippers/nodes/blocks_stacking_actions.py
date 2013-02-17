#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Anh Tran

PKG = 'wubble_blocks'
NAME = 'actions_demo'

import roslib; roslib.load_manifest(PKG)
import rospy

from actionlib import SimpleActionClient
from geometry_msgs.msg import PointStamped
from wubble_actions.msg import *


def move_head(head_pan, head_tilt):
    goal = WubbleHeadGoal()
    goal.target_joints = [head_pan, head_tilt]

    head_client.send_goal(goal)
    head_client.wait_for_result()

    result = head_client.get_result()
    if result.success == False:
        print "Action failed"
    else:
        print "Result: [" + str(result.head_position[0]) + ", " + str(result.head_position[1]) + "]"


def look_at(frame_id, x, y, z):
    goal = WubbleHeadGoal()
    goal.target_point = PointStamped()
    goal.target_point.header.frame_id = frame_id
    goal.target_point.point.x = x
    goal.target_point.point.y = y
    goal.target_point.point.z = z

    head_client.send_goal(goal)
    head_client.wait_for_result()

    result = head_client.get_result()
    if result.success == False:
        print "Action failed"
    else:
        print "Result: [" + str(result.head_position[0]) + ", " + str(result.head_position[1]) + "]"


def move_arm(shoulder_pan, shoulder_tilt, elbow_tilt, wrist_rotate):
    goal = SmartArmGoal()
    goal.target_joints = [shoulder_pan, shoulder_tilt, elbow_tilt, wrist_rotate]

    arm_client.send_goal(goal)
    arm_client.wait_for_result()

    result = arm_client.get_result()
    if result.success == False:
        print "Action failed"
    else:
        print "Result: [" + str(result.arm_position[0]) + ", " + str(result.arm_position[1]) + \
            str(result.arm_position[2]) + ", " + str(result.arm_position[3]) + "]"


def reach_at(frame_id, x, y, z):
    goal = SmartArmGoal()
    goal.target_point = PointStamped()
    goal.target_point.header.frame_id = frame_id
    goal.target_point.point.x = x
    goal.target_point.point.y = y
    goal.target_point.point.z = z

    arm_client.send_goal(goal)
    arm_client.wait_for_result()

    result = arm_client.get_result()
    if result.success == False:
        print "Action failed"
    else:
        print "Result: [" + str(result.arm_position[0]) + ", " + str(result.arm_position[1]) + \
            str(result.arm_position[2]) + ", " + str(result.arm_position[3]) + "]"


def move_gripper(left_finger, right_finger):
    goal = SmartArmGripperGoal()
    goal.target_joints = [left_finger, right_finger]

    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()

    result = gripper_client.get_result()
    if result.success == False:
        print "Action failed"
    else:
        print "Result: [" + str(result.gripper_position[0]) + ", " + str(result.gripper_position[1]) + "]"


def tilt_laser(n=1):
    goal = HokuyoLaserTiltGoal()
    goal.tilt_cycles = n
    goal.amplitude = 0.685
    goal.offset = 0.0
    goal.duration = 1.0

    laser_client.send_goal(goal)
    laser_client.wait_for_result()

    result = laser_client.get_result()
    if result.success == False:
        print "Action failed"
    else:
        print "Result: " + str(result.tilt_position)



if __name__ == '__main__':
    try:
        rospy.init_node(NAME, anonymous=True)
        head_client = SimpleActionClient("wubble_head_action", WubbleHeadAction)
        arm_client = SimpleActionClient("smart_arm_action", SmartArmAction)
        gripper_client = SimpleActionClient("smart_arm_gripper_action", SmartArmGripperAction)
        laser_client = SimpleActionClient('hokuyo_laser_tilt_action', HokuyoLaserTiltAction)
        head_client.wait_for_server()
        arm_client.wait_for_server()
        gripper_client.wait_for_server()
        laser_client.wait_for_server()

        print "Starting blocks-stacking actions."

        print "Laser tilt"
        tilt_laser(2)
        print "Look at blocks"
        look_at("/arm_base_link", 0.2825, 0.0, -0.025)
        print "Open gripper"
        move_gripper(0.2, -0.2);
        print "Reach right stack"
        reach_at("/arm_base_link", 0.2, -0.2, -0.040)
        rospy.sleep(0.5)
        print "Close gripper"
        move_gripper(-0.075, 0.075)
        print "Raise arm"
        move_arm(0.0, 0.75, -1.972222, 0.0)
        print "Reach left stack"
        reach_at("/arm_base_link", 0.2, 0.2, -0.030)
        rospy.sleep(0.5)
        print "Open gripper"
        move_gripper(0.2, -0.2)
        print "Raise arm"
        move_arm(0.0, 0.75, -1.972222, 0.0)
        print "Reach middle stack"
        reach_at("/arm_base_link", 0.28, 0.0, -0.040)
        rospy.sleep(0.5)
        print "Close gripper"
        move_gripper(-0.075, 0.075)
        print "Raise arm"
        move_arm(0.0, 0.75, -1.972222, 0.0)
        print "Reach left stack"
        reach_at("/arm_base_link", 0.2, 0.205, 0.035)
        rospy.sleep(0.5)
        print "Open gripper"
        move_gripper(0.2, -0.2)
        print "Reset arm"
        move_arm(0.0, 1.972222, -1.972222, 0.0)

        print "Blocks-stacking actions completed."

    except rospy.ROSInterruptException:
        pass

