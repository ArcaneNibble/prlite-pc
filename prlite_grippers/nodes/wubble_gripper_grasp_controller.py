#! /usr/bin/env python

# Copyright (c) 2010, Antons Rebguns
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
#
# Author: Antons Rebguns
#

import roslib; roslib.load_manifest('wubble2_robot')
import rospy
from rospy.exceptions import ROSException

from actionlib import SimpleActionClient
from actionlib import SimpleActionServer

from std_msgs.msg import Float64
from object_manipulation_msgs.msg import GraspHandPostureExecutionAction
from object_manipulation_msgs.msg import GraspHandPostureExecutionGoal
from object_manipulation_msgs.srv import GraspStatus
from wubble2_gripper_controller.msg import WubbleGripperAction
from wubble2_gripper_controller.msg import WubbleGripperGoal
from dynamixel_hardware_interface.msg import JointState

class WubbleGripperGraspController:
    def __init__(self):
        self.object_presence_pressure_threshold = rospy.get_param('object_presence_pressure_threshold', 200.0)
        self.object_presence_opening_threshold = rospy.get_param('object_presence_opening_threshold', 0.02)
        
        gripper_action_name = rospy.get_param('gripper_action_name', 'wubble_gripper_command_action')
        self.gripper_action_client = SimpleActionClient('wubble_gripper_action', WubbleGripperAction)
#        
#        while not rospy.is_shutdown():
#            try:
#                self.gripper_action_client.wait_for_server(timeout=rospy.Duration(2.0))
#                break
#            except ROSException as e:
#                rospy.loginfo('Waiting for %s action' % gripper_action_name)
#            except:
#                rospy.logerr('Unexpected error')
#                raise
                
        rospy.loginfo('Using gripper action client on topic %s' % gripper_action_name)
        
        query_service_name = rospy.get_param('grasp_query_name', 'wubble_grasp_status')
        self.query_srv = rospy.Service(query_service_name, GraspStatus, self.process_grasp_status)
        
        posture_action_name = rospy.get_param('posture_action_name', 'wubble_gripper_grasp_action')
        self.action_server = SimpleActionServer(posture_action_name, GraspHandPostureExecutionAction, self.process_grasp_action, False)
        self.action_server.start()
        
        rospy.loginfo('wubble_gripper grasp hand posture action server started on topic %s' % posture_action_name)

    def process_grasp_action(self, msg):
        gripper_command = WubbleGripperGoal()
        
        if msg.goal == GraspHandPostureExecutionGoal.GRASP:
            rospy.loginfo('Received GRASP request')
#            if not msg.goal.grasp.position:
#                msg = 'wubble gripper grasp execution: position vector empty in requested grasp'
#                rospy.logerr(msg)
#                self.action_server.set_aborted(text=msg)
#                return
#                
            gripper_command.command = WubbleGripperGoal.CLOSE_GRIPPER
            gripper_command.torque_limit = 0.4
            gripper_command.dynamic_torque_control = True
            gripper_command.pressure_upper = 1900.0
            gripper_command.pressure_lower = 1800.0
        elif msg.goal == GraspHandPostureExecutionGoal.PRE_GRASP:
            rospy.loginfo('Received PRE_GRASP request')
            gripper_command.command = WubbleGripperGoal.OPEN_GRIPPER
            gripper_command.torque_limit = 0.6
        elif msg.goal == GraspHandPostureExecutionGoal.RELEASE:
            rospy.loginfo('Received RELEASE request')
            gripper_command.command = WubbleGripperGoal.OPEN_GRIPPER
            gripper_command.torque_limit = 0.6
        else:
            msg = 'wubble gripper grasp execution: unknown goal code (%d)' % msg.goal
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)
            
        self.gripper_action_client.send_goal(gripper_command)
        self.gripper_action_client.wait_for_result()
        self.action_server.set_succeeded()

    def process_grasp_status(self, msg):
        result = GraspStatus()
        pressure_msg = rospy.wait_for_message('/total_pressure', Float64)
        opening_msg = rospy.wait_for_message('/gripper_opening', Float64)
        left_pos = rospy.wait_for_message('/left_finger_controller/state', JointState).position
        right_pos = rospy.wait_for_message('/right_finger_controller/state', JointState).position
        
        if pressure_msg.data <= self.object_presence_pressure_threshold:
            rospy.loginfo('Gripper grasp query false: gripper total pressure is below threshold (%.2f <= %.2f)' % (pressure_msg.data, self.object_presence_pressure_threshold))
            return False
        else:
            if opening_msg.data <= self.object_presence_opening_threshold:
                rospy.loginfo('Gripper grasp query false: gripper opening is below threshold (%.2f <= %.2f)' % (opening_msg.data, self.object_presence_opening_threshold))
                return False
            else:
                if (left_pos > 0.85 and right_pos > 1.10) or (right_pos < -0.85 and left_pos < -1.10):
                    rospy.loginfo('Gripper grasp query false: gripper fingers are too off center (%.2f, %.2f)' % (left_pos, right_pos))
                    return False
                    
                rospy.loginfo('Gripper grasp query true: pressure is %.2f, opening is %.2f' % (pressure_msg.data, opening_msg.data))
                return True

if __name__ == '__main__':
    try:
        rospy.init_node('wubble_gripper_grasp_controller', anonymous=True)
        grasp_controller = WubbleGripperGraspController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

