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
#
# Authors: Antons Rebguns
#          Tom Walsh
#

import roslib; roslib.load_manifest('wubble_blocks')
import rospy

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan

from wubble_actions.msg import ErraticBaseGoal
from wubble_actions.msg import ErraticBaseAction
from wubble_actions.msg import SmartArmGoal
from wubble_actions.msg import SmartArmGripperGoal
from wubble_actions.msg import SmartArmAction
from wubble_actions.msg import SmartArmGripperAction

from dynamixel_controllers.srv import SetSpeed
from wubble_blocks.srv import ClassifyObject

class ObjectSwat:
    def __init__(self):
        self.all_objects = []
        self.current_obj_idx = -1
        self.swat_time = rospy.Time.now()

        self.min_dist = 10
        self.front_dist = 10
        self.avg_front_dist = 10
        self.min_front_dist = 10

        rospy.Subscriber('overhead_objects', PoseArray, self.update_object_positions)
        rospy.Subscriber('tilt_laser/scan', LaserScan, self.filter_scan)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        self.sh_pan_speed_srv = rospy.ServiceProxy('shoulder_pan_controller/set_speed', SetSpeed)
        self.classify_obj_srv = rospy.ServiceProxy('classify_object', ClassifyObject)

        self.base_client = SimpleActionClient('erratic_base_action', ErraticBaseAction)
        self.arm_client = SimpleActionClient('smart_arm_action', SmartArmAction)
        self.gripper_client = SimpleActionClient('smart_arm_gripper_action', SmartArmGripperAction)

        rospy.wait_for_service('shoulder_pan_controller/set_speed')
        rospy.wait_for_service('classify_object')

        self.base_client.wait_for_server()
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()

        rospy.loginfo('Connected to all action servers')

    def update_object_positions(self, msg):
        self.all_objects = msg.poses

    def filter_scan(self, scan):
        center_scan_index = (len(scan.ranges) + 1) / 2

        min_idx = center_scan_index - 50
        max_idx = center_scan_index + 50

        for dist in scan.ranges[min_idx:max_idx]:
            if dist < self.min_front_dist and dist > 0.05:
                self.min_front_dist = dist

    def tuck_arm_for_navigation(self):
        goal = SmartArmGripperGoal()
        goal.target_joints = [0.15, -0.15]

        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        result = self.gripper_client.get_result()

        if result.success:
            rospy.loginfo('Gripper in position')
        else:
            rospy.loginfo('Gripper positioning failed')

        goal = SmartArmGoal()
        goal.target_joints = [0.0, 1.77, 0.0, 0.0]
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result()
        result = self.arm_client.get_result()

        if result.success:
            rospy.loginfo('Arm tucked')
            return True
        else:
            rospy.loginfo('Failed to tuck arm')
            return False

    def go_to_object(self, idx):
        self.current_obj_idx = idx

        goal = ErraticBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.pose = self.all_objects[idx]
        goal.vicinity_range = 0.3

        self.base_client.send_goal(goal)
        self.base_client.wait_for_result()

        if self.base_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo('Robot is in front of the object')
            return True
        elif self.base_client.get_state() == GoalStatus.PREEMPTED:
            rospy.loginfo('Action pre-empted')
            return False
        else:
            rospy.loginfo('Action failed')
            return False

    def position_self(self):
        max_speed = 0.1
        cmd_vel = Twist()

        rospy.loginfo('LIDAR reports object is %f m away...', self.min_front_dist)

        # move in closer
        if self.min_front_dist > 0.24:
            range = abs(self.min_front_dist - 0.24)
            rospy.loginfo('Object is too far away, closing in %f m', range)
            cmd_vel.linear.x = max_speed
            cmd_vel.angular.z = 0.0

            r = rospy.Rate(15)
            et = rospy.Time.now() + rospy.Duration.from_sec(range / max_speed)

            while rospy.Time.now() < et:
                self.cmd_vel_pub.publish(cmd_vel)
                r.sleep()

            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
        # back away
        elif self.min_front_dist < 0.22:
            range = abs(self.min_front_dist - 0.22)
            rospy.loginfo('Object is too close, backing away %f m', range)
            cmd_vel.linear.x = -max_speed
            cmd_vel.angular.z = 0.0

            r = rospy.Rate(15)
            et = rospy.Time.now() + rospy.Duration.from_sec(range / max_speed)

            while rospy.Time.now() < et:
                self.cmd_vel_pub.publish(cmd_vel)
                r.sleep()

            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
        # quit, navigation failed?
        elif self.min_front_dist > 0.50:
            rospy.logwarn('Object is way out there, navigation should take care of traversing great distances')
        else:
            rospy.loginfo('Object is within reach, no need to move')

    def swat_object(self):
        self.tuck_arm_for_navigation()

        goal = SmartArmGoal()
        goal.target_joints = [0.93, -0.63, 0.0, 0.0]
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result()
        result = self.arm_client.get_result()

        if result.success:
            rospy.loginfo('Arm ready to swat')
        else:
            rospy.loginfo('Failed to position the arm for swatting')

        self.swat_time = rospy.Time.now()
        rospy.loginfo('Swatting action at ' + str(self.swat_time.secs) + '.' + str(self.swat_time.nsecs))

        self.sh_pan_speed_srv(3.0)

        goal.target_joints = [-0.93, -0.63, 0.0, 0.0]
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result()
        result = self.arm_client.get_result()

        if result.success:
            rospy.loginfo('Arm just swatted')
        else:
            rospy.loginfo('Arm failed to swat')

        self.sh_pan_speed_srv(1.17)
        self.tuck_arm_for_navigation()
        rospy.sleep(5)

    def classify_object(self):
        resp = self.classify_obj_srv(self.current_obj_idx, self.swat_time)
        rospy.loginfo('The object is %s', resp.category)

if __name__ == '__main__':
    try:
        rospy.init_node('object_swatter', anonymous=True)
        swat = ObjectSwat()
        r = rospy.Rate(0.2)

        while not rospy.is_shutdown():
            rospy.loginfo('There are %d objects in the world' % len(swat.all_objects))

            for idx in range(len(swat.all_objects)):
                if swat.tuck_arm_for_navigation():
                    if swat.go_to_object(idx):
                        swat.position_self()
                        swat.swat_object()
                        swat.classify_object()

            r.sleep()
    except rospy.ROSInterruptException:
        pass
