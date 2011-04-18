#!/usr/bin/env python

import roslib
roslib.load_manifest('prlite_mimic')
import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import *
from prlite_kinematics.msg import some_status_thing
import os

class moving_with_user(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['aborted', 'user_is_stopped', 'still_moving'])
		print "Hello, init move with user state"
		self.nav_stack_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.server_is_up = False
	
	def execute(self, userdata):
		print "Hello, moving with user"
		
		return 'user_is_stopped'
		
		if not self.server_is_up:
			print "Wait for server up"
			self.nav_stack_action_client.wait_for_server()
			self.server_is_up = True
			print "OK, server up"
		
		#find the person
		
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'odom'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 10.0
		goal.target_pose.pose.position.y = 0.0
		goal.target_pose.pose.position.z = 0.0
		goal.target_pose.pose.orientation.x = 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = 0.0
		goal.target_pose.pose.orientation.w = 1.0
		
		self.nav_stack_action_client.send_goal(goal)
		self.nav_stack_action_client.wait_for_result()

		state = self.nav_stack_action_client.get_state()
		
		if state == actionlib.GoalStatus.SUCCEEDED:
			print "ZOMG, it worked"
			#are we still moving
			if False:
				return 'user_is_stopped'
			else:
				return 'still_moving'
		else:
			return 'aborted'

class ni_waiting_for_user_calibration(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['aborted', 'found_user', 'user_moved'])
		self.is_calibrated = False
		self.subscriber = rospy.Subscriber('kinect_hack_status', some_status_thing, self.kinect_hack_callback)
		os.system("rosrun prlite_kinect KinectTransform &")
		os.system("roslaunch prlite_kinematics dual_armik.launch &")
		os.system("roslaunch smart_arm_controller prlite.launch &")
		os.system("roslaunch ax12_controller_core controller_manager.launch &")
		#FIXME: how do I shut down?
		print "Hello, init wait for user state"
	
	def kinect_hack_callback(self,status):
		if status.lulz == 0:
			print "kinect stuff has launched"
		if status.lulz == 1:
			print "user is calibrated!"
			self.is_calibrated = True
	
	def execute(self, userdata):
		print "Hello, waiting for calibration"
		while not self.is_calibrated:
			pass
		return 'found_user'

class mimicking_arms(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['aborted', 'lost_user'])
		self.lost_user = False
		self.subscriber = rospy.Subscriber('kinect_hack_status', some_status_thing, self.kinect_hack_callback)
		print "Hello, init mimic arms state"
	
	def kinect_hack_callback(self,status):
		if status.lulz == 2:
			print "user is lost!"
			self.lost_user = True
	
	def execute(self, userdata):
		print "Hello, mimicking arms"
		while not self.lost_user:
			pass
		return 'lost_user'

def main():
	rospy.init_node('prlite_mimic')
	
	sm = smach.StateMachine(outcomes=['aborted'])
	
	with sm:
		smach.StateMachine.add('moving_with_user', moving_with_user(), transitions={'aborted':'aborted', 'user_is_stopped':'ni_waiting_for_user_calibration', 'still_moving':'moving_with_user'})
		smach.StateMachine.add('ni_waiting_for_user_calibration', ni_waiting_for_user_calibration(), transitions={'aborted':'aborted', 'found_user':'mimicking_arms', 'user_moved':'moving_with_user'})
		smach.StateMachine.add('mimicking_arms', mimicking_arms(), transitions={'aborted':'aborted', 'lost_user':'moving_with_user'})
	
	sis = smach_ros.IntrospectionServer('prlite_minic_introspection', sm, '/SM_ROOT')
	sis.start()
	outcome = sm.execute()
	
	rospy.spin()
	sis.stop()

if __name__=='__main__':
	main()
	
	os.system("killall armik_node")
