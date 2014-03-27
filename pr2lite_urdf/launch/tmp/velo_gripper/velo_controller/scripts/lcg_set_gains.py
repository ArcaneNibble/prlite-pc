#!/usr/bin/env python
import argparse
import roslib
roslib.load_manifest('velo_controller')
import rospy
import pr2_controllers_msgs.msg

import os


controller_name = 'l_gripper_controller'

def setVELOGains(p, i, d, i_clamp):
	if p != None:
		rospy.set_param('/l_gripper_controller/pid/p', p)	
	if i != None:
		rospy.set_param('/l_gripper_controller/pid/i', i)
	if d != None:
		rospy.set_param('/l_gripper_controller/pid/d', d)
	if i_clamp != None:
		rospy.set_param('/l_gripper_controller/pid/i_clamp', i_clamp)

	rospy.init_node('velo_setgains')
	rospy.sleep(0.2)	
	
	os.system('rosrun pr2_controller_manager pr2_controller_manager kill %s' % controller_name)
	rospy.sleep(0.5)	
	os.system('rosrun pr2_controller_manager pr2_controller_manager spawn %s' % controller_name)
	
	print "Respawned controller \'%s\' with updated gains' % controller_name"

if __name__ == '__main__':

	os.environ['ROS_MASTER_URI'] = 'http://prq1:11311'

	parser = argparse.ArgumentParser(description='Set PID gains for the PR2 low cost gripper controller.')
	parser.add_argument('-p', type=float, help='P Gain.', required=False)
	parser.add_argument('-i', type=float, help='I Gain.', required=False)
	parser.add_argument('-d', type=float, help='D Gain.', required=False)	
	parser.add_argument('-ic', type=float, help='I Clamp (max integral component).', required=False)

	args = parser.parse_args()
	try:
		setVELOGains(args.p, args.i, args.d, args.ic)
	except rospy.ROSInterruptException: 
		print "ROSInterruptException"
		pass
