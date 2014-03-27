#!/usr/bin/env python
import argparse
import roslib
roslib.load_manifest('velo_controller')
import rospy
import pr2_controllers_msgs.msg

import os


def setVELOPosition(desired_position, force, name_prefix):
	print "VELOPosition"
	if name_prefix == 'b': name_prefix = ['l','r']
	else:                  name_prefix = [name_prefix]
	for n in name_prefix:
		pub = rospy.Publisher('/%s_gripper_controller/command'%n, pr2_controllers_msgs.msg.Pr2GripperCommand, latch=True)
		rospy.init_node('velo_setpoint')
		rospy.sleep(0.3)	
		pub.publish(position = desired_position, max_effort=force)
		print "VELO set position: %.04f m, force %.02f N" % (desired_position, force)


if __name__ == '__main__':

	#os.environ['ROS_MASTER_URI'] = 'http://prq1:11311'

	parser = argparse.ArgumentParser(description='Set point and max force for a PR2 low cost gripper.')
	parser.add_argument('-f', type=float, help='Desired gripping force.', required=True)
	parser.add_argument('-p', type=float, help='Desired position setpoint (0.0 to 0.135 on the VELO).', required=True)
	parser.add_argument('-n', type=str, help='Change name prefix for gripper', default='l')

	args = parser.parse_args()

	try:
		setVELOPosition(args.p,args.f,args.n)
	except rospy.ROSInterruptException: 
		print "ROSInterruptException"
		pass
