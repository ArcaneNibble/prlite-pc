#!/usr/bin/python
import roslib
roslib.load_manifest('pr2lite_moveit_config')

import rospy
import actionlib
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal 
rospy.init_node('torso_down')
torso_action_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction);
rospy.loginfo("torso_down: waiting for torso action server")
torso_action_client.wait_for_server()
rospy.loginfo("torso_down: torso action server found")
goal = SingleJointPositionGoal()
goal.position = 0.0
rospy.loginfo("torso_down: sending command to lift torso")
torso_action_client.send_goal(goal)
torso_action_client.wait_for_result(rospy.Duration(30))
