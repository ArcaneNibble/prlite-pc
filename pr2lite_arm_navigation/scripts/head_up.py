#! /usr/bin/python
import roslib
roslib.load_manifest('pr2lite_arm_navigation')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *

rospy.init_node('move_the_head', anonymous=True)
client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
client.wait_for_server()

g = PointHeadGoal()
#g.target.header.frame_id = 'kinect_depth_optical_frame'
g.target.header.frame_id = 'kinect_link'
g.target.point.x = 1.0
g.target.point.y = 0.0
g.target.point.z = 0.0
g.pointing_frame = "kinect_depth_optical_frame";
g.min_duration = rospy.Duration(1.0)

client.send_goal(g)
client.wait_for_result()

if client.get_state() == GoalStatus.SUCCEEDED:
  print "Succeeded"
else:
  print "Failed"
