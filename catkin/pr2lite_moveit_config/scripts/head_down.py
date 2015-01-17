#! /usr/bin/python
import roslib
# roslib.load_manifest('pr2_position_scripts')
roslib.load_manifest('pr2lite_moveit_config')

import rospy
import actionlib
from actionlib_msgs.msg import *
from control_msgs.msg import *
from geometry_msgs.msg import *

rospy.init_node('move_the_head', anonymous=True)

client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
client.wait_for_server()

g = PointHeadGoal()
g.target.header.frame_id = 'base_link'
# .75 looks good for arm_up
# .55 looks good for arm_down except lower arm is too long
# .60 gets the elbow right
# .65 gets the chess pieces right
g.target.point.x = 0.65
g.target.point.y = 0.0
g.target.point.z = 0.0
g.pointing_frame = "kinect_depth_optical_frame";
# (pointing_axis defaults to X-axis)
g.min_duration = rospy.Duration(1.0)

client.send_goal(g)
client.wait_for_result()

if client.get_state() == GoalStatus.SUCCEEDED:
  print "Succeeded"
else:
  print "Failed"
